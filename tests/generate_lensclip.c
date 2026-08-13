/*
 * generate_lensclip.c
 *
 *  A synthetic clip for the image-level lens correction test: a checkerboard
 *  whose two tones swap inside every other radial band, distorted by a known
 *  lens and shaken by a known camera path.
 *
 *  The scene is an analytic function of continuous image coordinates rather
 *  than a painted bitmap, and every frame is evaluated directly at its own
 *  final coordinates.  No source image is ever resampled to build the footage,
 *  so the clip carries no generation blur and the ground truth for the round
 *  trip is the pattern function itself.
 *
 *  See docs/lens-distortion.md, "Seeing it: the checkerboard round trip".
 */

#include "lensdistortion.h"

#define LC_WIDTH   640
#define LC_HEIGHT  480
#define LC_CELL    32.0   /* checkerboard cell size in px: 20x15 cells       */
#define LC_BAND    61.0   /* radial inversion period in px; half the frame
                             diagonal is 400, so ~6 boundaries cross the
                             picture.

                             MUST stay coprime with LC_CELL, not merely a
                             non-divisor: a band circle tangent to a cell line
                             leaves a lune of opposite tone thinner than a
                             subsample, which aliases badly.  Distances from
                             the centre to vertical cell lines run 0 (mod 32)
                             and to horizontal ones 16 (mod 32), and with
                             gcd = 1 the first coincidence of either family
                             sits at r = 976, well outside the 400 px the
                             picture reaches.                               */
#define LC_SS      4      /* supersamples per axis per pixel                 */

/* Both tones are coloured, not grey: the PPM dumps double as the figure in
   docs/lens-distortion.md.  Their luma differs enough (~57 vs ~200) that the
   clip would also clear motion detection's contrast threshold, though nothing
   here depends on that. */
static const uint8_t LC_TONE[2][3] = {
  { 25,  35,  90},   /* dark blue  */
  {240, 205,  90}    /* light amber */
};

static void lcToneRGB(int tone, uint8_t* rgb){
  rgb[0] = LC_TONE[tone][0];
  rgb[1] = LC_TONE[tone][1];
  rgb[2] = LC_TONE[tone][2];
}

/* Which of the two tones the ideal scene shows at continuous point (x,y).

   The checkerboard term gives straight lines, which distortion visibly bends
   -- but only far from the centre, where the bend has room to accumulate.
   The band term gives circles, which distortion moves purely radially: what
   changes is the *spacing* between successive boundaries, compressed at the
   edge under barrel and stretched under pincushion.  That signal is present
   at every radius, including where the line bending is invisible. */
static int lcPatternTone(double x, double y){
  double dx = x - LC_WIDTH/2.0;
  double dy = y - LC_HEIGHT/2.0;
  int cell = ((int)floor(x/LC_CELL) + (int)floor(y/LC_CELL)) & 1;
  int band = ((int)floor(sqrt(dx*dx + dy*dy)/LC_BAND)) & 1;
  return cell ^ band;
}

/* A backward map from destination pixel (xd,yd) to the scene point it shows.
   ctx carries whatever the particular map needs; it is const because no map
   here mutates it. */
typedef void (*LCPointMap)(const void* ctx, double xd, double yd,
                           double* xs, double* ys);

static void lcIdentityMap(const void* ctx, double xd, double yd,
                          double* xs, double* ys){
  (void)ctx;
  *xs = xd; *ys = yd;
}

/* Renders pattern(map(x,y)) into frame, averaging LC_SS x LC_SS subsamples
   over each destination pixel's footprint so that edges are antialiased
   instead of stair-stepped.

   The supersampling is done in DESTINATION space and each subsample is mapped
   individually, which is what keeps the antialiasing correct where the map
   compresses the picture -- averaging in scene space would not.

   Pixel (x,y) covers [x-0.5, x+0.5) x [y-0.5, y+0.5); the render path's own
   loops treat integer index x as coordinate x (transformfixedpoint.c:335,
   transformPacked -- the fixed-point path, which is what these tests
   actually execute; see the note on lcBackwardAffine below), so the two
   conventions agree. */
/* Which tone the scene shows at a continuous point.  lcPatternTone is the one
   this file's clip uses; generate_fovclip.c supplies an aperiodic variant for
   the sequences that have to survive block matching. */
typedef int (*LCToneFn)(double x, double y);

static void lcRenderMappedTone(VSFrame* frame, const VSFrameInfo* fi,
                               LCPointMap map, const void* ctx, LCToneFn tone){
  int x, y, i, j;
  for(y=0; y<fi->height; y++){
    for(x=0; x<fi->width; x++){
      int acc[3] = {0,0,0};
      for(j=0; j<LC_SS; j++){
        for(i=0; i<LC_SS; i++){
          double sx, sy;
          uint8_t rgb[3];
          double ox = x + (i + 0.5)/LC_SS - 0.5;
          double oy = y + (j + 0.5)/LC_SS - 0.5;
          map(ctx, ox, oy, &sx, &sy);
          lcToneRGB(tone(sx, sy), rgb);
          acc[0] += rgb[0]; acc[1] += rgb[1]; acc[2] += rgb[2];
        }
      }
      setPixelRGB(frame, fi, x, y,
                  (uint8_t)((acc[0] + LC_SS*LC_SS/2)/(LC_SS*LC_SS)),
                  (uint8_t)((acc[1] + LC_SS*LC_SS/2)/(LC_SS*LC_SS)),
                  (uint8_t)((acc[2] + LC_SS*LC_SS/2)/(LC_SS*LC_SS)));
    }
  }
}

static void lcRenderMapped(VSFrame* frame, const VSFrameInfo* fi,
                           LCPointMap map, const void* ctx){
  lcRenderMappedTone(frame, fi, map, ctx, lcPatternTone);
}

#define LC_NUM_FRAMES 6
#define LC_K_BARREL (-0.25)   /* the clip that gets dumped as the figure */
#define LC_K_PIN      0.15    /* assertion-only second pass             */

/* The camera path.  Bounded and drift free -- |x| <= 12, |y| <= 9,
   |alpha| <= 1.2 deg -- so the scene never leaves the canvas and the valid
   region stays large.  Every term vanishes at i == 0, deliberately: frame 0 is
   then the base scene seen through the lens and nothing else, which is what
   makes it the frame worth putting in the figure and what lets the Wobble
   assertion compare against it directly. */
static VSTransform lcClipTransform(int i){
  double s = (double)i;
  VSTransform t = null_transform();
  t.x     = 12.0 * sin(s * 0.7);
  t.y     =  9.0 * sin(s * 0.9);
  t.alpha =  1.2 * sin(s * 0.5) * M_PI / 180.0;
  t.zoom  = 0;
  return t;
}

/* The render path's own backward affine, transcribed from
   transformfixedpoint.c:279-361 (transformPacked) for a plane at full luma
   resolution and equal source and destination sizes:

     z      = 1 - t.zoom/100
     M_t(x) = z * A(alpha) * (x - c) + c - (t.x, t.y),  A the CCW rotation

   The code spells the rotation as cos(-alpha)/sin(-alpha) with the signs of
   the second row flipped, which is the same matrix.  Centre is w/2, NOT
   (w-1)/2, matching c_d_x there.

   Under -DTESTING the float entry points are renamed with a _float suffix
   (transformfloat.h), so vsDoTransform reaches the fixed-point path and that
   is what these tests exercise.  The two paths are kept in step by
   test_lensmap.c's fixed/float equivalence tests, not by this file. */
static void lcBackwardAffine(const VSTransform* t, double xd, double yd,
                             double* xs, double* ys){
  double cx = LC_WIDTH/2.0, cy = LC_HEIGHT/2.0;
  double z  = 1.0 - t->zoom/100.0;
  double ca = cos(t->alpha), sa = sin(t->alpha);
  double dx = xd - cx, dy = yd - cy;
  *xs = z*(ca*dx - sa*dy) + cx - t->x;
  *ys = z*(sa*dx + ca*dy) + cy - t->y;
}

/* The transform whose backward affine is the inverse of t's.

   From M_t(x) = A(a)(x-c) + c - d  (with zoom 0, so z = 1),
        M_t^-1(y) = A(-a)((y-c) + d) + c = A(-a)(y-c) + c + A(-a)d,
   which is M_t' with alpha' = -a and d' = -A(-a)d.

   Only the zoom-free case is implemented, because lcClipTransform never sets
   zoom; the general case would additionally need z' = 1/z and
   zoom' = 100(1 - 1/z).  Asserting rather than silently mis-inverting. */
static VSTransform lcInverseTransform(VSTransform t){
  VSTransform r = null_transform();
  double ca = cos(-t.alpha), sa = sin(-t.alpha);
  test_bool(t.zoom == 0);
  r.alpha = -t.alpha;
  r.x = -( ca*t.x - sa*t.y);
  r.y = -( sa*t.x + ca*t.y);
  r.zoom = 0;
  return r;
}

/* Context for the clip's backward map: which pose, and which lens. */
typedef struct {
  VSLensDistortion ld;
  VSTransform      t;
  int              useLens;   /* 0 leaves U_k out, for the k == 0 case */
} LCClipMapCtx;

/* frame_i(x) = pattern( S_i( U_k(x) ) ): undistort the destination point to
   where an ideal lens would have put it, then move it by the camera pose.

   U_k comes from lensdistortion.c, which test_lensdistortion.c covers
   independently.  vsLensPlaneMapInit and its lookup tables are deliberately
   NOT used here -- building the footage with the same tables the round trip
   then checks would only confirm that the tables agree with themselves.

   Points outside the model's domain cannot occur for k <= 0, and for the
   pincushion pass they would sit far outside the frame; the pattern is
   defined everywhere, so the fallback simply leaves the point where it is. */
static void lcClipMap(const void* ctx, double xd, double yd,
                      double* xs, double* ys){
  const LCClipMapCtx* c = (const LCClipMapCtx*)ctx;
  double ux = xd, uy = yd;
  if(c->useLens)
    if(vsLensUndistortPoint(&c->ld, xd, yd, &ux, &uy) != VS_OK){ ux = xd; uy = yd; }
  lcBackwardAffine(&c->t, ux, uy, xs, ys);
}

/* Allocates and fills numFrames frames of the clip.  Caller frees them. */
static void lcGenerateClip(VSFrame* frames, VSFrameInfo* fi, VSPixelFormat pf,
                           double k, int numFrames){
  int i;
  test_bool(vsFrameInfoInit(fi, LC_WIDTH, LC_HEIGHT, pf) != 0);
  for(i=0; i<numFrames; i++){
    LCClipMapCtx ctx;
    ctx.ld      = vsLensDistortionInit(fi, k);
    ctx.t       = lcClipTransform(i);
    ctx.useLens = (k != 0.0);
    vsFrameAllocate(&frames[i], fi);
    lcRenderMapped(&frames[i], fi, lcClipMap, &ctx);
  }
}
