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
 *  See docs/superpowers/specs/2026-08-09-lens-checkerboard-footage-design.md
 */

#define LC_WIDTH   640
#define LC_HEIGHT  480
#define LC_CELL    32.0   /* checkerboard cell size in px: 20x15 cells       */
#define LC_BAND    60.0   /* radial inversion period in px; half the frame
                             diagonal is 400, so ~6 boundaries cross the
                             picture.  Not a divisor or multiple of LC_CELL,
                             so band and cell edges stay distinguishable.    */
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
   loops treat integer index x as coordinate x (transformfloat.c:378), so the
   two conventions agree. */
static void lcRenderMapped(VSFrame* frame, const VSFrameInfo* fi,
                           LCPointMap map, const void* ctx){
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
          lcToneRGB(lcPatternTone(sx, sy), rgb);
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
