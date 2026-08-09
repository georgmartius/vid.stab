/*
 * test_lenscorrect_roundtrip.c
 *
 *  Image-level test of the render path's lens correction: a synthetic
 *  checkerboard clip is distorted by a known k, corrected back, and compared
 *  against the analytic original.  The generator lives in generate_lensclip.c.
 *
 *  See docs/superpowers/specs/2026-08-09-lens-checkerboard-footage-design.md
 */

/* --- the pattern ---------------------------------------------------------- */

/* Radial band index at a point, used to state the preconditions the cell
   assertions below depend on. */
static int lcBandIndexAt(double x, double y){
  double dx = x - LC_WIDTH/2.0, dy = y - LC_HEIGHT/2.0;
  return (int)floor(sqrt(dx*dx + dy*dy)/LC_BAND);
}

/* Cell centres alternate tone with both indices, and the tone flips again in
   every other radial band.  Checked at hand-computed points so a sign error in
   either term cannot hide behind the other. */
static void test_lenscorrect_pattern_tones(void){
  double cx = LC_WIDTH/2.0, cy = LC_HEIGHT/2.0;
  int t00, t10, t01;

  fprintf(stderr, "--- lens clip pattern: cells and bands ---\n");

  /* Cell parity, read off a 2x2 block of cells lying WHOLLY inside one radial
     band -- otherwise the band term flips underneath the comparison and the
     cell structure cannot be isolated.  A 2x2 block spans 32..45 px of radius
     while a band is only 60 px wide, so the block has to be placed on purpose:
     (328, 425) and its +LC_CELL neighbours have radii 185.2 .. 220.7, all
     inside band 3 = [180, 240).  It is also clear of every cell boundary
     (328/32 = 10.25, 425/32 = 13.28) and of the frame edge (max 457 < 480).

     The single-band precondition is asserted rather than trusted, so retuning
     LC_CELL or LC_BAND fails loudly here instead of silently gutting the
     test. */
  {
    double bx = 328.0, by = 425.0;
    int b = lcBandIndexAt(bx, by);
    test_bool(lcBandIndexAt(bx + LC_CELL, by)           == b);
    test_bool(lcBandIndexAt(bx,           by + LC_CELL) == b);
    test_bool(lcBandIndexAt(bx + LC_CELL, by + LC_CELL) == b);

    t00 = lcPatternTone(bx,           by);
    t10 = lcPatternTone(bx + LC_CELL, by);
    t01 = lcPatternTone(bx,           by + LC_CELL);
    test_bool(t10 != t00);
    test_bool(t01 != t00);
    test_bool(lcPatternTone(bx + LC_CELL, by + LC_CELL) == t00);

    /* Deterministic and side-effect free. */
    test_bool(lcPatternTone(bx, by) == t00);
  }

  /* Bands.  Two points in the same cell column and row parity but either side
     of the first band boundary (r = LC_BAND) must differ, purely from the band
     term.  Walk along +x from the centre, staying inside one cell by choosing
     LC_BAND to fall inside a cell -- assert that precondition too, so the test
     fails loudly rather than silently if LC_CELL or LC_BAND is retuned. */
  {
    double rIn = LC_BAND - 2.0, rOut = LC_BAND + 2.0;
    test_bool(floor((cx + rIn)/LC_CELL) == floor((cx + rOut)/LC_CELL));
    test_bool(lcPatternTone(cx + rIn, cy + 0.5) !=
              lcPatternTone(cx + rOut, cy + 0.5));
  }
}

/* A pixel wholly inside one cell must come out as that exact tone; a pixel
   straddling a cell edge must come out strictly between the two tones.  That
   is what says the supersampling actually runs -- with LC_SS == 1 the second
   assertion fails. */
static void test_lenscorrect_pattern_supersampling(void){
  VSFrameInfo fi;
  VSFrame f;
  uint8_t rgb[3], want[3];
  int xEdge, yFlat;

  fprintf(stderr, "--- lens clip pattern: supersampled edges ---\n");

  test_bool(vsFrameInfoInit(&fi, LC_WIDTH, LC_HEIGHT, PF_RGB24) != 0);
  vsFrameAllocate(&f, &fi);
  lcRenderMapped(&f, &fi, lcIdentityMap, NULL);

  /* x = 128 is a cell boundary (a multiple of LC_CELL), and the pixel there
     covers [127.5, 128.5) so it straddles.  x = 144, y = 144 sits in the
     interior of one cell in both axes, and its radius from the centre (200.5)
     is 20 px clear of the nearest band boundary at 180, so neither term is
     near a transition.  lcRenderMapped treats integer index x as coordinate
     x, so the pixel centre IS 144.0 -- no half-pixel offset when asking the
     pattern what should be there. */
  xEdge = (int)(4*LC_CELL);            /* 128, a cell boundary column */
  yFlat = (int)(4*LC_CELL + LC_CELL/2);/* 144, mid-cell                */
  getPixelRGB(&f, &fi, xEdge + (int)(LC_CELL/2), yFlat, &rgb[0], &rgb[1], &rgb[2]);
  lcToneRGB(lcPatternTone(xEdge + LC_CELL/2, yFlat), want);
  test_bool(rgb[0] == want[0] && rgb[1] == want[1] && rgb[2] == want[2]);

  /* The pixel straddling the boundary column: strictly between the tones in
     the red channel, which differs by 215 between them. */
  getPixelRGB(&f, &fi, xEdge, yFlat, &rgb[0], &rgb[1], &rgb[2]);
  test_bool(rgb[0] > 25 && rgb[0] < 240);

  vsFrameFree(&f);
}

void test_lenscorrect_pattern(void){
  test_lenscorrect_pattern_tones();
  test_lenscorrect_pattern_supersampling();
}

/* --- the clip generator --------------------------------------------------- */

/* Which of the two tones the pixel is nearer.  The tones' red channels are 25
   and 240, so 133 is the midpoint; an antialiased edge pixel crosses it within
   about a pixel of the true boundary. */
static int lcToneAt(const VSFrame* f, const VSFrameInfo* fi, int x, int y){
  uint8_t r,g,b;
  getPixelRGB(f, fi, x, y, &r,&g,&b);
  return r < 133 ? 0 : 1;
}

/* First x > from, along the horizontal centre line, whose nearest tone differs
   from the one at `from`.  The true boundary lies in [x-1, x]. */
static int lcFirstEdgeRight(const VSFrame* f, const VSFrameInfo* fi, int from){
  int x, t0 = lcToneAt(f, fi, from, fi->height/2);
  for(x=from+1; x<fi->width-1; x++)
    if(lcToneAt(f, fi, x, fi->height/2) != t0) return x;
  return -1;
}

/* Every tone change of the IDEAL scene along +x from the frame centre, as an
   offset in px, in increasing order.  cx = 320 is itself a multiple of
   LC_CELL, so cell boundaries sit at 32, 64, 96, ...; band boundaries at 60,
   120, 180, ...  The two sets are disjoint below lcm(32,60) = 480, so every
   entry is a genuine single tone change -- no coincident pair that would flip
   both terms and cancel.  Returns how many were written. */
static int lcIdealEdges(double* out, int max, double uMax){
  int n = 0;
  double u;
  for(u = LC_CELL; u <= uMax && n < max; u += LC_CELL) out[n++] = u;
  for(u = LC_BAND; u <= uMax && n < max; u += LC_BAND) out[n++] = u;
  /* insertion sort; the list is tiny and already two sorted runs */
  { int i, j; for(i=1; i<n; i++){ double v = out[i];
      for(j=i-1; j>=0 && out[j] > v; j--) out[j+1] = out[j];
      out[j+1] = v; } }
  return n;
}

/* With k == 0 and an identity camera pose, the generated frame must be the
   base image bit for bit: the generator's map collapses to the identity and
   nothing but the pattern function is left. */
static void test_lenscorrect_generator_null(void){
  VSFrameInfo fi;
  VSFrame base, frames[LC_NUM_FRAMES];
  int x, y, i, diff = 0;

  fprintf(stderr, "--- lens clip generator: k=0, identity pose ---\n");

  test_bool(vsFrameInfoInit(&fi, LC_WIDTH, LC_HEIGHT, PF_RGB24) != 0);
  vsFrameAllocate(&base, &fi);
  lcRenderMapped(&base, &fi, lcIdentityMap, NULL);

  lcGenerateClip(frames, &fi, PF_RGB24, 0.0, LC_NUM_FRAMES);

  /* frame 0 only: lcClipTransform(0) is the identity by construction, so with
     k == 0 the whole map is the identity. */
  for(y=0; y<fi.height; y++)
    for(x=0; x<fi.width; x++){
      uint8_t a[3], b[3];
      getPixelRGB(&base,      &fi, x, y, &a[0],&a[1],&a[2]);
      getPixelRGB(&frames[0], &fi, x, y, &b[0],&b[1],&b[2]);
      if(a[0]!=b[0] || a[1]!=b[1] || a[2]!=b[2]) diff++;
    }
  test_bool(diff == 0);

  vsFrameFree(&base);
  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

/* Frame 0 of the clip is pattern(U_k(x)) -- lcClipTransform(0) is the identity
   -- so a feature the ideal scene puts at radius r appears at radius D_k(r).
   Checking every edge against that prediction measures one homogeneous thing,
   unlike comparing successive gaps, where cell boundaries every 32 px and band
   boundaries every 60 px interleave and the comparison silently comes to be
   between features of different kinds.

   `inward` states the physical signature separately from the numbers: barrel
   pulls every feature toward the centre, pincushion pushes every one out. */
static void lcCheckEdgePositions(double k, int inward, const char* label){
  VSFrameInfo fi;
  VSFrame frames[LC_NUM_FRAMES];
  VSLensDistortion ld;
  double ideal[64];
  int n, i, from, checked = 0;

  fprintf(stderr, "--- %s ---\n", label);

  lcGenerateClip(frames, &fi, PF_RGB24, k, LC_NUM_FRAMES);
  ld = vsLensDistortionInit(&fi, k);
  n = lcIdealEdges(ideal, 64, 340.0);

  from = fi.width/2 + 1;
  for(i=0; i<n; i++){
    double ox, oy, pred;
    int actual;
    test_bool(vsLensDistortPoint(&ld, fi.width/2.0 + ideal[i],
                                 fi.height/2.0, &ox, &oy) == VS_OK);
    pred = ox - fi.width/2.0;
    if(pred > 300.0) break;            /* keeps clear of the frame edge */
    if(inward) test_bool(pred < ideal[i]);
    else       test_bool(pred > ideal[i]);
    actual = lcFirstEdgeRight(&frames[0], &fi, from);
    test_bool(actual > 0);
    fprintf(stderr, "  ideal %6.1f -> predicted %6.1f, actual %i\n",
            ideal[i], pred, actual - fi.width/2);
    test_bool(fabs((double)(actual - fi.width/2) - pred) <= 2.0);
    from = actual + 1;
    checked++;
  }
  /* Enough edges to be a real check, not one lucky match. */
  test_bool(checked >= 8);

  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

static void test_lenscorrect_generator_edge_positions(void){
  lcCheckEdgePositions(LC_K_BARREL, 1, "barrel pulls every edge inward, k = -0.25");
  lcCheckEdgePositions(LC_K_PIN,    0, "pincushion pushes every edge outward, k = +0.15");
}

/* lcInverseTransform must invert the render path's backward affine exactly.
   Checked numerically by composing the two maps at a scatter of points, which
   is independent of how the inverse is derived. */
static void test_lenscorrect_inverse_transform(void){
  int i, j;
  double pts[5][2] = {{0,0},{639,479},{320,240},{10,470},{500,30}};

  fprintf(stderr, "--- inverse of the render backward affine ---\n");

  for(i=1; i<LC_NUM_FRAMES; i++){
    VSTransform t  = lcClipTransform(i);
    VSTransform ti = lcInverseTransform(t);
    for(j=0; j<5; j++){
      double ax, ay, bx, by;
      lcBackwardAffine(&ti, pts[j][0], pts[j][1], &ax, &ay);
      lcBackwardAffine(&t,  ax, ay, &bx, &by);
      test_bool(fabs(bx - pts[j][0]) < 1e-9);
      test_bool(fabs(by - pts[j][1]) < 1e-9);
    }
  }
}

void test_lenscorrect_generator(void){
  test_lenscorrect_generator_null();
  test_lenscorrect_generator_edge_positions();
  test_lenscorrect_inverse_transform();
}

/* --- the round trip ------------------------------------------------------- */

/* Measured worst case over the six frames at k = -0.25: maxFlat = 1,
   PSNR = 32.47 dB.  Set with margin; the `off` control scores 8.57 dB, so the
   floor is nowhere near loose enough to let an uncorrected frame through. */
#define LC_MAX_FLAT_DELTA 3
#define LC_MIN_PSNR       28.0

typedef struct {
  int    n;        /* pixels inside the valid mask                          */
  int    nFlat;    /* of those, pixels whose ideal neighbourhood is uniform  */
  int    maxFlat;  /* largest per-channel |delta| over the flat pixels       */
  double psnr;     /* over the whole valid mask, all three channels          */
} LCCompare;

/* Runs one frame through the render path with the given mode, k and pose. */
static void lcCorrect(VSFrame* out, const VSFrame* in, const VSFrameInfo* fi,
                      VSLensCorrectMode mode, double k, VSTransform t){
  VSTransformData td;
  VSTransformConfig cfg = vsTransformGetDefaultConfig("lensclip-test");
  cfg.interpolType   = VS_BiLinear;
  cfg.crop           = VSCropBorder;
  cfg.optZoom        = 0;
  cfg.lensCorrection = mode;
  test_bool(vsTransformDataInit(&td, &cfg, fi, fi) == VS_OK);
  vsTransformSetLensK(&td, k);
  test_bool(vsTransformPrepare(&td, in, out) == VS_OK);
  test_bool(vsDoTransform(&td, t) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);
  vsTransformDataCleanup(&td);
}

/* Marks the destination pixels whose backward map lands inside the source
   frame with room for the bilinear tap, computed from the same composition
   the render path uses:

     Off    x_src = M_t(x)
     Full   x_src = D_k(M_t(x))
     Wobble x_src = D_k(M_t(U_k(x)))

   Everything else is border fill on one side or the other and carries no
   information.  This is exact rather than an inset rectangle, which matters:
   under barrel correction the invalid region is a curved apron whose depth
   varies along the edge, and a rectangle big enough to contain it would throw
   away a large part of the picture. */
static void lcValidMask(unsigned char* valid, const VSFrameInfo* fi,
                        const VSLensDistortion* ld, const VSTransform* t,
                        VSLensCorrectMode mode){
  const double margin = 2.0;
  int x, y;
  for(y=0; y<fi->height; y++)
    for(x=0; x<fi->width; x++){
      double ax = x, ay = y, sx, sy;
      int ok = 1;
      if(mode == VSLensCorrectWobble)
        ok = (vsLensUndistortPoint(ld, x, y, &ax, &ay) == VS_OK);
      if(ok) lcBackwardAffine(t, ax, ay, &sx, &sy);
      if(ok && mode != VSLensCorrectOff){
        double dx, dy;
        ok = (vsLensDistortPoint(ld, sx, sy, &dx, &dy) == VS_OK);
        sx = dx; sy = dy;
      }
      if(ok)
        ok = sx >= margin && sy >= margin &&
             sx <= fi->width-1-margin && sy <= fi->height-1-margin;
      valid[y*fi->width + x] = (unsigned char)(ok ? 1 : 0);
    }
}

/* True when every pixel of the 5x5 neighbourhood of (x,y) in `ideal` carries
   the identical colour.  5x5 rather than 3x3 because the map's local scale is
   not exactly 1: near the frame edge under k = -0.25 it compresses by up to
   about 1.3x, so an output neighbourhood draws on a slightly larger source
   neighbourhood, and the wider window keeps the guarantee honest.

   Using exact equality rather than a gradient threshold means there is no
   magic number to tune here: inside a checkerboard cell the pattern is
   constant to the byte, and any antialiased edge pixel fails the test. */
static int lcIsFlat(const VSFrame* ideal, const VSFrameInfo* fi, int x, int y){
  uint8_t r0,g0,b0, r,g,b;
  int i, j;
  if(x < 2 || y < 2 || x >= fi->width-2 || y >= fi->height-2) return 0;
  getPixelRGB(ideal, fi, x, y, &r0,&g0,&b0);
  for(j=-2; j<=2; j++)
    for(i=-2; i<=2; i++){
      getPixelRGB(ideal, fi, x+i, y+j, &r,&g,&b);
      if(r!=r0 || g!=g0 || b!=b0) return 0;
    }
  return 1;
}

static LCCompare lcCompare(const VSFrame* got, const VSFrame* ideal,
                           const VSFrameInfo* fi, const unsigned char* valid){
  LCCompare c;
  double se = 0;
  int x, y;
  c.n = c.nFlat = c.maxFlat = 0;
  c.psnr = 0;
  for(y=0; y<fi->height; y++)
    for(x=0; x<fi->width; x++){
      uint8_t a[3], b[3];
      int ch, flat;
      if(!valid[y*fi->width + x]) continue;
      getPixelRGB(got,   fi, x, y, &a[0],&a[1],&a[2]);
      getPixelRGB(ideal, fi, x, y, &b[0],&b[1],&b[2]);
      c.n++;
      flat = lcIsFlat(ideal, fi, x, y);
      if(flat) c.nFlat++;
      for(ch=0; ch<3; ch++){
        int d = (int)a[ch] - (int)b[ch];
        se += (double)d*d;
        if(flat && abs(d) > c.maxFlat) c.maxFlat = abs(d);
      }
    }
  if(c.n > 0){
    double mse = se / (3.0*c.n);
    c.psnr = mse > 0 ? 10.0*log10(255.0*255.0/mse) : 99.0;
  }
  return c;
}

/* What a corrected frame is compared against.

   LC_REF_BASE is the undistorted, unshaken scene -- what Full mode must
   produce.  LC_REF_FRAME0 is frame 0 of the clip itself, which is what Wobble
   must produce: its output is pattern(U_k(x)), and that IS frame 0, because
   lcClipTransform(0) is the identity. */
typedef enum { LC_REF_BASE, LC_REF_FRAME0 } LCReference;

/* One round-trip case.  minValidDiv bounds how much of the frame the mask has
   to keep: the assertion is n > width*height/minValidDiv, which stops the
   tolerance checks below it from passing vacuously on a mask that collapsed
   to a handful of pixels. */
typedef struct {
  VSPixelFormat     pf;
  double            k;
  VSLensCorrectMode mode;
  LCReference       ref;
  int               maxFlat;
  double            minPsnr;
  int               minValidDiv;
  const char*       label;
} LCCase;

/* Generates the clip, corrects every frame with the exact inverse of its own
   pose, and holds the result against the case's reference. */
static void lcCheckRoundTrip(const LCCase* c){
  VSFrameInfo fi;
  VSFrame ref, out, frames[LC_NUM_FRAMES];
  unsigned char* valid;
  VSLensDistortion ld;
  int i;

  fprintf(stderr, "--- %s ---\n", c->label);

  lcGenerateClip(frames, &fi, c->pf, c->k, LC_NUM_FRAMES);
  ld = vsLensDistortionInit(&fi, c->k);
  vsFrameAllocate(&out, &fi);
  vsFrameAllocate(&ref, &fi);
  if(c->ref == LC_REF_BASE)
    lcRenderMapped(&ref, &fi, lcIdentityMap, NULL);
  else
    vsFrameCopy(&ref, &frames[0], &fi);
  valid = (unsigned char*)malloc(fi.width*fi.height);
  test_bool(valid != NULL);

  for(i=0; i<LC_NUM_FRAMES; i++){
    VSTransform ti = lcInverseTransform(lcClipTransform(i));
    LCCompare cmp;
    lcCorrect(&out, &frames[i], &fi, c->mode, c->k, ti);
    lcValidMask(valid, &fi, &ld, &ti, c->mode);
    cmp = lcCompare(&out, &ref, &fi, valid);
    fprintf(stderr, "  frame %i: valid %i, flat %i, maxFlat %i, PSNR %.2f dB\n",
            i, cmp.n, cmp.nFlat, cmp.maxFlat, cmp.psnr);
    test_bool(cmp.n     > fi.width*fi.height/c->minValidDiv);
    test_bool(cmp.nFlat > cmp.n/4);
    test_bool(cmp.maxFlat <= c->maxFlat);
    test_bool(cmp.psnr    >= c->minPsnr);
  }

  free(valid);
  vsFrameFree(&ref); vsFrameFree(&out);
  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

/* Correcting frame i with the known k and the exact inverse of its pose must
   return the base image:

     out(x) = frame_i( D_k( M_t'(x) ) )
            = pattern( S_i( U_k( D_k( M_t'(x) ) ) ) )
            = pattern( M_t( M_t'(x) ) )  =  pattern(x)

   Bilinear resampling of a hard edge is off by up to half a step AT that edge
   however correct the geometry is, so the comparison is split: an exact
   tolerance on pixels whose neighbourhood in the ideal image is uniform, and
   a PSNR floor over the whole valid region to catch gross geometric error in
   the textured parts. */
static void test_lenscorrect_full_recovers_base(void){
  LCCase c;
  c.pf          = PF_RGB24;
  c.k           = LC_K_BARREL;
  c.mode        = VSLensCorrectFull;
  c.ref         = LC_REF_BASE;
  c.maxFlat     = LC_MAX_FLAT_DELTA;
  c.minPsnr     = LC_MIN_PSNR;
  c.minValidDiv = 2;
  c.label       = "Full correction recovers the base image, k = -0.25";
  lcCheckRoundTrip(&c);
}

/* The control that stops all of the above from passing with the lens map
   stubbed out to a no-op: the same clip, same inverse pose, correction OFF.
   The distortion is then still in the picture and the score must collapse.

   Written out rather than routed through lcCheckRoundTrip because it is a
   different shape of assertion -- two runs compared against each other over
   one shared mask, not one run against a tolerance. */
static void test_lenscorrect_off_is_much_worse(void){
  VSFrameInfo fi;
  VSFrame base, outOn, outOff, frames[LC_NUM_FRAMES];
  unsigned char* valid;
  VSLensDistortion ld;
  VSTransform ti;
  LCCompare on, off;
  int i;

  fprintf(stderr, "--- control: correction off scores far worse ---\n");

  lcGenerateClip(frames, &fi, PF_RGB24, LC_K_BARREL, LC_NUM_FRAMES);
  ld = vsLensDistortionInit(&fi, LC_K_BARREL);
  vsFrameAllocate(&base, &fi);
  vsFrameAllocate(&outOn, &fi);
  vsFrameAllocate(&outOff, &fi);
  lcRenderMapped(&base, &fi, lcIdentityMap, NULL);
  valid = (unsigned char*)malloc(fi.width*fi.height);
  test_bool(valid != NULL);

  ti = lcInverseTransform(lcClipTransform(0));
  lcCorrect(&outOn,  &frames[0], &fi, VSLensCorrectFull, LC_K_BARREL, ti);
  lcCorrect(&outOff, &frames[0], &fi, VSLensCorrectOff,  LC_K_BARREL, ti);
  /* Both scored over the SAME mask -- the one the corrected run uses -- so the
     comparison is of picture quality, not of how much of the frame each run
     happened to fill. */
  lcValidMask(valid, &fi, &ld, &ti, VSLensCorrectFull);
  on  = lcCompare(&outOn,  &base, &fi, valid);
  off = lcCompare(&outOff, &base, &fi, valid);
  fprintf(stderr, "  PSNR on %.2f dB, off %.2f dB\n", on.psnr, off.psnr);
  test_bool(on.psnr > off.psnr + 6.0);
  test_bool(off.maxFlat > LC_MAX_FLAT_DELTA);

  free(valid);
  vsFrameFree(&base); vsFrameFree(&outOn); vsFrameFree(&outOff);
  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

void test_lenscorrect_roundtrip_full(void){
  test_lenscorrect_full_recovers_base();
  test_lenscorrect_off_is_much_worse();
}
