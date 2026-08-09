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
