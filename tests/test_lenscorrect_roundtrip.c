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
