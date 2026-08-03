/*
   test_draw.c

   Characterization + behaviour tests for the overlay drawing primitives used
   by the 'show' option (drawBox / drawRectangle / drawLine).

   The geometry tests here were written against the pre-existing planar
   implementation and pin its exact output, so that rebuilding the primitives
   on top of drawHLine/drawVLine cannot silently change what 'show' renders.
   The clipping tests describe the intended behaviour for shapes that reach
   past the frame edge.

   This file is part of vid.stab video stabilization library
   and distributed under the GNU GPL, see the top of motiondetect.c.
*/

/* logical frame; the buffer is deliberately taller than DRAW_H so that writes
   past the bottom edge land inside the allocation and can be detected instead
   of corrupting the heap */
#define DRAW_STRIDE 64
#define DRAW_W      64
#define DRAW_H      48
#define DRAW_ROWS   64   /* rows actually allocated */
#define DRAW_BUFSZ  (DRAW_STRIDE * DRAW_ROWS)

/* golden values for the diagonal line, captured from the original
   implementation (12,10)->(28,26), thickness 3 */
#define DIAG_EXPECT_COUNT 85
#define DIAG_EXPECT_SUM   99620

/* golden values for the end-to-end planar overlay, captured by running
   test_draw_planar_show() against the implementation as it stood before the
   primitives were rebuilt (origin/master at the time of this change) */
#define PLANAR_SHOW_CHECKSUM 4339987323UL
#define PLANAR_SHOW_NONZERO  72349

static void draw_clear(unsigned char* b){ memset(b, 0, DRAW_BUFSZ); }

/* mark a filled span of a single row in the expectation buffer */
static void exp_hspan(unsigned char* e, int x, int y, int len, unsigned char c){
  for (int k = 0; k < len; k++) e[(x + k) + y * DRAW_STRIDE] = c;
}
static void exp_vspan(unsigned char* e, int x, int y, int len, unsigned char c){
  for (int k = 0; k < len; k++) e[x + (y + k) * DRAW_STRIDE] = c;
}

/* report the first differing pixel, to make a failure diagnosable */
static int draw_cmp(const unsigned char* got, const unsigned char* exp, const char* what){
  for (int y = 0; y < DRAW_ROWS; y++){
    for (int x = 0; x < DRAW_STRIDE; x++){
      int i = x + y * DRAW_STRIDE;
      if (got[i] != exp[i]){
        fprintf(stderr, "  %s: first mismatch at (%d,%d): got %d, expected %d\n",
                what, x, y, got[i], exp[i]);
        return 0;
      }
    }
  }
  return 1;
}

/* number of pixels written outside the logical frame */
static int draw_outside_count(const unsigned char* b){
  int n = 0;
  for (int y = 0; y < DRAW_ROWS; y++)
    for (int x = 0; x < DRAW_STRIDE; x++)
      if ((y >= DRAW_H || x >= DRAW_W) && b[x + y * DRAW_STRIDE] != 0) n++;
  return n;
}

void test_draw_geometry(void){
  unsigned char* buf = (unsigned char*)vs_malloc(DRAW_BUFSZ);
  unsigned char* exp = (unsigned char*)vs_malloc(DRAW_BUFSZ);

  /* --- drawBox: filled, sizex x sizey, top-left at (x-sizex/2, y-sizey/2) --- */
  draw_clear(buf); draw_clear(exp);
  drawBox(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1, 20, 20, 8, 6, 200);
  for (int r = 0; r < 6; r++) exp_hspan(exp, 20 - 4, 20 - 3 + r, 8, 200);
  fprintf(stderr,"*** drawBox: filled 8x6 box\n");
  test_bool(draw_cmp(buf, exp, "drawBox"));

  /* --- drawRectangle: outline. Note the deliberate asymmetry of the original
     implementation: the horizontal lines span sizex columns starting at
     x-sizex/2, while the right vertical line sits at x+sizex/2, which for even
     sizex is one column past the end of those horizontal lines. Same for the
     lower horizontal line vs. the vertical extent. This is pinned on purpose. */
  draw_clear(buf); draw_clear(exp);
  drawRectangle(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1, 30, 24, 10, 8, 100);
  exp_hspan(exp, 30 - 5, 24 - 4, 10, 100);          /* upper */
  exp_hspan(exp, 30 - 5, 24 + 4, 10, 100);          /* lower */
  exp_vspan(exp, 30 - 5, 24 - 4,  8, 100);          /* left  */
  exp_vspan(exp, 30 + 5, 24 - 4,  8, 100);          /* right */
  fprintf(stderr,"*** drawRectangle: 10x8 outline\n");
  test_bool(draw_cmp(buf, exp, "drawRectangle"));

  /* --- drawLine, horizontal: thickness rows, endpoints inclusive --- */
  draw_clear(buf); draw_clear(exp);
  { Vec a = {10, 30}, b = {25, 30};
    drawLine(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1, &a, &b, 3, 150); }
  for (int r = -1; r <= 1; r++) exp_hspan(exp, 10, 30 + r, 25 - 10 + 1, 150);
  fprintf(stderr,"*** drawLine: horizontal, thickness 3\n");
  test_bool(draw_cmp(buf, exp, "drawLine/horizontal"));

  /* --- drawLine, vertical --- */
  draw_clear(buf); draw_clear(exp);
  { Vec a = {40, 8}, b = {40, 20};
    drawLine(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1, &a, &b, 3, 150); }
  for (int r = -1; r <= 1; r++) exp_vspan(exp, 40 + r, 8, 20 - 8 + 1, 150);
  fprintf(stderr,"*** drawLine: vertical, thickness 3\n");
  test_bool(draw_cmp(buf, exp, "drawLine/vertical"));

  /* --- drawLine, diagonal: geometry involves floating point slope, so pin it
     by the exact set of touched pixels rather than a closed form --- */
  draw_clear(buf);
  { Vec a = {12, 10}, b = {28, 26};
    drawLine(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1, &a, &b, 3, 150); }
  int diag_count = 0, diag_sum = 0;
  for (int i = 0; i < DRAW_BUFSZ; i++)
    if (buf[i]) { diag_count++; diag_sum += i; }
  fprintf(stderr,"*** drawLine: diagonal (count=%d checksum=%d)\n", diag_count, diag_sum);
  test_bool(diag_count == DIAG_EXPECT_COUNT);
  test_bool(diag_sum   == DIAG_EXPECT_SUM);

  vs_free(buf); vs_free(exp);
}

/* Shapes that reach past the frame edge must be clipped, not written out of
   bounds. Every case below would scribble outside the logical frame in the
   original implementation. */
void test_draw_clipping(void){
  unsigned char* buf = (unsigned char*)vs_malloc(DRAW_BUFSZ);

  unsigned char* exp = (unsigned char*)vs_malloc(DRAW_BUFSZ);

  fprintf(stderr,"*** clipping: shapes overlapping each frame edge\n");

  /* A clipped box must equal exactly the intersection of the box with the
     frame. Checking the whole buffer (not just a count of out-of-frame pixels)
     is what catches horizontal overrun, which wraps into the neighbouring row
     rather than leaving the frame and so is invisible to a bounds count. */
  struct { const char* name; int x, y, sx, sy; } cases[] = {
    {"left edge",   2,          20,         16,  8},
    {"right edge",  DRAW_W - 2, 20,         16,  8},
    {"bottom edge", 20,         DRAW_H - 2,  8, 16},
    {"top edge",    20,         2,           8, 16},
  };
  for (unsigned ci = 0; ci < sizeof(cases)/sizeof(cases[0]); ci++){
    int x0 = cases[ci].x - cases[ci].sx / 2, y0 = cases[ci].y - cases[ci].sy / 2;
    draw_clear(buf); draw_clear(exp);
    drawBox(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1,
            cases[ci].x, cases[ci].y, cases[ci].sx, cases[ci].sy, 200);
    for (int yy = y0; yy < y0 + cases[ci].sy; yy++)
      for (int xx = x0; xx < x0 + cases[ci].sx; xx++)
        if (xx >= 0 && xx < DRAW_W && yy >= 0 && yy < DRAW_H)
          exp[xx + yy * DRAW_STRIDE] = 200;
    test_bool(draw_cmp(buf, exp, cases[ci].name));
    test_bool(draw_outside_count(buf) == 0);
  }
  vs_free(exp);

  /* entirely outside: nothing at all may be written */
  draw_clear(buf);
  drawBox(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1, -50, -50, 8, 8, 200);
  drawBox(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1, DRAW_W + 50, DRAW_H + 50, 8, 8, 200);
  for (int i = 0; i < DRAW_BUFSZ; i++) test_bool(buf[i] == 0);

  /* rectangle and lines crossing edges */
  draw_clear(buf);
  drawRectangle(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1, 4, DRAW_H - 3, 20, 12, 100);
  test_bool(draw_outside_count(buf) == 0);

  draw_clear(buf);
  { Vec a = {-20, 25}, b = {DRAW_W + 20, 25};
    drawLine(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1, &a, &b, 3, 150); }
  test_bool(draw_outside_count(buf) == 0);

  draw_clear(buf);
  { Vec a = {30, -20}, b = {30, DRAW_H + 20};
    drawLine(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1, &a, &b, 3, 150); }
  test_bool(draw_outside_count(buf) == 0);

  draw_clear(buf);
  { Vec a = {-10, -10}, b = {DRAW_W + 10, DRAW_H + 10};
    drawLine(buf, DRAW_STRIDE, DRAW_W, DRAW_H, 1, &a, &b, 3, 150); }
  test_bool(draw_outside_count(buf) == 0);

  vs_free(buf);
}

/* End-to-end guard on the planar overlay: run the detector with show enabled
   over two planar frames and check the rendered frame against a golden
   checksum captured from the implementation before the primitives were
   rebuilt. This test deliberately uses only the public entry points, so it
   compiles unchanged against either implementation. */
void test_draw_planar_show(void){
  VSFrameInfo fi;
  VSFrame f1, f2;
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig("test_planar_show");
  VSMotionDetect md;
  LocalMotions lms;
  unsigned long sum = 0;
  int count = 0;

  fprintf(stderr,"*** planar show: overlay output must not change\n");
  test_bool(vsFrameInfoInit(&fi, 320, 240, PF_YUV420P) == 1);
  vsFrameAllocate(&f1,&fi);
  vsFrameAllocate(&f2,&fi);

  /* deterministic content: a fixed pseudo random luma plane, shifted by (8,-6) */
  unsigned int seed = 12345;
  for(int y=0; y<fi.height; y++){
    for(int x=0; x<fi.width; x++){
      seed = seed * 1103515245u + 12345u;
      f1.data[0][x + y*f1.linesize[0]] = (seed >> 16) & 0xFF;
    }
  }
  for(int p=1; p<3; p++)
    for(int y=0; y<fi.height/2; y++)
      memset(f1.data[p] + y*f1.linesize[p], 128, fi.width/2);
  vsFrameCopy(&f2,&f1,&fi);
  for(int y=0; y<fi.height; y++){
    for(int x=0; x<fi.width; x++){
      int sx = x - 8, sy = y + 6;
      f2.data[0][x + y*f2.linesize[0]] =
        (sx>=0 && sy>=0 && sx<fi.width && sy<fi.height)
        ? f1.data[0][sx + sy*f1.linesize[0]] : 0;
    }
  }

  mdconf.show = 2;
  test_bool(vsMotionDetectInit(&md, &mdconf, &fi) == VS_OK);
  md.conf.numThreads = 1;
  test_bool(vsMotionDetection(&md, &lms, &f1) == VS_OK);
  vs_vector_del(&lms);
  test_bool(vsMotionDetection(&md, &lms, &f2) == VS_OK);
  vs_vector_del(&lms);

  for(int y=0; y<fi.height; y++)
    for(int x=0; x<fi.width; x++){
      unsigned char v = f2.data[0][x + y*f2.linesize[0]];
      sum += (unsigned long)v * (unsigned long)(x + 3*y + 1);
      count += (v != 0);
    }
  fprintf(stderr,"  planar show checksum=%lu nonzero=%d\n", sum, count);
  test_bool(sum   == PLANAR_SHOW_CHECKSUM);
  test_bool(count == PLANAR_SHOW_NONZERO);

  vsMotionDetectionCleanup(&md);
  vsFrameFree(&f1);
  vsFrameFree(&f2);
}

/* The primitives must treat bytesPerPixel uniformly: 1 byte for planar (luma
   only), 3 for RGB24/BGR24, and 3-of-4 for RGBA with alpha left untouched.
   Because every overlay colour is a grey level, RGB and BGR are identical. */
void test_draw_packed(void){
  const int W = 32, H = 16;
  fprintf(stderr,"*** packed: bytesPerPixel 3 and 4, grey overlay, alpha preserved\n");

  for (int bpp = 3; bpp <= 4; bpp++){
    int linesize = W * bpp;
    unsigned char* buf = (unsigned char*)vs_malloc(linesize * H);
    memset(buf, 0, linesize * H);
    /* pre-fill alpha so we can tell whether it survives */
    if (bpp == 4)
      for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) buf[x*bpp + y*linesize + 3] = 0xAB;

    drawBox(buf, linesize, W, H, bpp, 10, 8, 6, 4, 200);

    int drawn = 0, greyOk = 1, alphaOk = 1, outside = 0;
    for (int y = 0; y < H; y++){
      for (int x = 0; x < W; x++){
        unsigned char* p = buf + x*bpp + y*linesize;
        int inBox = (x >= 10-3 && x < 10-3+6 && y >= 8-2 && y < 8-2+4);
        int set = (p[0] || p[1] || p[2]);
        if (inBox){
          drawn++;
          if (!(p[0] == 200 && p[1] == 200 && p[2] == 200)) greyOk = 0;
        } else if (set) outside++;
        if (bpp == 4 && p[3] != 0xAB) alphaOk = 0;
      }
    }
    test_bool(drawn == 6*4);
    test_bool(greyOk);
    test_bool(outside == 0);
    test_bool(alphaOk);
    vs_free(buf);
  }
}
