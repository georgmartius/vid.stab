/* test_simd_equivalence.c
 *
 * Real (failable) equivalence tests between the optimized SIMD routines and
 * the plain C reference implementations:
 *   compareSubImg_thr        (src/motiondetect.c)      <-> compareSubImg_thr_sse2 (src/motiondetect_opt.c)
 *   contrastSubImg           (src/motiondetect.c)      <-> contrastSubImg1_SSE    (src/motiondetect_opt.c)
 *
 * IMPORTANT -- only field sizes that are a multiple of 16 are legal input for
 * the SSE2 routines: their inner loops advance 16 bytes per iteration
 * ("k += 16"), so a size that is not a multiple of 16 makes them read past the
 * end of the field and the two implementations then legitimately disagree.
 * Production code can never do this: vsMotionDetectInit() rounds both
 * fieldSize and fieldSizeFine up to a multiple of 16 when USE_SSE2 /
 * USE_SSE2_ASM is enabled (src/motiondetect.c, "fieldSize = (fieldSize / 16 + 1) * 16").
 * Therefore the tests below only use sizes 16, 32, 48, 64, 80.
 *
 * IMPORTANT -- the "threshold" parameter: both implementations abort early once
 * the running sum exceeds threshold, and the value returned by an early-exited
 * call is NOT required to equal the full sum -- the only guarantee is that it
 * is greater than threshold.  (The SSE2 version even drops the not yet
 * horizontally-added partial sums when it breaks out.)  Consequently:
 *   - with threshold == UINT_MAX neither can exit early, so the returned values
 *     MUST be exactly equal  -> strict equality is asserted.
 *   - with a finite threshold we must not compare the numbers.  We assert the
 *     two properties the only caller (calcFieldTransPlanar, which uses the
 *     result exclusively as "if (error < minerror)") actually depends on:
 *       (a) both agree on whether the result is <= threshold,
 *       (b) a full argmin scan driven by either implementation selects the
 *           very same (tx,ty) and the same final minerror.
 */

#ifdef USE_SSE2

/* only multiples of 16 are valid, see file header */
static const int simd_field_sizes[]  = { 16, 32, 48, 64, 80 };
#define SIMD_NUM_FIELD_SIZES ((int)(sizeof(simd_field_sizes)/sizeof(simd_field_sizes[0])))

/* several field positions, all far enough from the border for size 80 + shift 24 */
static const int simd_field_pos[][2] = { {400,300}, {200,200}, {900,500}, {640,360} };
#define SIMD_NUM_FIELD_POS ((int)(sizeof(simd_field_pos)/sizeof(simd_field_pos[0])))

/* mimics the search of calcFieldTransPlanar (non-spiral variant):
   start at (0,0), then scan with a tightening minerror that is handed to the
   compare function as threshold. Returns the selected shift and the minerror. */
static void simd_argmin_scan(cmpSubImgFunc cmpsubfunc,
                             unsigned char* I1, unsigned char* I2,
                             const Field* field, int width1, int width2, int height,
                             int maxShift, int stepSize,
                             int* tx_out, int* ty_out, unsigned int* minerror_out){
  int i, j;
  int tx = 0, ty = 0;
  unsigned int minerror = cmpsubfunc(I1, I2, field, width1, width2, height,
                                    1, 0, 0, UINT_MAX);
  for (i = -maxShift; i <= maxShift; i += stepSize) {
    for (j = -maxShift; j <= maxShift; j += stepSize) {
      if (i == 0 && j == 0)
        continue; /* already done */
      unsigned int error = cmpsubfunc(I1, I2, field, width1, width2, height,
                                      1, i, j, minerror);
      if (error < minerror) {
        minerror = error;
        tx = i;
        ty = j;
      }
    }
  }
  *tx_out = tx;
  *ty_out = ty;
  *minerror_out = minerror;
}

/* threshold == UINT_MAX: no early exit possible on either side,
   so the returned sums must be bit-for-bit equal. */
static void simd_test_compare_strict(const TestData* testdata){
  int s, p, dx, dy;
  int mismatches = 0;
  unsigned char* I1 = testdata->frames[0].data[0];
  unsigned char* I2 = testdata->frames[1].data[0];
  int w1 = testdata->frames[0].linesize[0];
  int w2 = testdata->frames[1].linesize[0];
  int h  = testdata->fi.height;

  fprintf(stderr,"*** compareSubImg: strict equality (threshold=UINT_MAX)\n");
  for (s = 0; s < SIMD_NUM_FIELD_SIZES; s++) {
    for (p = 0; p < SIMD_NUM_FIELD_POS; p++) {
      Field f;
      f.size = simd_field_sizes[s];
      f.x    = simd_field_pos[p][0];
      f.y    = simd_field_pos[p][1];
      for (dy = -16; dy <= 16; dy += 8) {       /* includes 0 */
        for (dx = -16; dx <= 16; dx += 8) {     /* includes 0 */
          unsigned int refval = compareSubImg_thr(I1, I2, &f, w1, w2, h,
                                                 1, dx, dy, UINT_MAX);
          unsigned int optval = compareSubImg_thr_sse2(I1, I2, &f, w1, w2, h,
                                                       1, dx, dy, UINT_MAX);
          if (refval != optval && mismatches++ < 10)
            fprintf(stderr,"  MISMATCH size=%i pos=(%i,%i) d=(%i,%i): C=%u SSE2=%u\n",
                    f.size, f.x, f.y, dx, dy, refval, optval);
          test_bool(refval == optval);
        }
      }
    }
  }
}

/* finite threshold: the returned numbers may legally differ (early exit).
   Assert only what the caller relies on: both must agree on whether the
   result is <= threshold. */
static void simd_test_compare_threshold(const TestData* testdata){
  int s, p, t, dx, dy;
  unsigned char* I1 = testdata->frames[0].data[0];
  unsigned char* I2 = testdata->frames[1].data[0];
  int w1 = testdata->frames[0].linesize[0];
  int w2 = testdata->frames[1].linesize[0];
  int h  = testdata->fi.height;

  fprintf(stderr,"*** compareSubImg: finite threshold, <=threshold decision must agree\n");
  for (s = 0; s < SIMD_NUM_FIELD_SIZES; s++) {
    for (p = 0; p < SIMD_NUM_FIELD_POS; p++) {
      Field f;
      f.size = simd_field_sizes[s];
      f.x    = simd_field_pos[p][0];
      f.y    = simd_field_pos[p][1];
      /* a representative full sum to derive thresholds from */
      unsigned int full = compareSubImg_thr(I1, I2, &f, w1, w2, h, 1, 0, 0, UINT_MAX);
      unsigned int thresholds[4];
      thresholds[0] = 0;
      thresholds[1] = full / 4;
      thresholds[2] = full;
      thresholds[3] = full * 2 + 1;
      for (t = 0; t < 4; t++) {
        unsigned int thr = thresholds[t];
        for (dy = -8; dy <= 8; dy += 8) {
          for (dx = -8; dx <= 8; dx += 8) {
            unsigned int refval = compareSubImg_thr(I1, I2, &f, w1, w2, h,
                                                    1, dx, dy, thr);
            unsigned int optval = compareSubImg_thr_sse2(I1, I2, &f, w1, w2, h,
                                                         1, dx, dy, thr);
            /* the guarantee: an early-exited result is > threshold */
            test_bool((refval <= thr) == (optval <= thr));
          }
        }
      }
    }
  }
}

/* the property that really matters: the search in calcFieldTransPlanar must
   pick the same shift and end with the same minerror, whichever
   implementation drives it. */
static void simd_test_compare_argmin(const TestData* testdata){
  int s, p;
  unsigned char* I1 = testdata->frames[0].data[0];
  unsigned char* I2 = testdata->frames[1].data[0];
  int w1 = testdata->frames[0].linesize[0];
  int w2 = testdata->frames[1].linesize[0];
  int h  = testdata->fi.height;

  fprintf(stderr,"*** compareSubImg: argmin of a calcFieldTransPlanar-like scan must match\n");
  for (s = 0; s < SIMD_NUM_FIELD_SIZES; s++) {
    for (p = 0; p < SIMD_NUM_FIELD_POS; p++) {
      Field f;
      int txC, tyC, txO, tyO;
      unsigned int minC, minO;
      f.size = simd_field_sizes[s];
      f.x    = simd_field_pos[p][0];
      f.y    = simd_field_pos[p][1];
      simd_argmin_scan(compareSubImg_thr,      I1, I2, &f, w1, w2, h, 12, 2,
                       &txC, &tyC, &minC);
      simd_argmin_scan(compareSubImg_thr_sse2, I1, I2, &f, w1, w2, h, 12, 2,
                       &txO, &tyO, &minO);
      if (txC != txO || tyC != tyO || minC != minO)
        fprintf(stderr,"  ARGMIN MISMATCH size=%i pos=(%i,%i): "
                "C=(%i,%i,%u) SSE2=(%i,%i,%u)\n",
                f.size, f.x, f.y, txC, tyC, minC, txO, tyO, minO);
      test_bool(txC == txO);
      test_bool(tyC == tyO);
      test_bool(minC == minO);
    }
  }
}

/* contrastSubImg (C, bytesPerPixel=1) vs contrastSubImg1_SSE.
   Both reduce the field to an 8 bit minimum and maximum and then evaluate the
   very same expression (maxi-mini)/(maxi+mini+0.1) in double precision.  If
   the two reductions agree, the doubles are computed by identical operations
   on identical integer inputs and are therefore bit-identical -- there is no
   reordering of floating point arithmetic anywhere.  Hence we assert exact
   equality rather than an epsilon: any nonzero difference means the SIMD
   min/max reduction is wrong, which is exactly what we want to catch. */
static void simd_test_contrast(const TestData* testdata){
  int s, p, fr;
  fprintf(stderr,"*** contrastSubImg: strict equality C vs SSE2\n");
  for (fr = 0; fr < 2; fr++) {
    unsigned char* I = testdata->frames[fr].data[0];
    int w = testdata->frames[fr].linesize[0];
    int h = testdata->fi.height;
    for (s = 0; s < SIMD_NUM_FIELD_SIZES; s++) {
      for (p = 0; p < SIMD_NUM_FIELD_POS; p++) {
        Field f;
        f.size = simd_field_sizes[s];
        f.x    = simd_field_pos[p][0];
        f.y    = simd_field_pos[p][1];
        double cC = contrastSubImg(I, &f, w, h, 1);
        double cO = contrastSubImg1_SSE(I, &f, w, h);
        if (cC != cO)
          fprintf(stderr,"  CONTRAST MISMATCH frame=%i size=%i pos=(%i,%i): "
                  "C=%.17g SSE2=%.17g\n", fr, f.size, f.x, f.y, cC, cO);
        test_bool(cC == cO);
      }
    }
  }
}

#endif /* USE_SSE2 */

void test_simd_equivalence(const TestData* testdata){
#ifdef USE_SSE2
  fprintf(stderr,"********** SSE2 vs C equivalence:\n");
  simd_test_compare_strict(testdata);
  simd_test_compare_threshold(testdata);
  simd_test_compare_argmin(testdata);
  simd_test_contrast(testdata);
#else
  (void)testdata;
  fprintf(stderr,"********** SSE2 vs C equivalence: SSE2 not enabled, nothing to compare\n");
#endif
}
