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

#if defined(VS_HAVE_SSE2) || defined(VS_HAVE_AVX2) || defined(VS_HAVE_AVX512) \
 || defined(VS_HAVE_NEON)
#define SIMD_HAVE_ANY_KERNEL 1
#endif

#ifdef SIMD_HAVE_ANY_KERNEL

/* The table of kernels to check.  Every kernel compiled into this build is
   checked against the plain C reference -- not only the one the dispatcher
   would pick, since it can select just one per run.

   But "compiled in" is not the same as "runnable here": what a build machine's
   *compiler* can emit routinely exceeds what its *CPU* can execute, and a CI
   runner with a recent GCC on a pre-AVX-512 core is the normal case rather
   than an exotic one.  Calling such a kernel directly, as this test does,
   bypasses the runtime dispatch that protects the library itself and gets
   SIGILL.  So each entry carries the feature bit it needs and is skipped
   unless vs_cpu_flags() reports it.

   `requires == VS_CPU_NONE` means "always safe to call": that is the NEON
   kernel when it was built against the scalar emulation in neon_emu.h, which
   is plain C and runs anywhere. */
typedef struct {
  const char*         name;
  unsigned int        requires;
  vsCompareSubImgFn   cmp;
  vsContrastSubImg1Fn con;
} SimdKernel;

static const SimdKernel simd_kernels[] = {
#ifdef VS_HAVE_SSE2
  { "SSE2",   VS_CPU_SSE2,   compareSubImg_thr_sse2,   contrastSubImg1_SSE    },
#endif
#ifdef VS_HAVE_AVX2
  { "AVX2",   VS_CPU_AVX2,   compareSubImg_thr_avx2,   contrastSubImg1_avx2   },
#endif
#ifdef VS_HAVE_AVX512
  { "AVX512", VS_CPU_AVX512, compareSubImg_thr_avx512, contrastSubImg1_avx512 },
#endif
#ifdef VS_HAVE_NEON
#ifdef VS_NEON_EMULATION
  { "NEON(emulated)", VS_CPU_NONE, compareSubImg_thr_neon, contrastSubImg1_neon },
#else
  { "NEON",   VS_CPU_NEON,   compareSubImg_thr_neon,   contrastSubImg1_neon   },
#endif
#endif
};
#define SIMD_NUM_KERNELS ((int)(sizeof(simd_kernels)/sizeof(simd_kernels[0])))

/* only multiples of 16 are valid, see file header */
static const int simd_field_sizes[]  = { 16, 32, 48, 64, 80 };
#define SIMD_NUM_FIELD_SIZES ((int)(sizeof(simd_field_sizes)/sizeof(simd_field_sizes[0])))

/* several field positions, all far enough from the border for size 80 + shift 24 */
static const int simd_field_pos[][2] = { {400,300}, {200,200}, {900,500}, {640,360} };
#define SIMD_NUM_FIELD_POS ((int)(sizeof(simd_field_pos)/sizeof(simd_field_pos[0])))

/* mimics the search of calcFieldTransPlanar (non-spiral variant):
   start at (0,0), then scan with a tightening minerror that is handed to the
   compare function as threshold. Returns the selected shift and the minerror. */
static void simd_argmin_scan(vsCompareSubImgFn cmpsubfunc,
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
static void simd_test_compare_strict(const TestData* testdata, const SimdKernel* k){
  int s, p, dx, dy;
  int mismatches = 0;
  unsigned char* I1 = testdata->frames[0].data[0];
  unsigned char* I2 = testdata->frames[1].data[0];
  int w1 = testdata->frames[0].linesize[0];
  int w2 = testdata->frames[1].linesize[0];
  int h  = testdata->fi.height;

  fprintf(stderr,"*** [%s] compareSubImg: strict equality (threshold=UINT_MAX)\n", k->name);
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
          unsigned int optval = k->cmp(I1, I2, &f, w1, w2, h,
                                          1, dx, dy, UINT_MAX);
          if (refval != optval && mismatches++ < 10)
            fprintf(stderr,"  MISMATCH [%s] size=%i pos=(%i,%i) d=(%i,%i): C=%u opt=%u\n",
                    k->name, f.size, f.x, f.y, dx, dy, refval, optval);
          test_bool(refval == optval);
        }
      }
    }
  }
}

/* finite threshold: the returned numbers may legally differ (early exit).
   Assert only what the caller relies on: both must agree on whether the
   result is <= threshold. */
static void simd_test_compare_threshold(const TestData* testdata, const SimdKernel* k){
  int s, p, t, dx, dy;
  unsigned char* I1 = testdata->frames[0].data[0];
  unsigned char* I2 = testdata->frames[1].data[0];
  int w1 = testdata->frames[0].linesize[0];
  int w2 = testdata->frames[1].linesize[0];
  int h  = testdata->fi.height;

  fprintf(stderr,"*** [%s] compareSubImg: finite threshold, <=threshold decision must agree\n", k->name);
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
            unsigned int optval = k->cmp(I1, I2, &f, w1, w2, h,
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
static void simd_test_compare_argmin(const TestData* testdata, const SimdKernel* k){
  int s, p;
  unsigned char* I1 = testdata->frames[0].data[0];
  unsigned char* I2 = testdata->frames[1].data[0];
  int w1 = testdata->frames[0].linesize[0];
  int w2 = testdata->frames[1].linesize[0];
  int h  = testdata->fi.height;

  fprintf(stderr,"*** [%s] compareSubImg: argmin of a calcFieldTransPlanar-like scan must match\n", k->name);
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
      simd_argmin_scan(k->cmp,                 I1, I2, &f, w1, w2, h, 12, 2,
                       &txO, &tyO, &minO);
      if (txC != txO || tyC != tyO || minC != minO)
        fprintf(stderr,"  ARGMIN MISMATCH [%s] size=%i pos=(%i,%i): "
                "C=(%i,%i,%u) opt=(%i,%i,%u)\n",
                k->name, f.size, f.x, f.y, txC, tyC, minC, txO, tyO, minO);
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
static void simd_test_contrast(const TestData* testdata, const SimdKernel* k){
  int s, p, fr;
  fprintf(stderr,"*** [%s] contrastSubImg: strict equality vs C\n", k->name);
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
        double cO = k->con(I, &f, w, h);
        if (cC != cO)
          fprintf(stderr,"  CONTRAST MISMATCH [%s] frame=%i size=%i pos=(%i,%i): "
                  "C=%.17g opt=%.17g\n", k->name, fr, f.size, f.x, f.y, cC, cO);
        test_bool(cC == cO);
      }
    }
  }
}

#endif /* SIMD_HAVE_ANY_KERNEL */

void test_simd_equivalence(const TestData* testdata){
#ifdef SIMD_HAVE_ANY_KERNEL
  int i, checked = 0;
  unsigned int cpu = vs_cpu_flags();
  fprintf(stderr,"********** SIMD vs C equivalence (%i kernel(s) compiled in, "
          "this CPU: %s):\n", SIMD_NUM_KERNELS, vs_cpu_flags_name(cpu));
  for (i = 0; i < SIMD_NUM_KERNELS; i++) {
    const SimdKernel* k = &simd_kernels[i];
    /* Calling a kernel the CPU cannot execute is a SIGILL, not a test
       failure -- skip loudly so a permanently-skipped kernel is visible
       rather than quietly passing as "nothing to do". */
    if (k->requires != VS_CPU_NONE && !(cpu & k->requires)) {
      fprintf(stderr,"*** [%s] SKIPPED: compiled in, but not supported by this CPU\n",
              k->name);
      continue;
    }
    simd_test_compare_strict(testdata, k);
    simd_test_compare_threshold(testdata, k);
    simd_test_compare_argmin(testdata, k);
    simd_test_contrast(testdata, k);
    checked++;
  }
  fprintf(stderr,"********** %i of %i kernel(s) checked on this machine\n",
          checked, SIMD_NUM_KERNELS);
#else
  (void)testdata;
  fprintf(stderr,"********** SIMD vs C equivalence: no SIMD kernels in this build\n");
#endif
}
