/*
 * test_fovmodel.c
 *
 *  What the similarity model costs on wide glass, measured.
 *
 *  The footage comes from generate_fovclip.c: the checkerboard scene moved by
 *  a genuine camera rotation at a known field of view, so the ground truth is
 *  an exact rotation and not an approximation of one.  Two things are checked.
 *
 *  1. That the rotational model IS the current model in the long lens limit
 *     (test_fov_degenerates_to_similarity).  This is a statement about
 *     geometry only, it needs no detector, and it is what licenses fov = 0
 *     staying bit identical to today.
 *
 *  2. That it stops being the current model as the lens widens, by the amount
 *     the geometry predicts, and that this is visible end to end through real
 *     motion detection and the real fit (test_fov_similarity_gap).
 *
 *  See docs/fov_correction.md.
 */

/* --- 1. the long lens limit ---------------------------------------------- */

/* Evaluates the rotational backward map for one rotation, without the clip
   machinery: same arithmetic as fcRotMap with the lens left out. */
static void fovRotPoint(double yaw, double pitch, double roll, double f,
                        double xd, double yd, double* xs, double* ys){
  FCMapCtx ctx;
  ctx.f = f;
  ctx.useLens = 0;
  fcRotation(yaw, pitch, roll, ctx.r);
  fcRotMap(&ctx, xd, yd, xs, ys);
}

/* Largest distance, over a grid covering the whole frame including its
   corners, between where the rotational model says a destination pixel looks
   and where the similarity model says it looks.

   The rotation is the one that the similarity transform t degenerates FROM:
   yaw = t.x/f, pitch = t.y/f, roll = t.alpha.  Holding t fixed while f grows
   is the limit that matters -- the same picture motion, taken through an ever
   longer lens -- and it is the limit docs/fov_correction.md claims is
   exact. */
static double fovMaxModelGap(VSTransform t, double f){
  double worst = 0;
  int i, j;
  for(j=0; j<=12; j++)
    for(i=0; i<=16; i++){
      double xd = i * (LC_WIDTH  - 1) / 16.0;
      double yd = j * (LC_HEIGHT - 1) / 12.0;
      double rx, ry, ax, ay, d;
      fovRotPoint(t.x/f, t.y/f, t.alpha, f, xd, yd, &rx, &ry);
      lcBackwardAffine(&t, xd, yd, &ax, &ay);
      d = sqrt((rx-ax)*(rx-ax) + (ry-ay)*(ry-ay));
      if(d > worst) worst = d;
    }
  return worst;
}

/* The gap vanishes quadratically in 1/f, so it is not merely small at a long
   focal length, it is the right kind of small.  Both ways of getting the
   convention wrong are caught here and neither would be caught by an
   eyeball:

     - a sign flip on yaw leaves a gap of 2|t.x| = 24 px at every f;
     - applying roll outermost instead of innermost leaves |d| sin(alpha),
       about 0.31 px, CONSTANT in f -- which passes any fixed tolerance loose
       enough for the wide end and still poisons the ground truth.

   The quadratic assertion catches both because it constrains the shape of
   the sequence, not just its last term. */
static void test_fov_degenerates_to_similarity(void){
  static const double FS[] = {320.0, 1000.0, 3000.0, 10000.0, 30000.0};
  VSTransform t = new_transform(12.0, 9.0, 1.2*M_PI/180.0, 0, 0, 0, 0);
  double prev = 0;
  int i;

  fprintf(stderr, "--- rotational vs similarity model, t = (12, 9, 1.2 deg) ---\n");
  for(i=0; i<(int)(sizeof(FS)/sizeof(FS[0])); i++){
    double f   = FS[i];
    double gap = fovMaxModelGap(t, f);
    double fov = 2.0 * atan((LC_WIDTH/2.0)/f) * 180.0 / M_PI;
    fprintf(stderr, "  f = %8.0f (fov %5.1f deg): max gap %8.4f px\n", f, fov, gap);

    if(i > 0){
      /* Ten times the focal length must buy at least fifty times less gap
         (a hundred if it were purely quadratic; the slack absorbs the
         third-order term, which is still material at the short end). */
      double ratio = FS[i]/FS[i-1];
      test_bool(gap < prev / (0.5 * ratio * ratio));
    }
    prev = gap;
  }
  /* And the far end is flatly negligible: a thirtieth of a pixel at 30000 is
     three thousand times finer than the 0.02 percent detection floor. */
  test_bool(prev < 0.01);

  /* The near end is not: at f = 320, a 90 degree lens, the two models
     disagree by more than a pixel across the frame.  Asserted so that this
     test also fails if someone "fixes" the gap away. */
  test_bool(fovMaxModelGap(t, 320.0) > 1.0);
}

/* --- 2. the gap end to end ----------------------------------------------- */

/* Runs detection and the real gradient fit over a clip and returns the
   largest per frame error against the exact rotational ground truth.

   Conventions copied from test_localmotion2transform(): one VSMotionDetect
   per sequence, called from frame 0 so the first call establishes the
   reference and is not scored, and the recovered transform compared against
   the NEGATED applied step, because vsDoTransform's map is backward. */
typedef struct { double xy; double alpha; } FovErr;

static FovErr fovFitClip(double fovDeg, double k, double cfgFov, const char* label){
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig(label);
  VSTransformConfig    tdconf = vsTransformGetDefaultConfig(label);
  VSMotionDetect md;
  VSTransformData td;
  VSFrameInfo fi;
  VSFrame frames[FC_NUM_FRAMES];
  FovErr worst = {0, 0};
  double f = fcFocal(fovDeg);
  int i;

  fcGenerateClip(frames, &fi, PF_RGB24, fovDeg, k, FC_NUM_FRAMES);

  test_bool(vsMotionDetectInit(&md, &mdconf, &fi) == VS_OK);
  md.conf.numThreads = 1;
  tdconf.fov = cfgFov;
  test_bool(vsTransformDataInit(&td, &tdconf, &fi, &fi) == VS_OK);

  fprintf(stderr, "--- %s: fov %.0f deg (f = %.0f), k = %.2f, model fov %.0f ---\n",
          label, fovDeg, f, k, cfgFov);
  for(i=0; i<FC_NUM_FRAMES; i++){
    LocalMotions lms;
    VSTransform got, want, diff;

    VSTransform simple;
    test_bool(vsMotionDetection(&md, &lms, &frames[i]) == VS_OK);
    got    = vsMotionsToTransform(&td, &lms, 0);
    simple = vsSimpleMotionsToTransform(fi, label, &lms);
    vs_vector_del(&lms);
    if(i == 0) continue;   /* no reference yet; nothing to score */

    want = mult_transform_(fcStepTransform(i, f), -1.0);
    diff = sub_transforms(&got, &want);
    fprintf(stderr, "  frame %i: want (%7.3f %7.3f %8.5f)  got (%7.3f %7.3f %8.5f)"
                    "  err (%7.3f %7.3f %8.5f)  [simple %7.3f %7.3f %8.5f]\n",
            i, want.x, want.y, want.alpha, got.x, got.y, got.alpha,
            diff.x, diff.y, diff.alpha, simple.x, simple.y, simple.alpha);

    if(fabs(diff.x)     > worst.xy)    worst.xy    = fabs(diff.x);
    if(fabs(diff.y)     > worst.xy)    worst.xy    = fabs(diff.y);
    if(fabs(diff.alpha) > worst.alpha) worst.alpha = fabs(diff.alpha);
  }
  fprintf(stderr, "  worst: %.3f px, %.5f rad\n", worst.xy, worst.alpha);

  vsTransformDataCleanup(&td);
  vsMotionDetectionCleanup(&md);
  for(i=0; i<FC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
  return worst;
}

/* The same shake, the same scene, the same detector and the same fit at three
   field of view settings.  The picture moves by identical amounts in all
   three (see the note on FC_X_AMP), so the only thing that varies is the
   perspective term, and the error is therefore attributable to the model and
   to nothing else.

   The numbers this pins, measured:

     10 deg   0.79 px   -- the detector's own floor on this scene
     50 deg   1.08 px
    110 deg   3.65 px   -- four and a half times the floor

   The wide bound is what fails today and passes once VSTransformConfig.fov
   is honoured; the narrow bound is what guards against "fixing" it by
   breaking the common case.  Both are set with roughly 50 percent headroom
   over the measured values, which is the room detector noise needs and no
   more -- a tolerance loose enough to swallow the effect would make the test
   decorative. */
static void test_fov_similarity_gap(void){
  FovErr narrow = fovFitClip( 10.0, 0.0, 0.0, "fov-narrow");
  FovErr mid    = fovFitClip( 50.0, 0.0, 0.0, "fov-mid");
  FovErr wide   = fovFitClip(110.0, 0.0, 0.0, "fov-wide");

  fprintf(stderr, "--- similarity fit error vs field of view ---\n");
  fprintf(stderr, "   10 deg: %.3f px   50 deg: %.3f px   110 deg: %.3f px\n",
          narrow.xy, mid.xy, wide.xy);

  test_bool(narrow.xy < 1.5);
  test_bool(mid.xy    < 2.0);
  test_bool(wide.xy   > 2.5);
  test_bool(wide.xy   > 2.0 * narrow.xy);
}

/* The pairing the field of view parameter only makes sense in: barrel
   distortion and a wide lens at once, which is what real wide glass actually
   delivers.

   Measured, the error is LOWER with the barrel than without -- 3.06 px
   against 3.65 -- and that is not noise, it is the two effects partly
   cancelling.  Perspective pushes the periphery outward relative to a
   translation; barrel pulls it inward.  Uncorrected wide footage therefore
   flatters the similarity model, and it follows that correcting the lens
   alone, without also modelling the field of view, can make the fit WORSE
   than leaving both wrong.  That is a real caveat for anyone enabling lens
   correction on wide footage today, and it is an argument for the sequencing
   docs/fov_correction.md already asks for rather than against it.

   Measurement only, no target asserted: what the number should be once both
   corrections are applied is the subject of the fov implementation, and
   pinning it here beforehand would only enshrine today's accident. */
static void test_fov_with_lens(void){
  FovErr bare = fovFitClip(110.0,  0.0, 0.0, "fov-wide-nolens");
  FovErr lens = fovFitClip(110.0, -0.25, 0.0, "fov-wide-barrel");
  fprintf(stderr, "--- wide lens, with and without barrel distortion ---\n");
  fprintf(stderr, "   k =  0.00: %.3f px    k = -0.25: %.3f px\n",
          bare.xy, lens.xy);
}

/* --- the figure ----------------------------------------------------------- */

/* Three panels of the same camera pose: the base scene, then frame 5 at 110
   degrees and at 10 degrees.  The two moved panels carry identical centre
   motion by construction, so everything that differs between them -- the
   outer cells sliding further than the inner ones -- is the perspective term
   the similarity model has no way to express.  Asserts nothing; reached only
   via --dumpFovClip. */
void fcDumpClip(void){
  VSFrameInfo fi, fiSheet;
  VSFrame sheet, wide[FC_NUM_FRAMES], narrow[FC_NUM_FRAMES];
  const int gutter = 8;
  int i;

  fcGenerateClip(wide,   &fi, PF_RGB24, 110.0, 0.0, FC_NUM_FRAMES);
  fcGenerateClip(narrow, &fi, PF_RGB24,  10.0, 0.0, FC_NUM_FRAMES);

  for(i=0; i<FC_NUM_FRAMES; i++){
    char name[64];
    snprintf(name, sizeof(name), "fovclip/wide_%03i.ppm", i);
    test_bool(storePPMImage(testOut(name), &wide[i], &fi));
    snprintf(name, sizeof(name), "fovclip/narrow_%03i.ppm", i);
    test_bool(storePPMImage(testOut(name), &narrow[i], &fi));
  }

  test_bool(vsFrameInfoInit(&fiSheet, 3*LC_WIDTH + 2*gutter, LC_HEIGHT,
                            PF_RGB24) != 0);
  vsFrameAllocate(&sheet, &fiSheet);
  fillFrameRGB(&sheet, &fiSheet, 128, 128, 128);
  lcBlit(&sheet, &fiSheet, 0,                     0, &wide[0],   &fi);
  lcBlit(&sheet, &fiSheet, LC_WIDTH + gutter,     0, &wide[5],   &fi);
  lcBlit(&sheet, &fiSheet, 2*(LC_WIDTH + gutter), 0, &narrow[5], &fi);
  test_bool(storePPMImage(testOut("fovclip/sheet.ppm"), &sheet, &fiSheet));
  vsFrameFree(&sheet);

  for(i=0; i<FC_NUM_FRAMES; i++){ vsFrameFree(&wide[i]); vsFrameFree(&narrow[i]); }
  fprintf(stderr, "dumped fov clip PPMs to %s/fovclip "
                  "(base | 110 deg | 10 deg sheet.ppm, same centre motion)\n",
          TEST_OUTPUT_DIR);
}

/* The point of the whole exercise: told the field of view, the same fit on
   the same footage recovers the rotation the similarity model could not. */
static void test_fov_model_recovers(void){
  FovErr blind  = fovFitClip(110.0, 0.0,   0.0, "fov-wide-blind");
  FovErr told   = fovFitClip(110.0, 0.0, 110.0, "fov-wide-told");
  fprintf(stderr, "--- 110 deg clip: fov = 0 vs fov = 110 ---\n");
  fprintf(stderr, "   blind: %.3f px    told: %.3f px\n", blind.xy, told.xy);
  /* Measured 3.65 -> 0.43 px.  The recovered value is not merely better, it
     is below the 0.79 px the DETECTOR floor costs on the 10 degree clip --
     that is, told the field of view, a 110 degree lens is fitted at least as
     well as a 10 degree one, which is the whole claim. */
  test_bool(blind.xy > 2.5);
  test_bool(told.xy  < 0.7);
  test_bool(told.xy  < 0.25 * blind.xy);
}

void test_fov_model(void){
  test_fov_degenerates_to_similarity();
  test_fov_similarity_gap();
  test_fov_with_lens();
  test_fov_model_recovers();
}
