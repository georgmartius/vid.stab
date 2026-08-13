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

/* --- 3. the warp ---------------------------------------------------------- */

/* The fit being right buys nothing if the renderer still draws the old model,
   and an image that merely looks plausible will not catch a perspective term
   that is half the size it should be.  So this borrows the pattern
   test_lensmap_fixed_reference established: run the shipping warp loop, then
   recompute every destination pixel's source coordinate independently with
   vsLensMapBackward in double precision and sample the SAME interpolator at
   it.  Interpolator behaviour cancels exactly, leaving only the geometry
   under test.

   It covers both loops (planar and packed), both backends (the fixed-point
   one that ships and the float one built under -DTESTING), and a 4:2:2
   format, whose two axes are to different scales -- the case where a focal
   length scaled per plane instead of applied in luma units would be wrong,
   and the case 4:2:0 test footage silently passes.  See issue #79 and
   commit 9e2f8b7 for the same bug in the rotation. */
static void fovWarpVsReference(VSPixelFormat pf, int packed, int useFloat,
                               double fovDeg, const char* name){
  VSTransformConfig cfg = vsTransformGetDefaultConfig(name);
  VSTransformData td;
  VSFrameInfo fi;
  VSFrame src, dest;
  VSLensPlaneMap ref;
  VSTransform t = null_transform();
  uint32_t seed = 7;
  double sumDiff = 0;
  int n = 0, maxDiff = 0, x, y;

  t.x = 6.25; t.y = -3.5; t.alpha = 0.008;

  test_bool(vsFrameInfoInit(&fi, 320, 240, pf) != 0);
  vsFrameAllocate(&src, &fi);
  ldFillTexture(&src, &fi, &seed);
  vsFrameAllocate(&dest, &fi);

  cfg.fov            = fovDeg;
  cfg.lensCorrection = VSLensCorrectOff;
  cfg.interpolType   = VS_BiLinear;
  cfg.crop           = VSCropBorder;   /* so the out-of-frame default is 16 */
  cfg.optZoom        = 0;
  test_bool(vsTransformDataInit(&td, &cfg, &fi, &fi) == VS_OK);

  test_bool(vsTransformPrepare(&td, &src, &dest) == VS_OK);
  if(packed)
    test_bool((useFloat ? transformPacked_float(&td, t) : transformPacked(&td, t)) == VS_OK);
  else
    test_bool((useFloat ? transformPlanar_float(&td, t) : transformPlanar(&td, t)) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);

  /* An "off" map per plane: no radial terms, correct plane geometry, and the
     frame's focal length.  EVERY plane is checked, which is the point of
     including a 4:2:2 format -- on 4:2:0 the two axes are subsampled equally
     and a per-plane scaling of f would cancel out, so luma-only coverage
     would pass a wrong implementation. */
  {
    int planes = packed ? 1 : fi.planes;
    int p;
    for(p=0; p<planes; p++){
      int pw = packed ? fi.width  : CHROMA_SIZE(fi.width,  vsGetPlaneWidthSubS(&fi, p));
      int ph = packed ? fi.height : CHROMA_SIZE(fi.height, vsGetPlaneHeightSubS(&fi, p));
      uint8_t def = p == 0 ? 0 : 0x80;
      test_bool(vsLensPlaneMapInit(&ref, &fi, &fi, p, 0.0, VSLensCorrectOff) == VS_OK);
      ref.f = focal_from_fov(fovDeg, fi.width);
      test_bool(ref.f > 0.0);

      for(y=0; y<ph; y++)
        for(x=0; x<pw; x++){
          double xs, ys;
          uint8_t expected, actual;
          int diff;
          test_bool(vsLensMapBackward(&ref, &t, x, y, &xs, &ys) == VS_OK);
          if(packed){
            int c;
            for(c=0; c<fi.bytesPerPixel; c++){
              if(useFloat)
                interpolateN_float(&expected, (float)xs, (float)ys, src.data[0],
                                   src.linesize[0], fi.width, fi.height,
                                   fi.bytesPerPixel, (uint8_t)c, 16);
              else
                interpolateN(&expected, lmDToFp16(xs), lmDToFp16(ys), src.data[0],
                             src.linesize[0], fi.width, fi.height,
                             fi.bytesPerPixel, (uint8_t)c, 16);
              actual = dest.data[0][y*dest.linesize[0] + x*fi.bytesPerPixel + c];
              diff = actual > expected ? actual - expected : expected - actual;
              sumDiff += diff; n++;
              if(diff > maxDiff) maxDiff = diff;
            }
            continue;
          }
          if(useFloat)
            td.interpolate_float(&expected, (float)xs, (float)ys, src.data[p],
                                 src.linesize[p], pw, ph, def);
          else
            td.interpolate(&expected, lmDToFp16(xs), lmDToFp16(ys), src.data[p],
                           src.linesize[p], pw, ph, def);
          actual = dest.data[p][y*dest.linesize[p]+x];
          diff = actual > expected ? actual - expected : expected - actual;
          sumDiff += diff; n++;
          if(diff > maxDiff) maxDiff = diff;
        }
      vsLensPlaneMapFree(&ref);
    }
  }

  fprintf(stderr, "  %-28s fov=%3.0f: mean=%.4f max=%d\n",
          name, fovDeg, sumDiff/n, maxDiff);
  /* Same bounds and the same reasoning as test_lensmap_fixed_reference: the
     mean carries the guarantee, the max only catches a systematic break,
     because two computations agreeing to a few thousandths of a pixel can
     still land either side of an interpolator's discontinuity. */
  test_bool(sumDiff/n <= 0.15);
  test_bool(maxDiff   <= 150);

  vsFrameFree(&src); vsFrameFree(&dest);
  vsTransformDataCleanup(&td);
}

static void test_fov_warp_against_reference(void){
  fprintf(stderr, "--- warp loops vs the double-precision reference map ---\n");
  fovWarpVsReference(PF_GRAY8,   0, 0,  90.0, "planar fixed, gray");
  fovWarpVsReference(PF_GRAY8,   0, 1,  90.0, "planar float, gray");
  fovWarpVsReference(PF_YUV420P, 0, 0, 110.0, "planar fixed, 4:2:0");
  fovWarpVsReference(PF_YUV422P, 0, 0, 110.0, "planar fixed, 4:2:2");
  fovWarpVsReference(PF_YUV422P, 0, 1, 110.0, "planar float, 4:2:2");
  fovWarpVsReference(PF_RGB24,   1, 0, 110.0, "packed fixed, rgb");
  fovWarpVsReference(PF_RGB24,   1, 1, 110.0, "packed float, rgb");
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

/* --- 4. is the field of view identifiable? -------------------------------- */

/* Exploratory: fit the clip at a range of assumed field of view values and
   report the residual of each fit.  If the rotational model is identifiable
   from local motions at all, this curve has a minimum at the truth, and an
   estimator is a search over it.  If it is flat, or if its minimum wanders
   with the lens, no estimator can work and the parameter has to stay a user
   input.  Prints only. */
static double fovResidualAt(const LocalMotions* lmsAll, int nFrames,
                            const VSFrameInfo* fi, double cfgFov){
  VSTransformConfig cfg = vsTransformGetDefaultConfig("fov-sweep");
  VSTransformData td;
  double total = 0;
  int i;
  cfg.fov = cfgFov;
  test_bool(vsTransformDataInit(&td, &cfg, fi, fi) == VS_OK);
  for(i=1; i<nFrames; i++){
    const LocalMotions* lms = &lmsAll[i];
    VSTransform t = vsMotionsToTransform(&td, lms, 0);
    PreparedTransform pt = prepare_transform_fov(&t, fi,
                             focal_from_fov(cfgFov, fi->width));
    int j, n = vs_vector_size(lms);
    double e = 0;
    for(j=0; j<n; j++){
      LocalMotion* m = LMGet(lms, j);
      double vx, vy;
      transform_vec_double(&vx, &vy, &pt, (Vec*)&m->f);
      vx -= m->f.x + m->v.x; vy -= m->f.y + m->v.y;
      e += vx*vx + vy*vy;
    }
    total += e / (n > 0 ? n : 1);
  }
  vsTransformDataCleanup(&td);
  return total / (nFrames - 1);
}

/* Returns the residual at the best candidate as a fraction of the residual
   the similarity model (fov = 0) leaves.  Below 1 means the data prefers some
   rotational model; near 1 means the residual is flat and the field of view
   is not identifiable from this footage at all. */
static double fovSweep(double clipFov, double k, const char* label){
  static const double CAND[] = {0.0, 40.0, 60.0, 80.0, 90.0, 100.0, 110.0,
                                120.0, 130.0, 150.0};
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig(label);
  VSMotionDetect md;
  VSFrameInfo fi;
  VSFrame frames[FC_NUM_FRAMES];
  LocalMotions lmsAll[FC_NUM_FRAMES];
  double best = 0, base = 0; int bestI = 0;
  int i;

  fcGenerateClip(frames, &fi, PF_RGB24, clipFov, k, FC_NUM_FRAMES);
  test_bool(vsMotionDetectInit(&md, &mdconf, &fi) == VS_OK);
  md.conf.numThreads = 1;
  for(i=0; i<FC_NUM_FRAMES; i++)
    test_bool(vsMotionDetection(&md, &lmsAll[i], &frames[i]) == VS_OK);

  fprintf(stderr, "--- %s: residual vs assumed fov (clip is %.0f deg, k=%.2f) ---\n",
          label, clipFov, k);
  for(i=0; i<(int)(sizeof(CAND)/sizeof(CAND[0])); i++){
    double r = fovResidualAt(lmsAll, FC_NUM_FRAMES, &fi, CAND[i]);
    fprintf(stderr, "   fov %5.0f: residual %10.5f%s\n", CAND[i], r,
            (i == 0 ? "   (similarity model)" : ""));
    if(i == 0) base = r;                       /* CAND[0] is the fov = 0 case */
    if(i == 0 || r < best){ best = r; bestI = i; }
  }
  fprintf(stderr, "   -> minimum at fov %.0f (truth %.0f), %.1f%% of the "
                  "similarity residual\n",
          CAND[bestI], clipFov, 100.0*best/base);

  for(i=0; i<FC_NUM_FRAMES; i++) vs_vector_del(&lmsAll[i]);
  vsMotionDetectionCleanup(&md);
  for(i=0; i<FC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
  return best/base;
}

/* Whether fov could be ESTIMATED rather than supplied, which is the obvious
   thing to want given that k already is (VSTransformConfig.lensK).  Measured
   here, the answer is "only on genuinely wide glass", and this test exists to
   keep that answer honest rather than to gate a feature.

   At 110 degrees the residual curve has a clear interior minimum, 13 percent
   below what the similarity model leaves, and a barrel lens on top does not
   destroy it -- so a search would find something.  Two caveats, both visible
   above: the minimum sits at 100 rather than 110, biased low by about a
   tenth, and at 60 degrees the curve is FLAT -- 108.17 at fov = 0 against
   108.05 at its best, a fifth of a percent, which is noise.  An estimator run
   on 60 degree footage would return an arbitrary number, and a wrong fov is
   worse than no fov (see VSTransformConfig.fov).

   So an estimator would need a confidence gate of its own -- like the lens
   estimator's "determined" -- keyed on the depth of the minimum, and it would
   have to leave fov at 0 far more often than it sets it.  That is why fov
   ships as a user parameter: the bounds below say what any future estimator
   has to respect, not that one exists. */
void test_fov_identifiability(void){
  double wide   = fovSweep(110.0,  0.0,  "sweep-110-nolens");
  double narrow = fovSweep( 60.0,  0.0,  "sweep-60-nolens");
  double barrel = fovSweep(110.0, -0.25, "sweep-110-barrel");

  fprintf(stderr, "--- best residual as a fraction of the similarity one ---\n");
  fprintf(stderr, "   110 deg: %.3f    60 deg: %.3f    110 deg + barrel: %.3f\n",
          wide, narrow, barrel);

  /* A wide lens leaves a signal worth searching... */
  test_bool(wide   < 0.95);
  test_bool(barrel < 0.95);
  /* ...and a moderate one does not.  Asserted in the direction that matters:
     if this ever drops well below 1, the flatness finding is stale and the
     case for gating an estimator needs re-examining. */
  test_bool(narrow > 0.98);
}

void test_fov_model(void){
  test_fov_degenerates_to_similarity();
  test_fov_similarity_gap();
  test_fov_with_lens();
  test_fov_warp_against_reference();
  test_fov_model_recovers();
}

/* probe: does fov survive the lens-aware fit path? */

/* --- 5. the lens estimator is blind to all of this ------------------------ */

/* Set by the two estimator helpers below so the calibration study can read the
   reported standard error alongside the estimate.  Not thread safe and does
   not need to be: the tests run these one at a time. */
static double fovLastSigma = 0;

/* Where in the pipeline the rotational model is and is not applied, checked
   rather than assumed.

   MOTION ESTIMATION does not use it, and does not need to.  Its one call to
   prepare_transform (motiondetect.c, fieldSearchOffset) only centres the fine
   scan's search window on the coarse scan's guess; the fine scan then
   measures the displacement itself, so the model there is a hint, not a
   measurement.  VSMotionDetectConfig has no fov by design -- detection is
   streaming and fov is a transform-side parameter.

   LENS ESTIMATION does use a model, runs BEFORE the fit, and is fitted as a
   similarity (vsLensFitSimilarity).  It has no fov and no way to accept one.
   That is not a cosmetic gap: perspective expands the periphery and pincushion
   expands the periphery, so on a wide lens the estimator explains the
   perspective term with k and returns the wrong number.

   Measured below on footage whose true k is -0.25 throughout:

      20 deg   k = -0.2445 +- 0.0103   determined -- correct
      40 deg   k = -0.1454 +- 0.0128   determined -- 42 percent too small
      70 deg   k = +0.0487 +- 0.0107   determined -- WRONG SIGN
     110 deg   k = +0.2000 +- 0.0174   not determined (at the search bound)

   The uncertainties are the tell.  They stay around 0.01 while the estimate
   drifts by 0.3, because this is a systematic bias from a misspecified model,
   not scatter -- and "determined" keys on the uncertainty, which cannot see
   bias.  So at 40 and 70 degrees the estimator is confidently wrong and the
   result is USED: it is handed to vsTransformSetLensK and a wrong distortion
   is applied to the picture.  Only at 110 does it fail safe, and only because
   it runs into the bound.

   This is a defect in the lens estimator that predates fov -- it needs no fov
   set to happen, and this clip is simply the first thing to have shown it. */
static double fovEstimateKf(double clipFov, double trueK, double cfgFov,
                            const char* label){
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig(label);
  VSLensEstimateConfig ecfg = vsLensEstimateGetDefaultConfig();
  VSMotionDetect md;
  VSFrameInfo fi;
  VSFrame frames[FC_NUM_FRAMES];
  VSManyLocalMotions mlms;
  VSLensEstimate est;
  int i;

  fcGenerateClip(frames, &fi, PF_RGB24, clipFov, trueK, FC_NUM_FRAMES);
  test_bool(vsMotionDetectInit(&md, &mdconf, &fi) == VS_OK);
  md.conf.numThreads = 1;
  vs_vector_init(&mlms, FC_NUM_FRAMES);
  for(i=0; i<FC_NUM_FRAMES; i++){
    LocalMotions lms;
    test_bool(vsMotionDetection(&md, &lms, &frames[i]) == VS_OK);
    vs_vector_append_dup(&mlms, &lms, sizeof(LocalMotions));
  }

  ecfg.f = focal_from_fov(cfgFov, fi.width);
  est = vsEstimateLensDistortion(&fi, &mlms, &ecfg);
  fovLastSigma = est.uncertainty;
  fprintf(stderr, "  %-22s true k=%+.2f -> k=%+.4f +- %.4f  %s\n",
          label, trueK, est.k, est.uncertainty,
          est.determined ? "DETERMINED (and used)" : "not determined");

  vsMotionDetectionCleanup(&md);
  for(i=0; i<FC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
  return est.k;
}

static double fovEstimateK(double clipFov, double trueK, const char* label){
  return fovEstimateKf(clipFov, trueK, 0.0, label);
}

void test_fov_lens_estimator_coupling(void){
  const double K = -0.25;
  double k20, k40, k70, k110;

  fprintf(stderr, "--- lens estimate vs field of view (the estimator has no fov) ---\n");
  k20  = fovEstimateK( 20.0, K, "20 deg");
  k40  = fovEstimateK( 40.0, K, "40 deg");
  k70  = fovEstimateK( 70.0, K, "70 deg");
  k110 = fovEstimateK(110.0, K, "110 deg");

  /* Where the similarity model holds, the estimator is right.  This is the
     assertion that must survive any fix. */
  test_bool(fabs(k20 - K) < 0.02);

  /* The bias is monotone in the field of view and toward pincushion, which is
     what identifies the perspective term as its cause rather than noise. */
  test_bool(k20 < k40);
  test_bool(k40 < k70);
  test_bool(k70 < k110);

  /* Blind, at 70 degrees, the estimate has the wrong sign.  Asserted so the
     account above cannot go stale silently. */
  test_bool(k70 > 0.0);

  /* Told the field of view, the same footage through the same estimator:
     -0.2405 at 40 degrees and -0.2473 at 70, against -0.1472 and +0.0504
     blind.  The sign error is gone and both are within the same 0.02 the 20
     degree case meets unaided.

     110 degrees lands at -0.3763 -- much closer than the blind +0.2000, but
     not within tolerance, and that residual error is NOT the estimator's
     model.  test_fov_estimator_exact() below feeds the same estimator
     correspondences computed straight from the model and gets -0.2537 there,
     so what is left at 110 degrees is the DETECTOR: block matching assumes a
     local translation, and at 110 degrees the perspective term stretches a
     16 px field enough that the match it returns is biased.  Fixing that
     would mean changing the matcher, not the estimator. */
  fprintf(stderr, "--- the same clips, with the estimator told the fov ---\n");
  k40  = fovEstimateKf( 40.0, K,  40.0, "40 deg, told");
  k70  = fovEstimateKf( 70.0, K,  70.0, "70 deg, told");
  k110 = fovEstimateKf(110.0, K, 110.0, "110 deg, told");
  test_bool(fabs(k40 - K) < 0.02);
  test_bool(fabs(k70 - K) < 0.02);
  /* At 110 the claim is only that it is much better, and on the right side. */
  test_bool(k110 < 0.0);
  test_bool(fabs(k110 - K) < 0.35*fabs(0.2000 - K));
}

/* --- 6. the estimator on exact correspondences ---------------------------- */

/* Feeds the estimator matches computed straight from the model -- q =
   D_k(S(U_k(p))) with S a known rotation at a known f -- so nothing the
   detector does can be blamed.  This is what separates "the estimator's model
   is wrong" from "the measurements going into it are noisy". */
static double fovExactK(double fovDeg, double trueK, double cfgFov,
                        int quantise, const char* label){
  VSFrameInfo fi;
  VSLensDistortion ld;
  VSLensEstimateConfig ecfg = vsLensEstimateGetDefaultConfig();
  VSLensEstimate est;
  VSManyLocalMotions mlms;
  double f = fcFocal(fovDeg);
  int frame, gx, gy, i;

  test_bool(vsFrameInfoInit(&fi, LC_WIDTH, LC_HEIGHT, PF_GRAY8) != 0);
  ld = vsLensDistortionInit(&fi, trueK);

  vs_vector_init(&mlms, FC_NUM_FRAMES);
  for(frame=1; frame<FC_NUM_FRAMES; frame++){
    LocalMotions lms;
    double yaw, pitch, roll, rb[9], rf[9];
    fcStepAngles(frame, f, &yaw, &pitch, &roll);
    /* the step's forward map, as the estimator's S */
    rotation_matrix_backward(yaw, pitch, roll, rb);
    rf[0]=rb[0]; rf[1]=rb[3]; rf[2]=rb[6];
    rf[3]=rb[1]; rf[4]=rb[4]; rf[5]=rb[7];
    rf[6]=rb[2]; rf[7]=rb[5]; rf[8]=rb[8];

    vs_vector_init(&lms, 16*12);
    for(gy=0; gy<12; gy++)
      for(gx=0; gx<16; gx++){
        LocalMotion lm;
        double px = 20 + gx*(LC_WIDTH -40)/15.0;
        double py = 20 + gy*(LC_HEIGHT-40)/11.0;
        double ux, uy, ax, ay, X, Y, Z, wx, wy, qx, qy;
        if(vsLensUndistortPoint(&ld, px, py, &ux, &uy) != VS_OK) continue;
        ax = ux - ld.cx; ay = uy - ld.cy;
        X = rf[0]*ax + rf[1]*ay + rf[2]*f;
        Y = rf[3]*ax + rf[4]*ay + rf[5]*f;
        Z = rf[6]*ax + rf[7]*ay + rf[8]*f;
        wx = f*X/Z + ld.cx; wy = f*Y/Z + ld.cy;
        if(vsLensDistortPoint(&ld, wx, wy, &qx, &qy) != VS_OK) continue;
        lm.f.x = (int16_t)lrint(px);
        lm.f.y = (int16_t)lrint(py);
        /* LocalMotion.v is integral, so the detector could never deliver more
           than this; quantise == 0 keeps the exact double to separate the
           model's error from the quantiser's. */
        lm.v.x = (int16_t)(quantise ? lrint(qx - lrint(px)) : lrint(qx - px));
        lm.v.y = (int16_t)(quantise ? lrint(qy - lrint(py)) : lrint(qy - py));
        lm.f.size = 32; lm.contrast = 1.0; lm.match = 1.0;
        vs_vector_append_dup(&lms, &lm, sizeof(LocalMotion));
      }
    vs_vector_append_dup(&mlms, &lms, sizeof(LocalMotions));
  }

  ecfg.f = focal_from_fov(cfgFov, fi.width);
  est = vsEstimateLensDistortion(&fi, &mlms, &ecfg);
  fovLastSigma = est.uncertainty;
  fprintf(stderr, "  %-26s true k=%+.2f -> k=%+.4f +- %.4f  %s\n",
          label, trueK, est.k, est.uncertainty,
          est.determined ? "determined" : "not determined");
  for(i=0; i<vs_vector_size(&mlms); i++) vs_vector_del(VSMLMGet(&mlms, i));
  return est.k;
}

void test_fov_estimator_exact(void){
  double k70, k110q, k110, kzero, kblind;
  fprintf(stderr, "--- estimator on exact model correspondences ---\n");
  k70    = fovExactK( 70.0, -0.25,  70.0, 1, "70 deg, told, quantised");
  k110q  = fovExactK(110.0, -0.25, 110.0, 1, "110 deg, told, quantised");
  k110   = fovExactK(110.0, -0.25, 110.0, 0, "110 deg, told, unquantised");
  kzero  = fovExactK(110.0,  0.0,  110.0, 0, "110 deg, k=0, unquantised");
  kblind = fovExactK(110.0, -0.25,   0.0, 0, "110 deg, BLIND, unquantised");

  /* Told f, the model is right at every field of view -- this is the
     assertion that says threading f into the estimator actually worked. */
  test_bool(fabs(k70   + 0.25) < 0.02);
  test_bool(fabs(k110  + 0.25) < 0.02);
  test_bool(fabs(kzero)        < 0.02);   /* and it does not invent distortion */

  /* Quantising the displacements to integers, which is all a detector can
     deliver, changes nothing: the 110 degree shortfall on real footage is
     not the quantiser either. */
  test_bool(fabs(k110q - k110) < 0.01);

  /* Blind, on correspondences with no measurement error whatsoever, it still
     returns the wrong sign.  That is the cleanest possible statement that the
     old behaviour was a misspecified model and not noise. */
  test_bool(kblind > 0.0);
}

/* --- 7. is the confidence gate calibrated under the rotational model? ------ */

/* maxUncertainty (default 0.02) gates whether an estimate is used at all.  It
   came from 71360a0, which introduced uncertainty = residual/sqrt(N*curvature)
   and calibrated it once: 0.0029 predicted against 0.0049 actual under 0.5 px
   noise, versus 0.47 for the unidentifiable case it had to reject.  0.02 is a
   round number in the middle of that gap, not a tuned one.

   The criterion that commit established is the one applied here: the reported
   standard error must BOUND the actual error.  Where it does, raising the
   threshold trades away nothing but coverage; where it does not, raising the
   threshold admits estimates whose error nobody measured.  Printed as a ratio
   so the two regimes are visible at a glance. */
void test_fov_uncertainty_calibration(void){
  static const double FOVS[] = {20.0, 40.0, 70.0, 90.0, 110.0};
  const double K = -0.25;
  double sPrev = 0;
  int i;

  fprintf(stderr, "--- does the reported sigma bound the actual error? ---\n");
  fprintf(stderr, "    (told f throughout; ratio > 1 means sigma UNDER-predicts)\n");
  for(i=0; i<(int)(sizeof(FOVS)/sizeof(FOVS[0])); i++){
    double kEx, sEx, kDet, sDet;
    char lbl[32];
    snprintf(lbl, sizeof(lbl), "%.0f exact", FOVS[i]);
    kEx = fovExactK(FOVS[i], K, FOVS[i], 1, lbl);
    sEx = fovLastSigma;
    snprintf(lbl, sizeof(lbl), "%.0f detector", FOVS[i]);
    kDet = fovEstimateKf(FOVS[i], K, FOVS[i], lbl);
    sDet = fovLastSigma;
    fprintf(stderr, "  fov %3.0f | exact: sigma %.4f err %.4f ratio %5.2f"
                    " | detector: sigma %.4f err %.4f ratio %5.2f\n",
            FOVS[i], sEx, fabs(kEx-K), fabs(kEx-K)/(sEx>0?sEx:1e-9),
            sDet, fabs(kDet-K), fabs(kDet-K)/(sDet>0?sDet:1e-9));

    /* The reported sigma grows with the field of view -- 0.011 at 20 degrees
       to 0.042 at 110 -- so the gate does get harder to pass as the lens
       widens.  That is the honest part. */
    if(i > 0) test_bool(sDet > 0.5*sPrev);
    sPrev = sDet;

    if(FOVS[i] <= 70.0){
      /* Up to 70 degrees the estimate is good and sigma bounds it.  These are
         the cases a higher threshold would legitimately admit. */
      test_bool(fabs(kDet - K) < 0.02);
      test_bool(sDet < 0.05);
    } else {
      /* At 90 and above the error jumps by an order of magnitude, to 0.12,
         while sigma barely doubles.  It is detector bias, and a standard
         error computed from scatter cannot see bias at any threshold.

         This pair is the answer to "why not raise maxUncertainty to 0.05":
         these estimates have sigma below 0.05, so 0.05 would call them
         determined and apply them -- a k off by 0.12, which is 21 px of
         residual distortion at the corner of a 640x480 frame.  0.02 rejects
         them, but by luck rather than by design: nothing about 0.02 was
         chosen with this regime in mind. */
      test_bool(fabs(kDet - K) > 0.10);
      test_bool(sDet < 0.05);
      test_bool(sDet > 0.02);
    }
  }
}
