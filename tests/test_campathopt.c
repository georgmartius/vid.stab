/*
 * tests for the L1 optimal camera path (l1campathoptimization.c)
 *
 * This file is included by tests.c and only compiled when an LP solver
 * backend is available.
 */

#include "l1campathoptimization.h"
#include "lpsolver.h"

/* optimal objectives of the two instances below, produced by
   docs/l1campath-reference.py (scipy/HiGHS) */
#define L1_REFERENCE_OBJECTIVE_24  1981.95228567
#define L1_REFERENCE_OBJECTIVE_200 23046.0705989

/* ---------------------------------------------------------------- helpers */

/** A deterministic synthetic camera path: a slow pan and sway with a slow
    rotation, plus a high frequency shake on top.  No RNG is involved so that
    the very same path can be reproduced by the reference implementation. */
static VSTransformLS campath_ground_truth(int t){
  double s = (double)t;
  VSTransformLS c;
  c.x = 12.0 * s + 5.0 * sin(s * 1.7);
  c.y = 20.0 * sin(s * 0.05) + 3.0 * cos(s * 2.3);
  double ang = 0.02 * sin(s * 0.11) + 0.01 * sin(s * 1.9);
  c.a = cos(ang);
  c.b = sin(ang);
  c.extra = 0;
  return c;
}

/** frame-pair transforms of that path: F_t = C_{t-1}^{-1} C_t */
static void campath_frame_pairs(VSTransformLS* F, int N){
  F[0] = id_transformLS();
  for (int t = 1; t < N; t++) {
    VSTransformLS prev = campath_ground_truth(t - 1);
    VSTransformLS cur  = campath_ground_truth(t);
    VSTransformLS inv  = invert_transformLS(&prev);
    F[t] = concat_transformLS(&inv, &cur);
  }
}

/** L1 norm of the n-th forward difference of a path, with the a/b part scaled
    up so that it is commensurable with the pixel-valued x/y part */
static double campath_diffnorm(const VSTransformLS* P, int N, int order){
  double* d = (double*)vs_malloc(sizeof(double) * N * 4);
  for (int t = 0; t < N; t++) {
    d[4*t+0] = P[t].x; d[4*t+1] = P[t].y;
    d[4*t+2] = P[t].a * 100.0; d[4*t+3] = P[t].b * 100.0;
  }
  int len = N;
  for (int o = 0; o < order; o++) {
    for (int t = 0; t < len - 1; t++)
      for (int k = 0; k < 4; k++) d[4*t+k] = d[4*(t+1)+k] - d[4*t+k];
    len--;
  }
  double sum = 0.0;
  for (int t = 0; t < len; t++)
    for (int k = 0; k < 4; k++) sum += fabs(d[4*t+k]);
  vs_free(d);
  return sum;
}

static VSL1Config campath_testconfig(double width, double height){
  VSL1Config c = vsL1GetDefaultConfig();
  c.frameWidth  = width;
  c.frameHeight = height;
  c.cropRatio   = 1.0 / 1.15;
  return c;
}

/* ------------------------------------------------- row and column indexing */

/** The row and column numbering must be a bijection onto 0..num-1, in the
    order the layout comment in l1campathoptimization.c describes.  A gap or an
    overlap here silently corrupts the whole constraint matrix, which is why
    this is checked exhaustively rather than spot-checked. */
void test_l1_indices(void){
  for (int N = 4; N <= 9; N++) {
    int expected = 0;
    for (int t = 0; t < N; t++) {
      test_bool(vs_l1_rowBase(t, N) == expected);
      for (int k = 0; k < 8; k++)
        test_bool(vs_l1_rowCorner(t, k, N) == expected++);
      for (int order = 1; order <= 3; order++) {
        if (t >= N - order) continue;
        for (int p = 0; p < 4; p++)
          for (int ul = 0; ul < 2; ul++)
            test_bool(vs_l1_row(order, t, p, ul, N) == expected++);
      }
    }
    test_bool(vs_l1_numrows(N) == expected);

    expected = 0;
    for (int group = 0; group <= 3; group++)
      for (int t = 0; t < N - group; t++)
        for (int p = 0; p < 4; p++)
          test_bool(vs_l1_col(group, t, p, N) == expected++);
    test_bool(vs_l1_numcols(N) == expected);
  }
}

/* ---------------------------------------------------- the LS transform type */

void test_l1_transformLS(void){
  VSTransformLS t = { 12.0, -7.0, 0.0, 0.0, 0 };
  t.a = 1.05 * cos(0.3);
  t.b = 1.05 * sin(0.3);

  /* t^-1 t == identity */
  VSTransformLS inv = invert_transformLS(&t);
  VSTransformLS id  = concat_transformLS(&inv, &t);
  test_bool(fabs(id.x - 0.0) < 1e-9);
  test_bool(fabs(id.y - 0.0) < 1e-9);
  test_bool(fabs(id.a - 1.0) < 1e-9);
  test_bool(fabs(id.b - 0.0) < 1e-9);

  /* concatenation must agree with applying the transforms one after another */
  VSTransformLS u = { -3.0, 8.0, 0.98 * cos(-0.1), 0.98 * sin(-0.1), 0 };
  VSTransformLS tu = concat_transformLS(&t, &u);
  double x1, y1, x2, y2;
  transformLS_vec(&x1, &y1, &u, 40.0, -25.0);
  transformLS_vec(&x1, &y1, &t, x1, y1);
  transformLS_vec(&x2, &y2, &tu, 40.0, -25.0);
  test_bool(fabs(x1 - x2) < 1e-9);
  test_bool(fabs(y1 - y2) < 1e-9);

  /* the (x,y,alpha,zoom) round trip must be lossless ... */
  VSTransform az = transformLStoAZ(&t);
  VSTransformLS back = transformAZtoLS(&az);
  test_bool(fabs(back.x - t.x) < 1e-9);
  test_bool(fabs(back.y - t.y) < 1e-9);
  test_bool(fabs(back.a - t.a) < 1e-9);
  test_bool(fabs(back.b - t.b) < 1e-9);

  /* ... and describe the same mapping as vid.stab's own prepare_transform() */
  VSFrameInfo fi;
  vsFrameInfoInit(&fi, 640, 480, PF_GRAY8);
  PreparedTransform pt = prepare_transform(&az, &fi);
  Vec v = { 320 + 40, 240 - 25 };
  double px, py;
  transform_vec_double(&px, &py, &pt, &v);
  transformLS_vec(&x1, &y1, &t, 40.0, -25.0);
  test_bool(fabs((px - 320) - x1) < 1e-6);
  test_bool(fabs((py - 240) - y1) < 1e-6);
}

/* --------------------------------------------------------- the optimization */

/** Solves the LP for the synthetic path and checks the three properties that
    make the result usable: the derivatives of the stabilized path go down, the
    crop window never leaves the frame, and the update transforms stay within
    the proximity bounds. */
static void test_l1_run(int N, double w1, double w2, double w3, const char* what){
  const double W = 640.0, H = 480.0;
  VSTransformLS* F = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  VSTransformLS* B = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  VSTransformLS* C = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  VSTransformLS* P = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  campath_frame_pairs(F, N);
  for (int t = 0; t < N; t++) C[t] = campath_ground_truth(t);

  VSL1Config conf = campath_testconfig(W, H);
  conf.w1 = w1; conf.w2 = w2; conf.w3 = w3;
  double objective = -1.0;
  int status = vsCameraPathOptimalL1LS(F, N, B, &conf, &objective);
  test_bool(status == VS_OK);
  if (status != VS_OK) { vs_free(F); vs_free(B); vs_free(C); vs_free(P); return; }
  test_bool(objective >= 0.0);

  for (int t = 0; t < N; t++) P[t] = concat_transformLS(&C[t], &B[t]);

  double before[3], after[3];
  for (int o = 1; o <= 3; o++) {
    before[o-1] = campath_diffnorm(C, N, o);
    after[o-1]  = campath_diffnorm(P, N, o);
  }
  fprintf(stderr, "  %-22s N=%3i  |D|: %8.2f -> %8.2f   |D2|: %8.2f -> %8.2f"
          "   |D3|: %8.2f -> %8.2f\n", what, N,
          before[0], after[0], before[1], after[1], before[2], after[2]);

  /* every weighted term must improve, the unweighted ones may not */
  if (w1 > 0) test_bool(after[0] < before[0]);
  if (w2 > 0) test_bool(after[1] < before[1]);
  if (w3 > 0) test_bool(after[2] < before[2]);
  /* The shake dominates the higher derivatives, so on a sequence long enough
     for the optimizer to have room they must drop a lot.  For very short
     sequences there are only a couple of difference terms and the bound is
     not meaningful. */
  if (N >= 20) {
    if (w2 > 0) test_bool(after[1] < 0.2 * before[1]);
    if (w3 > 0) test_bool(after[2] < 0.2 * before[2]);
  }

  /* inclusion: all four crop corners stay inside the frame */
  const double cw = W / 2.0 * conf.cropRatio, ch = H / 2.0 * conf.cropRatio;
  const double cx[4] = { -cw,  cw, cw, -cw };
  const double cy[4] = { -ch, -ch, ch,  ch };
  double worst = -1e30;
  for (int t = 0; t < N; t++) {
    for (int i = 0; i < 4; i++) {
      double px, py;
      transformLS_vec(&px, &py, &B[t], cx[i], cy[i]);
      worst = VS_MAX(worst, fabs(px) - W / 2.0);
      worst = VS_MAX(worst, fabs(py) - H / 2.0);
    }
    /* proximity */
    test_bool(B[t].a >= conf.minScale - 1e-6 && B[t].a <= conf.maxScale + 1e-6);
    test_bool(fabs(B[t].b) <= conf.maxSkewDev + 1e-6);
  }
  test_bool(worst <= 1e-6);

  vs_free(F); vs_free(B); vs_free(C); vs_free(P);
}

void test_l1_campath(void){
  test_l1_run(60,  10.0, 1.0, 100.0, "default weights");
  test_l1_run(200, 10.0, 1.0, 100.0, "default weights");
  /* fig. 8 of the paper: each single term on its own */
  test_l1_run(60,   1.0, 0.0,   0.0, "only |D|");
  test_l1_run(60,   0.0, 1.0,   0.0, "only |D^2|");
  test_l1_run(60,   0.0, 0.0,   1.0, "only |D^3|");
  /* the shortest sequence the third derivative is defined on */
  test_l1_run(4,   10.0, 1.0, 100.0, "minimal length");
}

/** The objective of two fixed instances, cross-checked against an independent
    implementation of the same program (scipy/HiGHS, see
    docs/l1campath-reference.py).  This is what catches a drift in the model
    that the property checks above would still accept.

    Since the returned update transforms are always feasible, the objective can
    never fall meaningfully below the optimum; it may sit slightly above it when
    the solver stops short of full convergence, which the built-in interior
    point method does on longer sequences. */
static void test_l1_reference_at(int N, double reference, double tolerance){
  VSTransformLS* F = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  VSTransformLS* B = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  campath_frame_pairs(F, N);
  VSL1Config conf = campath_testconfig(640.0, 480.0);
  double objective = -1.0;
  int status = vsCameraPathOptimalL1LS(F, N, B, &conf, &objective);
  test_bool(status == VS_OK);
  if (status == VS_OK) {
    double rel = (objective - reference) / reference;
    fprintf(stderr, "  N=%3i objective %.10g, optimum %.10g, %+.2e relative\n",
            N, objective, reference, rel);
    test_bool(rel > -1e-9);          // a feasible point cannot beat the optimum
    test_bool(rel < tolerance);
  }
  vs_free(F);
  vs_free(B);
}

void test_l1_reference(void){
  fprintf(stderr, "  LP backend: %s\n", vs_lp_backend_name());
  test_l1_reference_at(24,  L1_REFERENCE_OBJECTIVE_24,  1e-6);
  test_l1_reference_at(200, L1_REFERENCE_OBJECTIVE_200, 1e-3);
}

/** The library level entry point: relative transforms in, update transforms
    out, and a sane refusal for the cases it cannot handle. */
void test_l1_campath_transforms(TestData* testdata){
  const int N = 50;
  VSTransformConfig conf = vsTransformGetDefaultConfig("test_l1");
  conf.camPathAlgo = VSOptimalL1;
  /* The optimization has no parameters of its own: it reads the zoom budget
     off zoom/optZoom and the horizon off smoothing. */
  {
    VSTransformConfig c2 = conf;
    VSTransformData td2;
    VSL1Config a, b;
    /* automatic zoom without an amount -> the default budget */
    test_bool(vsTransformDataInit(&td2, &c2, &testdata->fi, &testdata->fi) == VS_OK);
    a = vsL1ConfigFromTransformConfig(&td2);
    test_bool(fabs(a.cropRatio - 1.0/(1.0 + VS_L1_DEFAULT_ZOOM/100.0)) < 1e-12);
    /* the default smoothing reproduces the weights of the paper */
    test_bool(fabs(a.w1 - 10.0) < 1e-12 && fabs(a.w2 - 1.0) < 1e-12
              && fabs(a.w3 - 100.0) < 1e-12);
    vsTransformDataCleanup(&td2);
    /* an explicit zoom is the budget, whether or not optZoom is on */
    c2.zoom = 30.0; c2.optZoom = 0; c2.smoothing = 30;
    test_bool(vsTransformDataInit(&td2, &c2, &testdata->fi, &testdata->fi) == VS_OK);
    b = vsL1ConfigFromTransformConfig(&td2);
    test_bool(fabs(b.cropRatio - 1.0/1.3) < 1e-12);
    /* twice the horizon shifts weight from |D(P)| onto |D^3(P)| */
    test_bool(b.w1 < a.w1 && b.w3 > a.w3);
    vsTransformDataCleanup(&td2);
    /* no automatic zoom and none asked for: no room, so it declines and the
       caller falls back to the gaussian filter */
    c2.zoom = 0.0; c2.optZoom = 0;
    test_bool(vsTransformDataInit(&td2, &c2, &testdata->fi, &testdata->fi) == VS_OK);
    test_bool(vsL1ConfigFromTransformConfig(&td2).cropRatio >= 1.0);
    vsTransformDataCleanup(&td2);
  }
  VSTransformData td;
  test_bool(vsTransformDataInit(&td, &conf, &testdata->fi, &testdata->fi) == VS_OK);

  VSTransformations trans;
  vsTransformationsInit(&trans);
  trans.ts = (VSTransform*)vs_malloc(sizeof(VSTransform) * N);
  trans.len = N;
  /* relative transforms of the synthetic path */
  VSTransformLS* F = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  campath_frame_pairs(F, N);
  for (int t = 0; t < N; t++) trans.ts[t] = transformLStoAZ(&F[t]);

  test_bool(vsPreprocessTransforms(&td, &trans) == VS_OK);

  /* The whole point of the inclusion constraints is that the stabilized frame
     has no undefined border pixels.  Check that end to end by running the
     corners of the destination frame through exactly the mapping the warping
     code uses (see transformPlanar in transformfloat.c):

        p_s = z R(-alpha) p_d - (x,y),   z = 1 - zoom/100

     and requiring that they land inside the source frame. */
  const double sx = td.fiSrc.width / 2.0,  sy = td.fiSrc.height / 2.0;
  const double dx = td.fiDest.width / 2.0, dy = td.fiDest.height / 2.0;
  double worst = -1e30;
  for (int t = 0; t < N; t++) {
    double z = 1.0 - trans.ts[t].zoom / 100.0;
    double zcos = z * cos(-trans.ts[t].alpha);
    double zsin = z * sin(-trans.ts[t].alpha);
    const double cx[4] = { -dx,  dx, dx, -dx };
    const double cy[4] = { -dy, -dy, dy,  dy };
    for (int i = 0; i < 4; i++) {
      double px =  zcos * cx[i] + zsin * cy[i] - trans.ts[t].x;
      double py = -zsin * cx[i] + zcos * cy[i] - trans.ts[t].y;
      worst = VS_MAX(worst, fabs(px) - sx);
      worst = VS_MAX(worst, fabs(py) - sy);
    }
    test_bool(fabs(trans.ts[t].alpha) < 0.2);
  }
  fprintf(stderr, "  worst border overshoot after warping: %.4f px\n", worst);
  test_bool(worst <= 1e-6);

  /* absolute transforms are not supported and must be rejected, not guessed */
  td.conf.relative = 0;
  test_bool(cameraPathOptimalL1(&td, &trans) == VS_ERROR);
  td.conf.relative = 1;
  /* too short for the third derivative */
  trans.len = 3;
  test_bool(cameraPathOptimalL1(&td, &trans) == VS_ERROR);
  trans.len = N;

  vs_free(F);
  vsTransformationsCleanup(&trans);
  vsTransformDataCleanup(&td);
}

/* ------------------------------------------- detection -> L1 integration */

/** Everything above exercises the L1 optimizer against purely synthetic,
    mathematically-defined transform sequences. This closes the remaining
    gap: run real vsMotionDetection on the synthetic circle frames from
    generate_synthetic.c (see tests/test_synthetic.c) to get a *detected*
    path -- not a known ground truth -- then feed that through the real
    library entry point (vsPreprocessTransforms with camPathAlgo ==
    VSOptimalL1) and check the same two properties test_l1_run() and
    test_l1_campath_transforms() check separately: the stabilized path is
    genuinely smoother, and the crop window never leaves the frame. */
static void test_l1_synthetic_detection_format(VSPixelFormat pf){
  const int N = SYN_L1_NUM_FRAMES;
  VSFrameInfo fi;
  VSFrame frames[SYN_L1_NUM_FRAMES];
  VSTransform* raw;
  VSTransformLS *F, *C, *B, *P;
  VSMotionDetectConfig mdconf;
  VSMotionDetect md;
  VSTransformConfig tdconf;
  VSTransformData td;
  VSTransformations trans;
  VSL1Config lsConf;
  double before2, after2, before3, after3;
  double sx, sy, dx, dy, worst;
  int i;

  fprintf(stderr, "--- detection -> L1 campath, %s ---\n", synFormatName(pf));

  generateL1ShakeFrames(frames, &fi, pf, SYN_WIDTH, SYN_HEIGHT, N);

  /* detect the raw path from the real synthetic frames, frame by frame,
     exactly as test_synthetic.c's checkRecoveredMotion() does -- except here
     the detected transform itself (not a known injected one) is what feeds
     the optimizer below. */
  mdconf = vsMotionDetectGetDefaultConfig("test_l1_syn_md");
  test_bool(vsMotionDetectInit(&md, &mdconf, &fi) == VS_OK);
  md.conf.numThreads = 1;

  raw = (VSTransform*)vs_malloc(sizeof(VSTransform) * N);
  for(i=0; i<N; i++){
    LocalMotions lms;
    test_bool(vsMotionDetection(&md, &lms, &frames[i]) == VS_OK);
    raw[i] = vsSimpleMotionsToTransform(fi, "test_l1_syn_md", &lms);
    vs_vector_del(&lms);
    vsFrameFree(&frames[i]);
  }
  vsMotionDetectionCleanup(&md);

  /* the real library entry point: vsPreprocessTransforms() dispatches to
     cameraPathOptimalL1() when camPathAlgo == VSOptimalL1, exactly as
     ffmpeg's vidstabtransform filter calls it */
  tdconf = vsTransformGetDefaultConfig("test_l1_syn_trans");
  tdconf.camPathAlgo = VSOptimalL1;
  test_bool(vsTransformDataInit(&td, &tdconf, &fi, &fi) == VS_OK);
  /* has to be taken before the run: cameraPathOptimalL1() marks the zoom
     budget as spent in td.conf once it has folded it into the transforms */
  lsConf = vsL1ConfigFromTransformConfig(&td);

  vsTransformationsInit(&trans);
  trans.ts = (VSTransform*)vs_malloc(sizeof(VSTransform) * N);
  trans.len = N;
  for(i=0; i<N; i++) trans.ts[i] = raw[i];

  test_bool(vsPreprocessTransforms(&td, &trans) == VS_OK);

  /* trans.ts is now the *render* transform updateToRenderTransform(B_t, ...)
     -- negated and folded together with the crop-window zoom, not B_t itself
     (see l1campathoptimization.c) -- which is exactly right for the
     border-inclusion check below, but not something to reverse-engineer B_t
     from. For the smoothness check we instead call the LS-level solver
     directly (vsCameraPathOptimalL1LS(), the same function
     cameraPathOptimalL1() calls internally, and the same one test_l1_run()
     uses) on the identical F[] built from the raw detected path, to get the
     true update transforms B_t and build the stabilized path P_t = C_t B_t. */
  F = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  C = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  B = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  P = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  F[0] = id_transformLS();
  C[0] = id_transformLS();
  for(i=1; i<N; i++){
    F[i] = transformAZtoLS(&raw[i]);
    C[i] = concat_transformLS(&C[i-1], &F[i]);
  }
  {
    double objective = -1.0;
    test_bool(vsCameraPathOptimalL1LS(F, N, B, &lsConf, &objective) == VS_OK);
  }
  for(i=0; i<N; i++)
    P[i] = concat_transformLS(&C[i], &B[i]);

  before2 = campath_diffnorm(C, N, 2);
  after2  = campath_diffnorm(P, N, 2);
  before3 = campath_diffnorm(C, N, 3);
  after3  = campath_diffnorm(P, N, 3);
  fprintf(stderr, "  |D2|: %8.2f -> %8.2f   |D3|: %8.2f -> %8.2f\n",
          before2, after2, before3, after3);
  test_bool(after2 < before2);
  test_bool(after3 < before3);

  /* crop-window inclusion, ported from test_l1_campath_transforms() */
  sx = td.fiSrc.width / 2.0;  sy = td.fiSrc.height / 2.0;
  dx = td.fiDest.width / 2.0; dy = td.fiDest.height / 2.0;
  worst = -1e30;
  for(i=0; i<N; i++){
    double z = 1.0 - trans.ts[i].zoom / 100.0;
    double zcos = z * cos(-trans.ts[i].alpha);
    double zsin = z * sin(-trans.ts[i].alpha);
    const double cx[4] = { -dx,  dx, dx, -dx };
    const double cy[4] = { -dy, -dy, dy,  dy };
    int k;
    for(k=0; k<4; k++){
      double px =  zcos * cx[k] + zsin * cy[k] - trans.ts[i].x;
      double py = -zsin * cx[k] + zcos * cy[k] - trans.ts[i].y;
      worst = VS_MAX(worst, fabs(px) - sx);
      worst = VS_MAX(worst, fabs(py) - sy);
    }
  }
  fprintf(stderr, "  worst border overshoot after warping: %.4f px\n", worst);
  test_bool(worst <= 1e-6);

  vs_free(F); vs_free(C); vs_free(B); vs_free(P);
  vs_free(raw);
  vsTransformationsCleanup(&trans);
  vsTransformDataCleanup(&td);
}

void test_l1_synthetic_detection(void){
  test_l1_synthetic_detection_format(PF_YUV420P);
  test_l1_synthetic_detection_format(PF_RGBA);
}
