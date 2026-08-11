/* Virtual tripod mode regression tests.

   Included as a translation unit by tests.c (after generate_synthetic.c and
   test_synthetic.c), so it needs no includes of its own.

   See GitHub issues #85 / #27. */

/* Tripod mode on the transform side is defined as relative=0:smoothing=0 (see
   transform.h:171). optZoom and zoom are switched off too: optZoom defaults to
   1, and that path ends up adding a zoom term to every transform (see the
   `else if (td->conf.zoom != 0)` branch in vsPreprocessTransforms), which would
   make a pass-through assertion fail for reasons that have nothing to do with
   the camera-path logic under test. maxShift/maxAngle are already -1 (no
   clamping) by default; they are set explicitly to document the intent. */
static VSTransformConfig tripodTransformConfig(const char* name, VSCamPathAlgo algo){
  VSTransformConfig c = vsTransformGetDefaultConfig(name);
  c.relative    = 0;   /* tripod */
  c.smoothing   = 0;   /* tripod */
  c.optZoom     = 0;
  c.zoom        = 0;
  c.maxShift    = -1;
  c.maxAngle    = -1;
  c.camPathAlgo = algo;
  return c;
}

/* Runs the real library entry point vsPreprocessTransforms() over `in` and
   copies the result to `out`. This is the same call ffmpeg's vidstabtransform
   filter makes. */
static void runTripodPreprocess(VSTransformConfig* conf, const VSFrameInfo* fi,
                                const VSTransform* in, VSTransform* out, int n){
  VSTransformData td;
  VSTransformations trans;
  int i;

  test_bool(vsTransformDataInit(&td, conf, fi, fi) == VS_OK);
  vsTransformationsInit(&trans);
  trans.ts  = (VSTransform*)vs_malloc(sizeof(VSTransform) * n);
  trans.len = n;
  for(i=0; i<n; i++) trans.ts[i] = in[i];

  test_bool(vsPreprocessTransforms(&td, &trans) == VS_OK);
  for(i=0; i<n; i++) out[i] = trans.ts[i];

  vsTransformationsCleanup(&trans);
  vsTransformDataCleanup(&td);
}

/* In tripod mode vsPreprocessTransforms() must be a pure pass-through: no
   relative-to-absolute integration, no smoothing, no camera-path optimization.
   Each stored transform is already the absolute correction for its frame. */
static void checkTripodPassThrough(VSCamPathAlgo algo, const char* algoName){
  const int N = SYN_TRIPOD_NUM_FRAMES;
  VSFrameInfo fi;
  VSTransform in[SYN_TRIPOD_NUM_FRAMES], out[SYN_TRIPOD_NUM_FRAMES];
  VSTransformConfig conf;
  int i;

  fprintf(stderr, "--- tripod transform pass-through, camPathAlgo=%s ---\n", algoName);
  test_bool(vsFrameInfoInit(&fi, SYN_WIDTH, SYN_HEIGHT, PF_YUV420P) != 0);

  /* the stored convention is the correction, i.e. the negated camera motion --
     see test_synthetic.c:73, which compares detection output against
     -getTestFrameTransform(i) for the same reason. */
  for(i=0; i<N; i++) in[i] = mult_transform_(synTripodTransform(i), -1.0);

  conf = tripodTransformConfig("test_tripod_trans", algo);
  runTripodPreprocess(&conf, &fi, in, out, N);

  for(i=0; i<N; i++){
    VSTransform d = sub_transforms(&out[i], &in[i]);
    int ok = fabs(d.x)<1e-9 && fabs(d.y)<1e-9
          && fabs(d.alpha)<1e-9 && fabs(d.zoom)<1e-9;
    if(!ok){
      fprintf(stderr, "frame %i  in: ", i); storeVSTransform(stderr, &in[i]);
      fprintf(stderr, "         out: ");    storeVSTransform(stderr, &out[i]);
    }
    test_bool(ok);
  }
}

/* Negative control. With the library default relative=1 the very same input
   must come out *different* (integrated into an absolute path). Without this,
   a regression that silently ignored `relative` would leave the pass-through
   assertion above trivially satisfied and green. VSGaussian with smoothing=0
   is used so that the integration step is the only thing that differs. */
static void checkTripodRelativeControl(void){
  const int N = SYN_TRIPOD_NUM_FRAMES;
  VSFrameInfo fi;
  VSTransform in[SYN_TRIPOD_NUM_FRAMES], out[SYN_TRIPOD_NUM_FRAMES];
  VSTransformConfig conf;
  int i, differs = 0;

  fprintf(stderr, "--- tripod negative control: relative=1 must differ ---\n");
  test_bool(vsFrameInfoInit(&fi, SYN_WIDTH, SYN_HEIGHT, PF_YUV420P) != 0);
  for(i=0; i<N; i++) in[i] = mult_transform_(synTripodTransform(i), -1.0);

  conf = tripodTransformConfig("test_tripod_relctl", VSGaussian);
  conf.relative = 1;
  runTripodPreprocess(&conf, &fi, in, out, N);

  for(i=0; i<N; i++){
    VSTransform d = sub_transforms(&out[i], &in[i]);
    if(fabs(d.x) > 1.0 || fabs(d.y) > 1.0) differs = 1;
  }
  test_bool(differs);
}

void test_tripod_transforms(void){
  checkTripodPassThrough(VSOptimalL1, "VSOptimalL1");
  checkTripodPassThrough(VSGaussian,  "VSGaussian");
  checkTripodPassThrough(VSAvg,       "VSAvg");
  checkTripodRelativeControl();
}

/* Decided tripod semantics (issue #85): frames before the reference frame
   (i <= reference-1, i.e. the lead-in including the reference frame itself)
   are not stabilized and emit a zero transform. From the reference frame on,
   every emitted motion is the correction that maps frame i back onto the
   reference frame, i.e. -(A_i - A_ref) = A_ref - A_i. Composing the two warps
   is approximated here by component-wise subtraction, exactly as the library
   itself does throughout (add_transforms / sub_transforms); at these
   amplitudes (<=23.9px, <=1.1deg) the neglected cross term is about 0.3px,
   far inside tolXY. */
static VSTransform tripodExpected(int i, int reference){
  if(i <= reference - 1) return null_transform();
  VSTransform ai = synTripodTransform(i);
  VSTransform ar = synTripodTransform(reference - 1);
  return sub_transforms(&ar, &ai);
}

/* Generates the tripod sequence and runs vsMotionDetection() over it frame by
   frame with virtualTripod set, collecting the raw per-frame transform. Same
   one-md-instance-per-sequence convention as checkRecoveredMotion() in
   test_synthetic.c. Frames are freed as they are consumed. */
static void detectTripodPath(VSPixelFormat pf, int reference,
                             VSTransform* raw, int n){
  VSFrameInfo fi;
  VSFrame frames[SYN_TRIPOD_NUM_FRAMES];
  VSMotionDetectConfig mdconf;
  VSMotionDetect md;
  int i;

  /* frames[] is fixed at SYN_TRIPOD_NUM_FRAMES; guard against a caller
     passing a larger n, which would overflow it. All current callers pass
     SYN_TRIPOD_NUM_FRAMES, so this is latent only. */
  test_bool(n <= SYN_TRIPOD_NUM_FRAMES);

  generateTripodFrames(frames, &fi, pf, SYN_WIDTH, SYN_HEIGHT, n);

  mdconf = vsMotionDetectGetDefaultConfig("test_tripod_md");
  mdconf.virtualTripod = reference;
  /* vsMotionDetectInit() reads md.serializationMode before writing it
     (src/motiondetect.c:111), which valgrind flags as a use of an
     uninitialised value; zero the struct first to keep this test
     valgrind-clean. The underlying read-before-write is a library-side
     issue, out of scope here. */
  memset(&md, 0, sizeof(md));
  test_bool(vsMotionDetectInit(&md, &mdconf, &fi) == VS_OK);
  md.conf.numThreads = 1;

  for(i=0; i<n; i++){
    LocalMotions lms;
    test_bool(vsMotionDetection(&md, &lms, &frames[i]) == VS_OK);
    raw[i] = vsSimpleMotionsToTransform(fi, "test_tripod_md", &lms);
    vs_vector_del(&lms);
    vsFrameFree(&frames[i]);
  }
  vsMotionDetectionCleanup(&md);
}

/* tolXY/tolAlpha are the values established empirically in test_synthetic.c
   (see the long comment at test_synthetic.c:120): the headroom on tolXY covers
   genuine C-vs-SSE2 block-matching divergence (CMakeLists.txt SSE2_FOUND /
   CMakeModules/FindSSE.cmake), not slop in the test. This scene warps from
   frame 0 rather than chaining, so it accumulates less error than the sequence
   those numbers were measured on and they are, if anything, generous here. */
#define TRIPOD_TOL_XY    4.0
#define TRIPOD_TOL_ALPHA 0.005

/* Asserts the recovered path is absolute with respect to the reference frame
   for EVERY frame. Returns the number of frames that failed (0 = all good) so
   callers can report the failure pattern, which is what localizes the bug. */
static int checkTripodDetection(VSPixelFormat pf, int reference){
  const int N = SYN_TRIPOD_NUM_FRAMES;
  VSTransform raw[SYN_TRIPOD_NUM_FRAMES];
  int i, bad = 0;

  fprintf(stderr, "--- tripod detection, %s, virtualTripod=%i ---\n",
          synFormatName(pf), reference);
  detectTripodPath(pf, reference, raw, N);

  for(i=0; i<N; i++){
    VSTransform exp = tripodExpected(i, reference);
    VSTransform d   = sub_transforms(&raw[i], &exp);
    int ok = fabs(d.x)<TRIPOD_TOL_XY && fabs(d.y)<TRIPOD_TOL_XY
          && fabs(d.alpha)<TRIPOD_TOL_ALPHA;
    /* Strengthen the lead-in check: frames up to and including the reference
       frame itself must be *exactly* zero (not merely within tolerance), so a
       regression that emitted small-but-nonzero motions there is caught. */
    if(i <= reference - 1){
      int exact = raw[i].x==0.0 && raw[i].y==0.0 && raw[i].alpha==0.0
               && raw[i].zoom==0.0;
      if(!exact) ok = 0;
    }
    /* Complementary check: the first stabilized frame (the reference index
       itself) must NOT be exactly zero. If the reference index silently
       drifts forward by one, this frame would report zero instead of the
       expected nonzero correction, and the tolerance-based check alone would
       not reliably catch it (see the module comment above). */
    if(i == reference){
      int allZero = raw[i].x==0.0 && raw[i].y==0.0 && raw[i].alpha==0.0;
      if(allZero) ok = 0;
    }
    fprintf(stderr, "frame %2i %s got: ", i, ok ? "ok  " : "FAIL");
    storeVSTransform(stderr, &raw[i]);
    if(!ok){
      fprintf(stderr, "            want: "); storeVSTransform(stderr, &exp);
      if(i == reference){
        fprintf(stderr, "            (expected nonzero: this is the first stabilized frame)\n");
      }
      bad++;
    }
  }
  return bad;
}

void test_tripod_detection(void){
  /* virtualTripod=1 is the self-consistent case: the reference is frame 1 and
     frame 0 emits a dummy motion, so every frame really is measured against
     the reference. This is why the default tripod setting mostly works and the
     bug looked intermittent. */
  test_bool(checkTripodDetection(PF_YUV420P, 1) == 0);
  test_bool(checkTripodDetection(PF_RGBA,    1) == 0);

  /* Fixed behavior for issue #85. Detection is a single streaming pass, so
     when frames 0..R-1 are processed the reference frame (at index R-1) has
     not been read yet and they *cannot* be measured against it. Rather than
     measuring them against their predecessor -- which produced a .trf file
     mixing relative and absolute conventions that the transform stage (which
     reads everything as absolute, relative=0) misinterpreted -- frames
     0..R-1 now emit a zero transform: no stabilization is applied until the
     reference frame is reached. Frames R.. are measured against the
     reference at index R-1, exactly as before. Expect frames 0,1,2 (and the
     reference frame itself at index R-1=3) to report exactly zero, and
     frames 4.. to match the reference-relative expectation. */
  test_bool(checkTripodDetection(PF_YUV420P, SYN_TRIPOD_REF) == 0);
  test_bool(checkTripodDetection(PF_RGBA,    SYN_TRIPOD_REF) == 0);
}
