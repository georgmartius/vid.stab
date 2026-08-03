static const VSPixelFormat SYN_FORMATS[6] = {
  PF_GRAY8, PF_YUV420P, PF_YUV422P, PF_YUV444P, PF_RGB24, PF_RGBA
};
#define SYN_NUM_FORMATS 6

static const char* synFormatName(VSPixelFormat pf){
  switch(pf){
   case PF_GRAY8:    return "PF_GRAY8";
   case PF_YUV420P:  return "PF_YUV420P";
   case PF_YUV422P:  return "PF_YUV422P";
   case PF_YUV444P:  return "PF_YUV444P";
   case PF_RGB24:    return "PF_RGB24";
   case PF_RGBA:     return "PF_RGBA";
   default:          return "?";
  }
}

static void test_synthetic_pixelhelpers(void){
  int fmt;
  fprintf(stderr, "--- synthetic pixel helper roundtrip ---\n");
  for(fmt=0; fmt<SYN_NUM_FORMATS; fmt++){
    VSFrameInfo fi;
    VSFrame frame;
    uint8_t r,g,b;
    int isGray = (SYN_FORMATS[fmt] == PF_GRAY8);

    test_bool(vsFrameInfoInit(&fi, 16, 16, SYN_FORMATS[fmt]) != 0);
    vsFrameAllocate(&frame, &fi);
    test_bool(!vsFrameIsNull(&frame));

    fillFrameRGB(&frame, &fi, 90, 90, 90);
    setPixelRGB(&frame, &fi, 4, 4, 230, 60, 40);

    getPixelRGB(&frame, &fi, 4, 4, &r, &g, &b);
    fprintf(stderr, "%s: wrote (230,60,40) read (%i,%i,%i)\n",
            synFormatName(SYN_FORMATS[fmt]), r, g, b);
    if(!isGray){
      test_bool(abs((int)r-230) < 30);
      test_bool(abs((int)g-60)  < 30);
      test_bool(abs((int)b-40)  < 30);
    }

    getPixelRGB(&frame, &fi, 0, 0, &r, &g, &b);
    test_bool(abs((int)r-90) < 10 && abs((int)g-90) < 10 && abs((int)b-90) < 10);

    vsFrameFree(&frame);
  }
}

/* Runs vsMotionDetection frame-by-frame (same one-md-instance-per-sequence
   convention as test_motionDetect() in test_motiondetect.c: call once per
   frame starting at frame 0, which establishes the internal reference with
   an expected ~zero motion) and asserts the recovered transform matches
   getTestFrameTransform(i) within tolerance. */
static void checkRecoveredMotion(const VSFrameInfo* fi, VSFrame* frames, int numFrames,
                                 const char* label, double tolXY, double tolAlpha){
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig(label);
  VSMotionDetect md;
  int i;

  test_bool(vsMotionDetectInit(&md, &mdconf, fi) == VS_OK);
  md.conf.numThreads = 1;

  for(i=0; i<numFrames; i++){
    LocalMotions lms;
    VSTransform t, orig, diff;
    int success;

    test_bool(vsMotionDetection(&md, &lms, &frames[i]) == VS_OK);
    t = vsSimpleMotionsToTransform(*fi, label, &lms);
    vs_vector_del(&lms);

    orig = mult_transform_(getTestFrameTransform(i), -1.0);
    diff = sub_transforms(&t, &orig);
    success = fabs(diff.x)<tolXY && fabs(diff.y)<tolXY && fabs(diff.alpha)<tolAlpha;

    fprintf(stderr, "%s frame %i: ", label, i);
    storeVSTransform(stderr, &t);
    if(!success){
      fprintf(stderr, "  Difference: ");
      storeVSTransform(stderr, &diff);
    }
    test_bool(success);
  }
  vsMotionDetectionCleanup(&md);
}

void test_synthetic_circles(void){
  int fmt;
  test_synthetic_pixelhelpers();

  fprintf(stderr, "--- synthetic circle sequence content ---\n");
  for(fmt=0; fmt<SYN_NUM_FORMATS; fmt++){
    VSFrameInfo fi;
    VSFrame frames[SYN_NUM_FRAMES];
    uint8_t r,g,b;
    int i;
    int isGray = (SYN_FORMATS[fmt] == PF_GRAY8);

    generateCircleFrames(frames, &fi, SYN_FORMATS[fmt], SYN_WIDTH, SYN_HEIGHT, SYN_NUM_FRAMES);

    /* frame 0: background color away from any circle, circle color at a circle center */
    getPixelRGB(&frames[0], &fi, 5, 5, &r, &g, &b);
    test_bool(abs((int)r-SYN_BG_R)<20 && abs((int)g-SYN_BG_G)<20 && abs((int)b-SYN_BG_B)<20);
    if(!isGray){
      getPixelRGB(&frames[0], &fi, SYN_CIRCLES[2].cx, SYN_CIRCLES[2].cy, &r, &g, &b);
      test_bool(abs((int)r-SYN_CIRCLE_R)<30 && abs((int)g-SYN_CIRCLE_G)<30 && abs((int)b-SYN_CIRCLE_B)<30);
    }

    fprintf(stderr, "%s: frame 0 background/circle colors OK\n", synFormatName(SYN_FORMATS[fmt]));

    checkRecoveredMotion(&fi, frames, SYN_NUM_FRAMES, synFormatName(SYN_FORMATS[fmt]), 2.0, 0.005);

    for(i=0; i<SYN_NUM_FRAMES; i++)
      vsFrameFree(&frames[i]);
  }
}
