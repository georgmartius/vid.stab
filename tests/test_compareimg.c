#define NUMCMP 2000

/* Compares the (possibly SIMD) compareSubImg against the plain C reference
   compareSubImg_thr and asserts that they agree exactly.
   field.size MUST be a multiple of 16: the SSE2 implementation advances 16
   bytes per inner iteration ("k += 16"), so any other size makes it read
   beyond the field and the results legitimately differ. Production code never
   does that because vsMotionDetectInit() rounds fieldSize up to a multiple of
   16 when SSE2 is enabled (see src/motiondetect.c:
   "fieldSize = (fieldSize / 16 + 1) * 16").
   The threshold is INT_MAX here, so neither implementation can exit early and
   strict equality is the correct assertion. */
int checkCompareImg(VSMotionDetect* md, const VSFrame* frame){
  int i;
  unsigned int error, errorRef;
  uint8_t *Y_c;
  Field field;
  field.x=400;
  field.y=400;
  field.size=64; /* multiple of 16, see comment above */

  Y_c = frame->data[0];
  int linesize = frame->linesize[0];

  for(i=-10;i<10; i+=2){
    printf("\nCheck: shiftX = %i\n",i);
    error = compareSubImg(Y_c, Y_c, &field,
                          linesize, linesize, md->fi.height,
                          1, i, 0, INT_MAX);
    errorRef = compareSubImg_thr(Y_c, Y_c, &field,
                                 linesize, linesize, md->fi.height,
                                 1, i, 0, INT_MAX);
    fprintf(stderr,"mismatch %i: opt %u, C %u\n", i, error, errorRef);
    test_bool(error == errorRef);
  }
  /* a shift of 0 against the identical image must give a difference of 0 */
  test_bool(compareSubImg(Y_c, Y_c, &field, linesize, linesize, md->fi.height,
                          1, 0, 0, INT_MAX) == 0);
  return 1;
}

void test_checkCompareImg(const TestData* testdata){
  VSMotionDetect md;
  VSMotionDetectConfig conf = vsMotionDetectGetDefaultConfig("test_checkCompareImg");
  conf.shakiness=6;
  conf.accuracy=12;
  test_bool(vsMotionDetectInit(&md, &conf, &testdata->fi) == VS_OK);
  fflush(stdout);
  test_bool(checkCompareImg(&md,&testdata->frames[0]));
  vsMotionDetectionCleanup(&md);
}


typedef unsigned int (*cmpSubImgFunc)(unsigned char* const I1, unsigned char* const I2,
                                      const Field* field,
                                      int width1, int width2, int height, int bytesPerPixel,
                                      int d_x, int d_y, unsigned int threshold);

// runs the compareSubImg routine and returns the time and stores the difference.
//  if diffsRef is given than the results are validated
int runcompare( cmpSubImgFunc cmpsubfunc,
                VSFrame frame1, VSFrame frame2, Field f,
                VSFrameInfo fi, int* diffs, int* diffsRef, int numruns){
  int start = timeOfDayinMS();
  int i;
  for(i=0; i<numruns; i++){
    diffs[i]=cmpsubfunc(frame1.data[0], frame2.data[0],
                        &f, frame1.linesize[0], frame2.linesize[0], fi.height,
                        2, i%200, i/200, INT_MAX);
  }
  int end = timeOfDayinMS();
  if(diffsRef){
    int reported = 0;
    for(i=0; i<numruns; i++){
      if(diffs[i]!=diffsRef[i] && reported++ < 10){
        fprintf(stderr, "ERROR! run %i: Ref difference %i, Opt difference %i\n",
                i, diffsRef[i], diffs[i]);
      }
      /* threshold is INT_MAX above, so no early exit: must be exactly equal */
      test_bool(diffs[i]==diffsRef[i]);
    }
  }
  return end-start;
}



void test_compareImg_performance(const TestData* testdata){
  Field f;
  f.size=128;
  f.x = 400;
  f.y = 300;
  fprintf(stderr,"********** Compare speedtest:\n");

  int numruns = NUMCMP;
  int diffsC[numruns];
  int diffsO[numruns];
  int timeC, timeO;
  timeC=runcompare(compareSubImg_thr, testdata->frames[0], testdata->frames[1],
                   f, testdata->fi, diffsC, 0, numruns);
  fprintf(stderr,"***C        time for %i runs: %i ms ****\n", numruns, timeC);
#ifdef USE_SSE2
  timeO=runcompare(compareSubImg_thr_sse2, testdata->frames[0], testdata->frames[1],
                   f, testdata->fi, diffsO, diffsC, numruns);
  fprintf(stderr,"***thr_sse2 time for %i runs: %i ms \tSpeedup %3.2f\n",
          numruns, timeO, (double)timeC/timeO);
#endif
#ifdef USE_SSE2_ASM
  timeO=runcompare(compareSubImg_thr_sse2_asm, testdata->frames[0], testdata->frames[1],
                   f, testdata->fi, diffsO, diffsC, numruns);
  fprintf(stderr,"***thr_asm  time for %i runs: %i ms \tSpeedup %3.2f\n",
          numruns, timeO, (double)timeC/timeO);
#endif
}
