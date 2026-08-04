#define NUMCNTR 2000

void test_contrastImg(const TestData* testdata){
  int i;
  Field f;
  // difference between michelson and absolute differences from mean
  //  is large for 100x100 at 500,300
  f.size=128;
  f.x = 400;
  f.y = 300;
  fprintf(stderr,"********** Contrast:\n");
  int numruns = NUMCNTR;
  /* Sized from the macro rather than from numruns: the latter makes these
     variable length arrays, which MSVC does not implement. */
  double contrastC[NUMCNTR];
  double contrastOpt[NUMCNTR];
  int timeC, timeOpt;
  fprintf(stderr,"********** Michelson Contrast (with SSE2):\n");
  {
    int start = timeOfDayinMS();
    for(i=0; i<numruns; i++){
      contrastC[i]=contrastSubImg(testdata->frames[0].data[0],
                                  &f, testdata->fi.width, testdata->fi.height,1);
    }
    int end = timeOfDayinMS();
    timeC=end-start;
    fprintf(stderr,"***C    time for %i runs: %i ms ****\n", numruns, timeC);
  }
#ifdef USE_SSE2
  {
    int start = timeOfDayinMS();
    for(i=0; i<numruns; i++){
      contrastOpt[i]=contrastSubImg1_SSE(testdata->frames[0].data[0],
                                         &f, testdata->fi.width, testdata->fi.height);
    }
    int end = timeOfDayinMS();
    timeOpt=end-start;
    fprintf(stderr,"***SSE2 time for %i runs: %i ms ****\n", numruns, timeOpt);
  }
  fprintf(stderr,"***Speedup %3.2f\n", (float)timeC/(float)timeOpt);
  for(i=0; i<numruns; i++){
    if(i==0){
      printf("SSE2 contrast %3.2f, C contrast %3.2f\n",contrastOpt[i], contrastC[i]);
    }
    test_bool(contrastC[i]==contrastOpt[i]);
  }

  /* Sweep several field sizes and positions, not just one.
     Sizes must be multiples of 16: contrastSubImg1_SSE loads 16 bytes per
     inner iteration ("k += 16"), any other size reads past the field.
     vsMotionDetectInit() guarantees that in production
     (see src/motiondetect.c: "fieldSize = (fieldSize / 16 + 1) * 16").
     Exact equality is required, not an epsilon: both functions reduce the
     field to an 8 bit min and max and then evaluate the identical expression
     (maxi-mini)/(maxi+mini+0.1); with identical integer inputs and no
     reassociation of float ops the doubles are bit-identical. */
  {
    static const int sizes[] = {16, 32, 48, 64, 80};
    static const int pos[][2] = { {400,300}, {200,200}, {900,500}, {640,360} };
    int s, p, fr;
    fprintf(stderr,"********** Contrast C vs SSE2 over sizes/positions:\n");
    for(fr=0; fr<2; fr++){
      for(s=0; s<(int)(sizeof(sizes)/sizeof(sizes[0])); s++){
        for(p=0; p<(int)(sizeof(pos)/sizeof(pos[0])); p++){
          Field ff;
          ff.size = sizes[s];
          ff.x = pos[p][0];
          ff.y = pos[p][1];
          double cC = contrastSubImg(testdata->frames[fr].data[0], &ff,
                                     testdata->frames[fr].linesize[0],
                                     testdata->fi.height, 1);
          double cO = contrastSubImg1_SSE(testdata->frames[fr].data[0], &ff,
                                          testdata->frames[fr].linesize[0],
                                          testdata->fi.height);
          if(cC != cO)
            fprintf(stderr,"  CONTRAST MISMATCH frame=%i size=%i pos=(%i,%i): "
                    "C=%.17g SSE2=%.17g\n", fr, ff.size, ff.x, ff.y, cC, cO);
          test_bool(cC == cO);
        }
      }
    }
  }
#endif
}
