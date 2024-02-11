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
  double contrastC[numruns];
  double contrastOpt[numruns];
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
#endif
}
