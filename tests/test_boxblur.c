// runs the boxblur routine and returns the time
int runboxblur( unsigned char* frame1, unsigned char* dest,
								DSFrameInfo fi, int numruns){
  int start = timeOfDayinMS();
  int i;
  boxblurYUV(dest, frame1, 0, &fi, 15, BoxBlurColor);
  for(i=1; i<numruns; i++){
    boxblurYUV(dest, dest, 0, &fi, 15, BoxBlurColor);
  }
  int end = timeOfDayinMS();
  return end-start;
}


void test_boxblur(const TestData* testdata){
	int time; //, timeref;
	int numruns=2;
	unsigned char* dest = (unsigned char*)malloc(testdata->fi.framesize);
	//    omp_set_dynamic( 0 );
	//    omp_set_num_threads( 1 );
	fprintf(stderr,"********** boxblur speedtest:\n");
	time = runboxblur(testdata->frames[4], dest, testdata->fi, numruns);
	fprintf(stderr,"***C    time for %i runs: %i ms\n", numruns, time);
	storePGMImage("boxblured.pgm", dest, testdata->fi);
	storePGMImage("orig4.pgm", testdata->frames[4], testdata->fi);
	// timeref=time;
	/* omp_set_dynamic( 0 ); */
	/* omp_set_num_threads( 2); */
	/* time = runboxblur(testdata->frames[4], dest, testdata->fi, numruns); */
	/* fprintf(stderr,"***C (2)time for %i runs: %i ms, Speedup %f\n", numruns, time, */
	/* 	    (double)timeref/time); */
	/* omp_set_dynamic( 1 ); */
	free(dest);
}
