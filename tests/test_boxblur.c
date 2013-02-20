// runs the boxblur routine and returns the time
int runboxblur( VSFrame frame1, VSFrame dest,
								VSFrameInfo fi, int numruns){
  int start = timeOfDayinMS();
  int i;
  boxblurYUV(&dest, &frame1, 0, &fi, 15, BoxBlurColor);
  for(i=1; i<numruns; i++){
    boxblurYUV(&dest, &dest, 0, &fi, 15, BoxBlurColor);
  }
  int end = timeOfDayinMS();
  return end-start;
}


void test_boxblur(const TestData* testdata){
	int time; //, timeref;
	int numruns=2;
	VSFrame dest;
	allocateFrame(&dest,&testdata->fi);
	//    omp_set_dynamic( 0 );
	//    omp_set_num_threads( 1 );
	fprintf(stderr,"********** boxblur speedtest:\n");
	time = runboxblur(testdata->frames[4], dest, testdata->fi, numruns);
	fprintf(stderr,"***C    time for %i runs: %i ms\n", numruns, time);
	storePGMImage("boxblured.pgm", dest.data[0], testdata->fi);
	storePGMImage("orig4.pgm", testdata->frames[4].data[0], testdata->fi);
	// timeref=time;
	/* omp_set_dynamic( 0 ); */
	/* omp_set_num_threads( 2); */
	/* time = runboxblur(testdata->frames[4], dest, testdata->fi, numruns); */
	/* fprintf(stderr,"***C (2)time for %i runs: %i ms, Speedup %f\n", numruns, time, */
	/* 	    (double)timeref/time); */
	/* omp_set_dynamic( 1 ); */
	freeFrame(&dest);
}
