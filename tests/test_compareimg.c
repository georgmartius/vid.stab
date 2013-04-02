#define NUMCMP 2000

int checkCompareImg(MotionDetect* md, const VSFrame* frame){
  int i;
  int error;
  uint8_t *Y_c;
  Field field;
  field.x=400;
  field.y=400;
  field.size=12;

  Y_c = frame->data[0];
	int linesize = frame->linesize[0];

  for(i=-10;i<10; i+=2){
    printf("\nCheck: shiftX = %i\n",i);
    error = compareSubImg(Y_c, Y_c, &field,
													linesize, linesize, md->fi.height,
													1, i, 0, INT_MAX);
    fprintf(stderr,"mismatch %i: %i\n", i, error);
  }
  return 1;
}

void test_checkCompareImg(const TestData* testdata){
	MotionDetect md;

  test_bool(initMotionDetect(&md, &testdata->fi, "test") == VS_OK);
  md.shakiness=6;
  md.accuracy=12;
  fflush(stdout);
	test_bool(configureMotionDetect(&md)== VS_OK);
	test_bool(checkCompareImg(&md,&testdata->frames[0]));
	cleanupMotionDetection(&md);
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
  if(diffsRef)
    for(i=0; i<numruns; i++){
      if(diffs[i]!=diffsRef[i]){
				fprintf(stderr, "ERROR! Ref difference %i, Opt difference %i\n",
								diffsRef[i], diffs[i]);
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
#ifdef USE_ORC
	timeO=runcompare(compareSubImg_orc, testdata->frames[0], testdata->frames[1],
									 f, testdata->fi, diffsO, diffsC, numruns);
	fprintf(stderr,"***orc      time for %i runs: %i ms \tSpeedup %3.2f\n",
					numruns, timeO, (double)timeC/timeO);
	timeO=runcompare(compareSubImg_thr_orc, testdata->frames[0], testdata->frames[1],
									 f, testdata->fi, diffsO, diffsC, numruns);
	fprintf(stderr,"***thr_orc  time for %i runs: %i ms \tSpeedup %3.2f\n",
					numruns, timeO, (double)timeC/timeO);
#endif
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
