
void testImageStripeYUV(int size, DSFrameInfo* fi, unsigned char** img){
  int i,j;
  fi->width=size;
  fi->height=4;
  fi->strive=size;
  fi->framesize=3*(size*fi->height)/2;
  fi->pFormat = PF_YUV;
  *img = (unsigned char*)malloc(sizeof(char)*fi->framesize);
  memset(*img,100,sizeof(char)*fi->framesize);
  for(j=0; j<fi->height; j++){
    for(i=0; i<size; i++){
      (*img)[i+j*size]= sin(((double)i)/size/(double)j)*128+128;
    }
  }
  for(j=0; j<fi->height/2; j++){
    for(i=0; i<size/2; i++){
      (*img)[i+fi->height*size+j*size/2]= sin(((double)i)/size/j*2.0)*128+128;
    }
  }
}


void test_transform_implementation(const TestData* testdata){

  DSFrameInfo fi;
  unsigned char* src;
  testImageStripeYUV(128,&fi,&src);
  unsigned char* dest= (unsigned char*)malloc(fi.framesize);
  unsigned char* cfinal = (unsigned char*)malloc(fi.framesize);
  TransformData td;
  fprintf(stderr,"--- Validate Interpolations ----\n");

  int it;
  int i;
  int sum;
  Transform t;
  t.x = 10;
  t.alpha = 2*M_PI/(180.0);

  for(it=Zero; it<=BiCubic; it++){
    memcpy(dest, src, fi.framesize);
    test_bool(initTransformData(&td, &fi, &fi, "test") == DS_OK);
    td.interpolType=it;
    test_bool(configureTransformData(&td)== DS_OK);

    fprintf(stderr,"Transform: %s\n", interpolTypes[it]);
    test_bool(transformPrepare(&td,dest,dest)== DS_OK);
    test_bool(transformYUV_float(&td, t)== DS_OK);

    memcpy(cfinal,td.dest,fi.framesize);

    memcpy(dest, src, fi.framesize);
    test_bool(initTransformData(&td, &fi, &fi, "test") == DS_OK);
    td.interpolType=it;
    test_bool(configureTransformData(&td)== DS_OK);
    test_bool(transformPrepare(&td,dest,dest)== DS_OK);
    test_bool(transformYUV(&td, t)== DS_OK);

    // validate
    sum=0;
    for(i=0; i<fi.framesize; i++){
      if(abs(cfinal[i] - td.dest[i])>2){
				sum+=abs(cfinal[i] - td.dest[i]);
				printf("%i,%i: %i\n", i/fi.width, i%fi.width ,cfinal[i] - td.dest[i]);
      }
    }
		cleanupTransformData(&td);
    printf("***Difference: %i\n", sum);
		test_bool(sum==0);
  }
	free(dest);
	free(cfinal);
	free(src);
}

void test_transform_performance(const TestData* testdata){


	fprintf(stderr,"--- Performance of Transforms ----\n");
	unsigned char* dest = (unsigned char*)malloc(testdata->fi.framesize);
	unsigned char* cfinal = (unsigned char*)malloc(testdata->fi.framesize);
	int it;
	int start, numruns;
	int timeC, timeCFP; //, timeOrc;
	numruns = 5;
	for(it=Zero; it<=BiCubic; it++){
		TransformData td;
		int i;
		//// Float implementation
		test_bool(initTransformData(&td, &testdata->fi, &testdata->fi, "test") == DS_OK);
		td.interpolType=it;
		test_bool(configureTransformData(&td)== DS_OK);

		fprintf(stderr,"Transform: %s", interpolTypes[it]);
		start = timeOfDayinMS();
		for(i=0; i<numruns; i++){
			Transform t = null_transform();
			t.x = i*10+10;
			t.alpha = (i+1)*2*M_PI/(180.0);
			t.zoom = 0;
			memcpy(dest, testdata->frames[0], testdata->fi.framesize);
			test_bool(transformPrepare(&td,dest,dest)== DS_OK);
			test_bool(transformYUV_float(&td, t)== DS_OK);
		}
		timeC = timeOfDayinMS() - start;
		fprintf(stderr,"\n***C   elapsed time for %i runs: %i ms ****\n",
						numruns, timeC );

		if(it==BiLinear){
			storePGMImage("transformed.pgm", td.dest, testdata->fi);
			storePGMImage("transformed_u.pgm",
										td.dest+testdata->fi.width*testdata->fi.height, testdata->fi_color);
			fprintf(stderr,"stored transformed.pgm\n");
		}
		memcpy(cfinal,td.dest,testdata->fi.framesize);
		cleanupTransformData(&td);

		//// fixed point implementation
		test_bool(initTransformData(&td, &testdata->fi, &testdata->fi, "test") == DS_OK);
		td.interpolType=it;
		test_bool(configureTransformData(&td)== DS_OK);
		start = timeOfDayinMS();
		for(i=0; i<numruns; i++){
			Transform t = null_transform();
			t.x = i*10+10;
			t.alpha = (i+1)*2*M_PI/(180.0);
			t.zoom = 0;
			memcpy(dest, testdata->frames[0], testdata->fi.framesize);
			test_bool(transformPrepare(&td,dest,dest)== DS_OK);
			test_bool(transformYUV(&td, t)== DS_OK);
		}
		timeCFP = timeOfDayinMS() - start;
		fprintf(stderr,"***FP  elapsed time for %i runs: %i ms ****\n",
						numruns, timeCFP );
		if(it==BiLinear){
			storePGMImage("transformed_FP.pgm", td.dest, testdata->fi);
			storePGMImage("transformed_u_FP.pgm",
										td.dest+testdata->fi.width*testdata->fi.height, testdata->fi_color);
			fprintf(stderr,"stored transformed_FP.pgm\n");
		}
		fprintf(stderr,"***Speedup %3.2f\n", (double)timeC/timeCFP);
		// validate
		int sum=0;
		for(i=0; i<testdata->fi.framesize; i++){
			if(abs(cfinal[i] - td.dest[i])>2){
				sum+=cfinal[i] - td.dest[i];
				//printf("%i,%i: %i\n", i/fi.width, i%fi.width ,cfinal[i] - td.dest[i]);
			}
		}
		printf("***Difference: %i\n", sum);
		cleanupTransformData(&td);
		test_bool(sum==0);
	}

	free(dest);
	free(cfinal);
}
