
void testImageStripeYUV(int size, VSFrameInfo* fi, VSFrame* img){
  int i,j;
	initFrameInfo(fi, size, 4, PF_YUV420P);
  allocateFrame(img,fi);
  memset(img->data[0],100,sizeof(uint8_t)*fi->width*fi->height);
  for(j=0; j<fi->height; j++){
    for(i=0; i<size; i++){
      img->data[0][i+j*img->linesize[0]]= sin(((double)i)/size/(double)j)*128+128;
    }
  }
  memset(img->data[1],100,sizeof(uint8_t)*(fi->width >> 1) *(fi->height>>1));
  memset(img->data[2],100,sizeof(uint8_t)*(fi->width >> 1) *(fi->height>>1));
  for(j=0; j<fi->height/2; j++){
    for(i=0; i<size/2; i++){
      img->data[1][i+j*img->linesize[1]]= sin(((double)i)/size/j*2.0)*128+128;
      img->data[2][i+j*img->linesize[2]]= cos(((double)i)/size/j*4.0)*128+128;
    }
  }
}


void test_transform_implementation(const TestData* testdata){

  VSFrameInfo fi;
  VSFrame src;
  testImageStripeYUV(128,&fi,&src);
	VSFrame dest;
	allocateFrame(&dest,&fi);
  VSFrame cfinal;
	allocateFrame(&cfinal,&fi);
  TransformData td;
  fprintf(stderr,"--- Validate Interpolations ----\n");

  int it;
  int i;
  int sum;
  Transform t;
  t.x = 10;
  t.alpha = 2*M_PI/(180.0);

  for(it=Zero; it<=BiCubic; it++){
    copyFrame(&dest, &src, &fi);
    test_bool(initTransformData(&td, &fi, &fi, "test") == VS_OK);
    td.interpolType=it;
    test_bool(configureTransformData(&td)== VS_OK);

    fprintf(stderr,"Transform: %s\n", interpolTypes[it]);
    test_bool(transformPrepare(&td,&dest,&dest)== VS_OK);
    test_bool(transformYUV_float(&td, t)== VS_OK);

    copyFrame(&cfinal,&td.dest,&fi);
		cleanupTransformData(&td);

    copyFrame(&dest, &src, &fi);
    test_bool(initTransformData(&td, &fi, &fi, "test") == VS_OK);
    td.interpolType=it;
    test_bool(configureTransformData(&td)== VS_OK);
    test_bool(transformPrepare(&td,&dest,&dest)== VS_OK);
    test_bool(transformYUV(&td, t)== VS_OK);

    // validate
    sum=0;
    for(i=0; i<fi.width*fi.height; i++){
			int diff = cfinal.data[0][i] - td.dest.data[0][i];
      if(abs(diff)>2){
				sum+=abs(diff);
				printf("%i,%i: %i\n", i/fi.width, i%fi.width, diff);
      }
    }
		cleanupTransformData(&td);
    printf("***Difference: %i\n", sum);
		test_bool(sum==0);
  }
	freeFrame(&dest);
	freeFrame(&cfinal);
	freeFrame(&src);
}

void test_transform_performance(const TestData* testdata){


	fprintf(stderr,"--- Performance of Transforms ----\n");
	VSFrame dest;
	VSFrame cfinal;
	int it;
	int start, numruns;
	int timeC, timeCFP; //, timeOrc;
	allocateFrame(&dest, &testdata->fi);
	allocateFrame(&cfinal, &testdata->fi);
	numruns = 5;
	for(it=Zero; it<=BiCubic; it++){
		TransformData td;
		int i;
		//// Float implementation
		test_bool(initTransformData(&td, &testdata->fi, &testdata->fi, "test") == VS_OK);
		td.interpolType=it;
		test_bool(configureTransformData(&td)== VS_OK);

		fprintf(stderr,"Transform: %s", interpolTypes[it]);
		start = timeOfDayinMS();
		for(i=0; i<numruns; i++){
			Transform t = null_transform();
			t.x = i*10+10;
			t.alpha = (i+1)*2*M_PI/(180.0);
			t.zoom = 0;
			copyFrame(&dest, &testdata->frames[0], &testdata->fi);
			test_bool(transformPrepare(&td,&dest,&dest)== VS_OK);
			test_bool(transformYUV_float(&td, t)== VS_OK);
		}
		timeC = timeOfDayinMS() - start;
		fprintf(stderr,"\n***C   elapsed time for %i runs: %i ms ****\n",
						numruns, timeC );

		if(it==BiLinear){
			storePGMImage("transformed.pgm", td.dest.data[0], testdata->fi);
			storePGMImage("transformed_u.pgm", td.dest.data[1], testdata->fi_color);
			fprintf(stderr,"stored transformed.pgm\n");
		}
		copyFrame(&cfinal,&td.dest,&testdata->fi);
		cleanupTransformData(&td);

		//// fixed point implementation
		test_bool(initTransformData(&td, &testdata->fi, &testdata->fi, "test") == VS_OK);
		td.interpolType=it;
		test_bool(configureTransformData(&td)== VS_OK);
		start = timeOfDayinMS();
		for(i=0; i<numruns; i++){
			Transform t = null_transform();
			t.x = i*10+10;
			t.alpha = (i+1)*2*M_PI/(180.0);
			t.zoom = 0;
			copyFrame(&dest, &testdata->frames[0], &testdata->fi);
			test_bool(transformPrepare(&td,&dest,&dest)== VS_OK);
			test_bool(transformYUV(&td, t)== VS_OK);
		}
		timeCFP = timeOfDayinMS() - start;
		fprintf(stderr,"***FP  elapsed time for %i runs: %i ms ****\n",
						numruns, timeCFP );
		if(it==BiLinear){
			storePGMImage("transformed_FP.pgm", td.dest.data[0], testdata->fi);
			storePGMImage("transformed_u_FP.pgm", td.dest.data[1], testdata->fi_color);
			fprintf(stderr,"stored transformed_FP.pgm\n");
		}
		fprintf(stderr,"***Speedup %3.2f\n", (double)timeC/timeCFP);
		// validate
		int sum=0;
		for(i=0; i<testdata->fi.width*testdata->fi.height; i++){
			int diff = cfinal.data[0][i] - td.dest.data[0][i];
      if(abs(diff)>2){
				sum+=abs(diff);
				//printf("%i,%i: %i\n", i/fi.width, i%fi.width, diff);
      }
    }
		printf("***Difference: %i\n", sum);
		cleanupTransformData(&td);
		test_bool(sum==0);
	}

	freeFrame(&dest);
	freeFrame(&cfinal);
}
