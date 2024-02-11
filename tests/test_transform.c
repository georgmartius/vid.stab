
void testImageStripeYUV(int size, VSFrameInfo* fi, VSFrame* img){
  int i,j;
  vsFrameInfoInit(fi, size, 4, PF_YUV420P);
  vsFrameAllocate(img,fi);
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
  vsFrameAllocate(&dest,&fi);
  VSFrame cfinal;
  vsFrameAllocate(&cfinal,&fi);
  VSTransformData td;
  VSTransformConfig conf = vsTransformGetDefaultConfig("test_transform_implementation");

  fprintf(stderr,"--- Validate Interpolations ----\n");

  int it;
  int i;
  int sum;
  VSTransform t;
  t.x = 10;
  t.alpha = 2*M_PI/(180.0);

  for(it=VS_Zero; it<=VS_BiCubic; it++){
    vsFrameCopy(&dest, &src, &fi);
    conf.interpolType=it;
    test_bool(vsTransformDataInit(&td, &conf, &fi, &fi) == VS_OK);

    fprintf(stderr,"Transform: %s\n", getInterpolationTypeName(it));
    test_bool(vsTransformPrepare(&td,&dest,&dest)== VS_OK);
    test_bool(transformPlanar_float(&td, t)== VS_OK);

    vsFrameCopy(&cfinal,&td.dest,&fi);
    vsTransformDataCleanup(&td);

    vsFrameCopy(&dest, &src, &fi);
    test_bool(vsTransformDataInit(&td, &conf, &fi, &fi) == VS_OK);
    test_bool(vsTransformPrepare(&td,&dest,&dest)== VS_OK);
    test_bool(transformPlanar(&td, t)== VS_OK);

    // validate
    sum=0;
    for(i=0; i<fi.width*fi.height; i++){
      int diff = cfinal.data[0][i] - td.dest.data[0][i];
      if(abs(diff)>2){
        sum+=abs(diff);
        printf("%i,%i: %i\n", i/fi.width, i%fi.width, diff);
      }
    }
    vsTransformDataCleanup(&td);
    printf("***Difference: %i\n", sum);
    test_bool(sum==0);
  }
  vsFrameFree(&dest);
  vsFrameFree(&cfinal);
  vsFrameFree(&src);
}

void test_transform_performance(const TestData* testdata){


  VSTransformConfig conf = vsTransformGetDefaultConfig("test_transform_performance");
  fprintf(stderr,"--- Performance of Transforms ----\n");
  VSFrame dest;
  VSFrame cfinal;
  int it;
  int start, numruns;
  int timeC, timeCFP;
  vsFrameAllocate(&dest, &testdata->fi);
  vsFrameAllocate(&cfinal, &testdata->fi);
  numruns = 5;
  for(it=VS_Zero; it<=VS_BiCubic; it++){
    VSTransformData td;
    int i;
    //// Float implementation
    conf.interpolType=it;
    test_bool(vsTransformDataInit(&td, &conf, &testdata->fi, &testdata->fi) == VS_OK);

    fprintf(stderr,"Transform: %s", getInterpolationTypeName(it));
    start = timeOfDayinMS();
    for(i=0; i<numruns; i++){
      VSTransform t = null_transform();
      t.x = i*10+10;
      t.alpha = (i+1)*2*M_PI/(180.0);
      t.zoom = 0;
      vsFrameCopy(&dest, &testdata->frames[0], &testdata->fi);
      test_bool(vsTransformPrepare(&td,&dest,&dest)== VS_OK);
      test_bool(transformPlanar_float(&td, t)== VS_OK);
    }
    timeC = timeOfDayinMS() - start;
    fprintf(stderr,"\n***C   elapsed time for %i runs: %i ms ****\n",
            numruns, timeC );

    if(it==VS_BiLinear){
      storePGMImage("transformed.pgm", td.dest.data[0], testdata->fi);
      storePGMImage("transformed_u.pgm", td.dest.data[1], testdata->fi_color);
      fprintf(stderr,"stored transformed.pgm\n");
    }
    vsFrameCopy(&cfinal,&td.dest,&testdata->fi);
    vsTransformDataCleanup(&td);

    //// fixed point implementation
    test_bool(vsTransformDataInit(&td, &conf, &testdata->fi, &testdata->fi) == VS_OK);
    start = timeOfDayinMS();
    for(i=0; i<numruns; i++){
      VSTransform t = null_transform();
      t.x = i*10+10;
      t.alpha = (i+1)*2*M_PI/(180.0);
      t.zoom = 0;
      vsFrameCopy(&dest, &testdata->frames[0], &testdata->fi);
      test_bool(vsTransformPrepare(&td,&dest,&dest)== VS_OK);
      test_bool(transformPlanar(&td, t)== VS_OK);
    }
    timeCFP = timeOfDayinMS() - start;
    fprintf(stderr,"***FP  elapsed time for %i runs: %i ms ****\n",
            numruns, timeCFP );
    if(it==VS_BiLinear){
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
    vsTransformDataCleanup(&td);
    test_bool(sum==0);
  }

  vsFrameFree(&dest);
  vsFrameFree(&cfinal);
}
