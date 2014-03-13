
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

int equalTransforms(VSTransform* t1, VSTransform* t2){
  double eps=1e-10;
  return (fabs(t1->x-t2->x)<eps && fabs(t1->y-t2->y)<eps
          && fabs(t1->alpha-t2->alpha)<eps && fabs(t1->zoom-t2->zoom)<eps);
}

void test_transform_basics(const TestData* testdata){
  VSTransform t0 = null_transform(); //ID
  VSTransform t1 = new_transform(1,2,0.1,5,0,0,0);

  // ID -> G = G
  VSTransform t1a = concat_transforms(&t0,&t1);
  test_bool(equalTransforms(&t1a, &t1));
  // G -> ID = G
  VSTransform t1b = concat_transforms(&t1,&t0);
  test_bool(equalTransforms(&t1b, &t1));


  // G -> G^-1 = ID
  VSTransform t2 = invert_transform(&t1);
  VSTransform id = concat_transforms(&t1,&t2);
  storeVSTransform(stdout,&t1);
  storeVSTransform(stdout,&t2);
  storeVSTransform(stdout,&id);
  test_bool(equalTransforms(&id, &t0));

  // double inverse yields original
  VSTransform t3 = new_transform(-5,0,0.2,2,0,0,0);
  VSTransform t4 = invert_transform(&t3);
  VSTransform t5 = invert_transform(&t4);
  test_bool(equalTransforms(&t3, &t5));

  VSTransform t7 = add_transforms(&t3,&t3);
  VSTransform t8 = mult_transform(&t3,2.0);
  test_bool(equalTransforms(&t7, &t8));
  VSTransform t9  = add_transforms(&t3,&t0);
  VSTransform t10 = add_transforms(&t0,&t3);
  test_bool(equalTransforms(&t9, &t3));
  test_bool(equalTransforms(&t9, &t10));

  // in this parametization we no not have a unit (1) transform
}

int equalTransformLSs(VSTransformLS* t1, VSTransformLS* t2){
  double eps=1e-10;
  return (fabs(t1->x-t2->x)<eps && fabs(t1->y-t2->y)<eps
          && fabs(t1->a-t2->a)<eps && fabs(t1->b-t2->b)<eps);
}

void test_transformLS_basics(const TestData* testdata){
  VSTransformLS t0 = id_transformLS(); //ID
  VSTransformLS t1 = new_transformLS(1,2,1.1*cos(0.1),1.1*sin(0.1),1.0,0);

  // ID -> G = G
  VSTransformLS t1a = concat_transformLS(&t0,&t1);
  test_bool(equalTransformLSs(&t1a, &t1));
  // G -> ID = G
  VSTransformLS t1b = concat_transformLS(&t1,&t0);
  test_bool(equalTransformLSs(&t1b, &t1));


  // G -> G^-1 = ID
  VSTransformLS t2 = invert_transformLS(&t1);
  VSTransformLS id = concat_transformLS(&t1,&t2);
  storeVSTransformLS(stdout,&t1);
  storeVSTransformLS(stdout,&t2);
  storeVSTransformLS(stdout,&id);
  test_bool(equalTransformLSs(&id, &t0));

  // double inverse yields original
  VSTransformLS t3 = new_transformLS(-5,0,0.8*cos(0.2),-0.8*sin(0.2),1.0,0);
  VSTransformLS t4 = invert_transformLS(&t3);
  VSTransformLS t5 = invert_transformLS(&t4);
  test_bool(equalTransformLSs(&t3, &t5));

  // conversions
  VSTransform t7   = transformLStoAZ(&t3);
  VSTransformLS t8 = transformAZtoLS(&t7);
  test_bool(equalTransformLSs(&t8, &t3));

  // yield same transforms
  PreparedTransform pt7 = prepare_transform(&t7, &testdata->fi);
  PreparedTransform pt8 = prepare_transformLS(&t8, &testdata->fi);
  Vec p1 = {300,420};
  Vec p1a = transform_vec(&pt7, &p1);
  Vec p1b = transform_vec(&pt8, &p1);
  test_bool(p1a.x==p1b.x && p1a.y==p1b.y);

  // distributive law and identity
  VSTransformLS t19   = add_transformLS_(id_transformLS(),t3);
  VSTransformLS t20  = concat_transformLS(&t1,&t19);
  VSTransformLS t21  = add_transformLS_(concat_transformLS(&t1,&t3),t1);
  test_bool(equalTransformLSs(&t20, &t21));

  // distributive law and identity
  VSTransformLS t22  = add_transformLS_(id_transformLS(),t3);
  VSTransformLS t23  = concat_transformLS(&t22,&t1);
  VSTransformLS t24  = add_transformLS_(concat_transformLS(&t3,&t1),t1);
  test_bool(equalTransformLSs(&t23, &t24));
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
  int timeC, timeCFP; //, timeOrc;
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
