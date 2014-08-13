void test_localmotion2transform(TestData* testdata){
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig("test_localmotion2transform");
  VSMotionDetect md;
  test_bool(vsMotionDetectInit(&md, &mdconf, &testdata->fi) == VS_OK);

  VSTransformConfig tdconf = vsTransformGetDefaultConfig("test_localmotion2transform-trans");
  VSTransformData td;

  test_bool(vsTransformDataInit(&td, &tdconf, &testdata->fi, &testdata->fi) == VS_OK);
  fprintf(stderr,"MotionDetect:\n");
  int numruns =5;
  int i;
  //int t;
  //        for(t = 1; t <= 4; t++){
  int start = timeOfDayinMS();
  //      omp_set_dynamic( 0 );
  //      omp_set_num_threads( t );

  for(i=0; i<numruns; i++){
    LocalMotions localmotions;
    VSTransform t;
    test_bool(vsMotionDetection(&md, &localmotions,&testdata->frames[i])== VS_OK);
    /* for(k=0; k < vs_vector_size(&localmotions); k++){ */
    /*   localmotion_print(LMGet(&localmotions,k),stderr); */
    /* } */
    t = vsMotionsToTransform(&td, &localmotions, 0);

    vs_vector_del(&localmotions);
    fprintf(stderr,"%i: ",i);
    storeVSTransform(stderr,&t);
    VSTransform orig = getTestFrameTransform(i);
    VSTransform diff = sub_transforms(&t,&orig);
    int tolerance = fabs(diff.x)<0.2 && fabs(diff.y)<0.2 && fabs(diff.alpha)<0.001;
    if(!tolerance){
      fprintf(stderr,"Difference: ");
      storeVSTransform(stderr,&diff);
    }
    test_bool(tolerance);
  }
  int end = timeOfDayinMS();

  fprintf(stderr,"\n*** elapsed time for %i runs: %i ms ****\n", numruns, end-start );

  vsMotionDetectionCleanup(&md);
}
