void test_motionDetect(TestData* testdata){
  VSMotionDetect md;
  test_bool(vsMotionDetectInit(&md, &testdata->fi, "test") == VS_OK);
  test_bool(vsMotionDetectConfigure(&md)== VS_OK);
  VSTransformData td;
  test_bool(vsTransformDataInit(&td,
                              &testdata->fi, &testdata->fi, "test") == VS_OK);
  test_bool(vsTransformDataConfigure(&td)== VS_OK);
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
    t = vsSimpleMotionsToTransform(&td, &localmotions);

    vs_vector_del(&localmotions);
    fprintf(stderr, "%i %6.4lf %6.4lf %8.5lf %6.4lf %i\n",
            i, t.x, t.y, t.alpha, t.zoom, t.extra);
    // TODO: here we have to compare with actual transforms!
  }
  int end = timeOfDayinMS();

  fprintf(stderr,"\n*** elapsed time for %i runs: %i ms ****\n", numruns, end-start );

  vsMotionDetectionCleanup(&md);
}
