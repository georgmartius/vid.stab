void test_motionDetect(TestData* testdata){
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig("test_motionDetect");
  VSMotionDetect md;

  VSTransformConfig tdconf = vsTransformGetDefaultConfig("test_motionDetect-trans");
  VSTransformData td;
  test_bool(vsTransformDataInit(&td, &tdconf, &testdata->fi, &testdata->fi) == VS_OK);

  fprintf(stderr,"MotionDetect:\n");
  int threads=1;
  int single_time=0;
  int loglevel=vs_log_level;
#ifdef USE_OMP
  int numthreads=omp_get_max_threads();
  for(threads = 1; threads <= numthreads; threads++){
#endif
    test_bool(vsMotionDetectInit(&md, &mdconf, &testdata->fi) == VS_OK);
    int numruns =5;
    int i;

    int start = timeOfDayinMS();
#ifdef USE_OMP
    omp_set_dynamic( 1 );
#endif
    md.conf.numThreads=threads;

    for(i=0; i<numruns; i++){
      LocalMotions localmotions;
      VSTransform t;

      test_bool(vsMotionDetection(&md, &localmotions,&testdata->frames[i])== VS_OK);
      /* for(k=0; k < vs_vector_size(&localmotions); k++){ */
      /*   localmotion_print(LMGet(&localmotions,k),stderr); */
      /* } */
      t = vsSimpleMotionsToTransform(td.fiSrc, td.conf.modName, &localmotions);

      vs_vector_del(&localmotions);
      if(threads==1){
        fprintf(stderr,"%i: ",i);
        storeVSTransform(stderr,&t);
      }
      VSTransform orig = mult_transform_(getTestFrameTransform(i),-1.0);
      VSTransform diff = sub_transforms(&t,&orig);
      int success = fabs(diff.x)<2 && fabs(diff.y)<2 && fabs(diff.alpha)<0.005;
      if(!success){
        fprintf(stderr,"Difference: ");
        storeVSTransform(stderr,&diff);
      }
      test_bool(success);
    }
    int end = timeOfDayinMS();
    if(threads==1){
      single_time = end-start;
    }
    double speedup=single_time/(double)(end-start);
    fprintf(stderr,"*** elapsed time for %i runs (%i threads): %i ms (%f speedup (%f))****\n", numruns, threads,  end-start, speedup, speedup/threads );
#ifdef USE_OMP
    vs_log_level=1;
  }
  vs_log_level=loglevel;

#endif
  vsMotionDetectionCleanup(&md);

}
