void test_motionDetect(TestData* testdata){
	MotionDetect md;
	test_bool(initMotionDetect(&md, &testdata->fi, "test") == VS_OK);
	test_bool(configureMotionDetect(&md)== VS_OK);
	TransformData td;
	test_bool(initTransformData(&td,
															&testdata->fi, &testdata->fi, "test") == VS_OK);
	test_bool(configureTransformData(&td)== VS_OK);
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
		Transform t;
		test_bool(motionDetection(&md, &localmotions,&testdata->frames[i])== VS_OK);
		/* for(k=0; k < vs_vector_size(&localmotions); k++){ */
		/* 	localmotion_print(LMGet(&localmotions,k),stderr); */
		/* } */
		t = simpleMotionsToTransform(&td, &localmotions);

		vs_vector_del(&localmotions);
		fprintf(stderr, "%i %6.4lf %6.4lf %8.5lf %6.4lf %i\n",
						i, t.x, t.y, t.alpha, t.zoom, t.extra);
		// TODO: here we have to compare with actual transforms!
	}
	int end = timeOfDayinMS();

	fprintf(stderr,"\n*** elapsed time for %i runs: %i ms ****\n", numruns, end-start );

	cleanupMotionDetection(&md);
}
