int compare_localmotions(const LocalMotions* lms1, const LocalMotions* lms2){
	test_bool(ds_vector_size(lms1) == ds_vector_size(lms2));
	int i;
	for(i=0; i<ds_vector_size(lms1); i++){
		test_bool(LMGet(lms1,i)->v.x == LMGet(lms2,i)->v.x);
		test_bool(LMGet(lms1,i)->v.y == LMGet(lms2,i)->v.y);
	}
	return 1;
}

int test_store_restore(const TestData* testdata){
	MotionDetect md;
	test_bool(initMotionDetect(&md, &testdata->fi, "test") == DS_OK);
	test_bool(configureMotionDetect(&md)== DS_OK);

	LocalMotions lms;
	int i;
	for(i=0; i<2; i++){
		test_bool(motionDetection(&md, &lms,testdata->frames[i])== DS_OK);
		if (i==0) ds_vector_del(&lms);
	}

	FILE* f = fopen("lmtest","w");
	storeLocalmotions(f,&lms);
	fclose(f);
	f = fopen("lmtest","r");
	LocalMotions test = restoreLocalmotions(f);
	fclose(f);
	storeLocalmotions(stderr,&test);
	compare_localmotions(&lms,&test);
	fprintf(stderr,"\n** LM and LMS OKAY\n");

	f = fopen("lmstest","w");
	md.frameNum=0;
	prepareFile(&md,f);
	writeToFile(&md,f,&lms);
	md.frameNum=1;
	writeToFile(&md,f,&test);
	fclose(f);

	f = fopen("lmstest","r");
	test_bool(readFileVersion(f)==1);
	LocalMotions read1;
	test_bool(readFromFile(f,&read1)==0);
	compare_localmotions(&lms,&read1);
	LocalMotions read2;
	test_bool(readFromFile(f,&read2)==1);
	compare_localmotions(&test,&read2);
	fclose(f);
	fprintf(stderr,"** Reading file stepwise OKAY\n");
	ds_vector_del(&read1);
	ds_vector_del(&read2);
	ds_vector_del(&test);
	ds_vector_del(&lms);

	f = fopen("lmstest","r");
	ManyLocalMotions mlms;
	test_bool(readLocalMotionsFile(f,&mlms)==DS_OK);
	test_bool(ds_vector_size(&mlms)==2);
	fprintf(stderr,"** Entire file routine OKAY\n\n");

	for(i=0; i< ds_vector_size(&mlms); i++){
		if(MLMGet(&mlms,i))
			ds_vector_del(MLMGet(&mlms,i));
	}
	ds_vector_del(&mlms);

	return 1;
}
