int compare_localmotions(const LocalMotions* lms1, const LocalMotions* lms2){
	test_bool(vs_vector_size(lms1) == vs_vector_size(lms2));
	int i;
	for(i=0; i<vs_vector_size(lms1); i++){
		test_bool(LMGet(lms1,i)->v.x == LMGet(lms2,i)->v.x);
		test_bool(LMGet(lms1,i)->v.y == LMGet(lms2,i)->v.y);
	}
	return 1;
}

int test_store_restore(TestData* testdata){
	VSMotionDetect md;
	test_bool(vsMotionDetectInit(&md, &testdata->fi, "test") == VS_OK);
	test_bool(vsMotionDetectConfigure(&md)== VS_OK);

	LocalMotions lms;
	int i;
	for(i=0; i<2; i++){
		test_bool(vsMotionDetection(&md, &lms,&testdata->frames[i])== VS_OK);
		if (i==0) vs_vector_del(&lms);
	}

	FILE* f = fopen("lmtest","w");
	vsStoreLocalmotions(f,&lms);
	fclose(f);
	f = fopen("lmtest","r");
	LocalMotions test = vsRestoreLocalmotions(f);
	fclose(f);
	vsStoreLocalmotions(stderr,&test);
	compare_localmotions(&lms,&test);
	fprintf(stderr,"\n** LM and LMS OKAY\n");

	f = fopen("lmstest","w");
	md.frameNum=1;
	vsPrepareFile(&md,f);
	vsWriteToFile(&md,f,&lms);
	md.frameNum=2;
	vsWriteToFile(&md,f,&test);
	fclose(f);

	f = fopen("lmstest","r");
	test_bool(vsReadFileVersion(f)==1);
	LocalMotions read1;
	test_bool(vsReadFromFile(f,&read1)==1);
	compare_localmotions(&lms,&read1);
	LocalMotions read2;
	test_bool(vsReadFromFile(f,&read2)==2);
	compare_localmotions(&test,&read2);
	fclose(f);
	fprintf(stderr,"** Reading file stepwise OKAY\n");
	vs_vector_del(&read1);
	vs_vector_del(&read2);
	vs_vector_del(&test);
	vs_vector_del(&lms);

	f = fopen("lmstest","r");
	VSManyLocalMotions mlms;
	test_bool(vsReadLocalMotionsFile(f,&mlms)==VS_OK);
	test_bool(vs_vector_size(&mlms)==2);
	fprintf(stderr,"** Entire file routine OKAY\n\n");

	for(i=0; i< vs_vector_size(&mlms); i++){
		if(VSMLMGet(&mlms,i))
			vs_vector_del(VSMLMGet(&mlms,i));
	}
	vs_vector_del(&mlms);

	return 1;
}
