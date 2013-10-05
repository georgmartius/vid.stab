#define NUM_RECTANGLES 100
void generateFrames(TestData* testdata, int num){
  int i;
  for(i=0; i<num; i++){
    vsFrameAllocate(&testdata->frames[i],&testdata->fi);
  }
  // first frame noise
  fillArrayWithNoise(testdata->frames[0].data[0],
                     testdata->fi.width*testdata->fi.height, 10);
  fillArrayWithNoise(testdata->frames[0].data[1],
                     testdata->fi.width/2*testdata->fi.height/2, 5);
  fillArrayWithNoise(testdata->frames[0].data[2],
                     testdata->fi.width/2*testdata->fi.height/2, 5);

  // add rectangles
  int k;
  for(k=0; k<NUM_RECTANGLES; k++){
    paintRectangle(testdata->frames[0].data[0],&testdata->fi,
                   randUpTo(testdata->fi.width), randUpTo(testdata->fi.height),
                   randUpTo((testdata->fi.width>>4)+4),
                   randUpTo((testdata->fi.height>>4)+4),randPixel());

  }

  VSTransformConfig conf = vsTransformGetDefaultConfig("test_generate");
  conf.interpolType=VS_Zero;
  VSTransformData td;
  test_bool(vsTransformDataInit(&td, &conf, &testdata->fi, &testdata->fi) == VS_OK);

  fprintf(stderr, "testframe transforms\n");

  for(i=1; i<num; i++){
    VSTransform t = getTestFrameTransform(i);
    fprintf(stderr,"%i: ",i);
    storeVSTransform(stderr,&t);

    test_bool(vsTransformPrepare(&td,&testdata->frames[i-1],&testdata->frames[i])== VS_OK);
    test_bool(transformPlanar_float(&td, t)== VS_OK);
    test_bool(vsTransformFinish(&td)== VS_OK);
  }
  vsTransformDataCleanup(&td);
}

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 *   c-basic-offset: 2 t
 * End:
 *
 * vim: expandtab shiftwidth=2:
 */
