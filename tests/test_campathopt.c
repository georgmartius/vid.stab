

VSTransformations copyTransforms(VSTransformations* src){
  VSTransformations trans;
  trans.ts=vs_malloc(sizeof(VSTransform)*src->len);
  trans.len=src->len;
  trans.current=src->current;
  for(int i=0; i<src->len; i++){
    trans.ts[i]=src->ts[i];
  }
  return trans;
}


VSTransformations genSmoothTransforms(int num){
  VSTransformations trans;
  trans.ts=vs_malloc(sizeof(VSTransform)*num);
  trans.len=num;
  trans.current=0;
  for(int i=0; i<num; i++){
    trans.ts[i].x = 0.0;
    trans.ts[i].y = 50*sin(((double)i)/10.0);
    trans.ts[i].alpha = M_PI/180*sin(0.1+((double)i)/20.0);
    trans.ts[i].y = 0;
    trans.ts[i].alpha = 0;
    trans.ts[i].zoom = 0.0;
  }
  return trans;
}

double myrand() { return ((double)rand())/RAND_MAX * 2.0 -1.0; }

void perturbTransforms(VSTransformations* trans, double var){
  // the 0th transform is 0
  for(int i=1; i<trans->len; i++){
    //  int i = 5;
  trans->ts[i].x += myrand()*var;
  trans->ts[i].y += myrand()*var;
  trans->ts[i].alpha += M_PI/180*myrand()*var;
  trans->ts[i].zoom += myrand()*var;

  //  i = 6;
  //  trans->ts[i] = invert_transform(&trans->ts[i-1]);

    }
}

VSTransformations integrateTransforms(VSTransformations* trans){
  VSTransformations dest = copyTransforms(trans);
  VSTransform t=null_transform();
  for(int i=0; i<trans->len; i++){
    t = concat_transforms(&t,&(trans->ts[i]));
    dest.ts[i] = t;
  }
  return dest;
}

VSTransformations derivativeTransforms(VSTransformations* trans){
  VSTransformations dest;
  dest.ts=vs_malloc(sizeof(VSTransform)*trans->len-1);
  dest.len=trans->len-1;
  dest.ts[0] = null_transform();
  for(int i=0; i<trans->len-1; i++){
    dest.ts[i] = sub_transforms(&(trans->ts[i+1]),&(trans->ts[i]));
  }
  return dest;
}

VSTransformations stabilizeTransforms(VSTransformations* trans, VSTransformations* transT){
  test_bool(trans->len==transT->len);
  VSTransformations dest = integrateTransforms(trans);
  for(int i=0; i<trans->len; i++){
    dest.ts[i] = concat_transforms(&(dest.ts[i]), &(transT->ts[i]));
    // dest.ts[i] = concat_transforms( &(transT->ts[i]), &(dest.ts[i]));
  }

  return dest;
}

double transformsDifference(VSTransformations* trans1, VSTransformations* trans2){
  test_bool(trans1->len==trans2->len);
  double sum=0;
  for(int i=0; i<trans1->len; i++){
    sum+=fabs(trans1->ts[i].x - trans2->ts[i].x);
    sum+=fabs(trans1->ts[i].y - trans2->ts[i].y);
    sum+=fabs(trans1->ts[i].alpha - trans2->ts[i].alpha)*180/M_PI;
    sum+=fabs(trans1->ts[i].zoom - trans2->ts[i].zoom);
  }
  return sum;
}

double totalSumTransforms(VSTransformations* trans){
  double sum=0;
  for(int i=0; i<trans->len; i++){
    sum+=fabs(trans->ts[i].x);
    sum+=fabs(trans->ts[i].y);
    sum+=fabs(trans->ts[i].alpha)*180/M_PI;
    sum+=fabs(trans->ts[i].zoom);
  }
  return sum;
}


void writeTransforms(VSTransformations* trans, char* filename){
  FILE* f = fopen(filename,"w");
  if(f){
    for(int i=0; i<trans->len; i++){
      storeVSTransform(f,&(trans->ts[i]));
    }
    fclose(f);
  }
}

void test_campathopt(TestData* testdata){
  VSTransformConfig tdconf = vsTransformGetDefaultConfig("test_campathopt");
  VSTransformData td;

  test_bool(vsTransformDataInit(&td, &tdconf, &testdata->fi, &testdata->fi) == VS_OK);
  int num=10;
  VSTransformations trans=genSmoothTransforms(num);
  VSTransformations transP=genSmoothTransforms(num);
  writeTransforms(&trans,"transforms1.log");

  test_bool(transformsDifference(&trans,&transP)==0.0);
  perturbTransforms(&transP,1);
  writeTransforms(&transP,"transformsP.log");

  VSTransformations beforeD1 = derivativeTransforms(&transP);
  VSTransformations beforeD2 = derivativeTransforms(&beforeD1);

  td.conf.camPathAlgo=
    VSOptimalL1;
  // VSGaussian;
  //VSAvg;

  VSTransformations transO = copyTransforms(&transP);

  cameraPathOptimization(&td, &transO);
  writeTransforms(&transO,"transformsO.log");
  VSTransformations transS = stabilizeTransforms(&transP,&transO);
  writeTransforms(&transS,"transformsS.log");

  VSTransformations afterD1 = derivativeTransforms(&transS);
  VSTransformations afterD2 = derivativeTransforms(&afterD1);
  writeTransforms(&afterD1,"transformsD1a.log");

  double d1_before= totalSumTransforms(&beforeD1);
  double d1_after= totalSumTransforms(&afterD1);
  double d2_before= totalSumTransforms(&beforeD2);
  double d2_after= totalSumTransforms(&afterD2);
  test_bool(d1_after < d1_before);
  test_bool(d2_after < d2_before);
  printf("cam-path optimizer: L1Opt: D1: %f -> %f \t D2: %f -> %f \n", d1_before, d1_after, d2_before, d2_after);

  writeTransforms(&transS,"transformsS.log");



  vsTransformDataCleanup(&td);
}
