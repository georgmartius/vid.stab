

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

VSTransformationsLS copyTransformsLS(VSTransformationsLS* src){
  VSTransformationsLS trans;
  trans.ts=vs_malloc(sizeof(VSTransformLS)*src->len);
  trans.len=src->len;
  trans.current=src->current;
  for(int i=0; i<src->len; i++){
    trans.ts[i]=src->ts[i];
  }
  return trans;
}


VSTransformations genSmoothTransforms(int num, int flat){
  VSTransformations trans;
  trans.ts=vs_malloc(sizeof(VSTransform)*num);
  trans.len=num;
  trans.current=0;
  for(int i=0; i<num; i++){
    trans.ts[i].x     = 0.0;
    trans.ts[i].y     = flat ? 0 : 2*sin(((double)i)/10.0);
    trans.ts[i].alpha = flat ? 0 : M_PI/180*sin(0.1+((double)i)/20.0);
    trans.ts[i].zoom  = 0.0;
  }
  return trans;
}

double myrand() { return ((double)rand())/RAND_MAX * 2.0 -1.0; }

void perturbTransforms(VSTransformations* trans, double var, int onlyone){
  // the 0th transform is 0
  for(int i= onlyone ? 2 : 1; i< (onlyone ? 3 : trans->len); i++){
    trans->ts[i].x += myrand()*var;
    trans->ts[i].y += myrand()*var;
    trans->ts[i].alpha += M_PI/180*myrand()*var;
    trans->ts[i].zoom += myrand()*var;
  }
  if(onlyone){
    trans->ts[3] = invert_transform(&trans->ts[3-1]);
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

VSTransformationsLS integrateTransformsLS(VSTransformationsLS* trans){
  VSTransformationsLS dest = copyTransformsLS(trans);
  VSTransformLS t=id_transformLS();
  for(int i=0; i<trans->len; i++){
    t = concat_transformLS(&t,&(trans->ts[i]));
    dest.ts[i] = t;
  }
  return dest;
}

VSTransformations derivativeTransforms(VSTransformations* trans){
  VSTransformations dest;
  dest.ts=vs_malloc(sizeof(VSTransform)*trans->len-1);
  dest.len=trans->len-1;
  if(trans->len-1>0){
    dest.len=trans->len-1;
    dest.ts[0] = null_transform();
    for(int i=0; i<trans->len-1; i++){
      dest.ts[i] = sub_transforms(&(trans->ts[i+1]),&(trans->ts[i]));
    }
  }
  return dest;
}

VSTransformationsLS derivativeTransformsLS(VSTransformationsLS* trans){
  VSTransformationsLS dest;
  dest.ts=vs_malloc(sizeof(VSTransformLS)*trans->len-1);
  dest.len=trans->len-1;
  if(trans->len-1>0){
    dest.len=trans->len-1;
    dest.ts[0] = id_transformLS();
    for(int i=0; i<trans->len-1; i++){
      dest.ts[i] = sub_transformLS(&(trans->ts[i+1]),&(trans->ts[i]));
    }
  }
  return dest;
}

// calculates the "wrong" second derivative as in the Grundmann paper
VSTransformationsLS getGrundmannDerivative2(VSTransformationsLS* fs, VSTransformationsLS* bs){
  VSTransformationsLS dest;
  dest.ts=vs_malloc(sizeof(VSTransformLS)*fs->len-2);
  dest.len=fs->len-2;
  if(fs->len-2>0){
    dest.len=fs->len-2;
    for(int i=0; i<fs->len-2; i++){

      VSTransformLS fp1 = add_transformLS_(id_transformLS(),fs->ts[i+1]);
      dest.ts[i] = add_transformLS_( sub_transformLS_(concat_transformLS(&fs->ts[i+2],&bs->ts[i+2]),
                                                      concat_transformLS(&fp1,&bs->ts[i+1])),
                                     bs->ts[i]
                                     );

    }
  }
  return dest;
}


VSTransformations stabilizeTransforms(VSTransformations* trans, VSTransformations* transT){
  test_bool(trans->len==transT->len);
  VSTransformations dest = integrateTransforms(trans);
  for(int i=0; i<trans->len; i++){
    dest.ts[i] = concat_transforms(&(dest.ts[i]), &(transT->ts[i]));
  }
  return dest;
}

VSTransformationsLS stabilizeTransformsLS(VSTransformationsLS* trans, VSTransformationsLS* transT){
  test_bool(trans->len==transT->len);
  VSTransformationsLS dest = integrateTransformsLS(trans);
  for(int i=0; i<trans->len; i++){
    dest.ts[i] = concat_transformLS(&(dest.ts[i]), &(transT->ts[i]));
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

double totalSumTransformsLS(VSTransformationsLS* trans){
  double sum=0;
  for(int i=0; i<trans->len; i++){
    sum+=fabs(trans->ts[i].x);
    sum+=fabs(trans->ts[i].y);
    sum+=fabs(trans->ts[i].a*100);
    sum+=fabs(trans->ts[i].b*100);
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

void writeTransformsLS(VSTransformationsLS* trans, char* filename){
  FILE* f = fopen(filename,"w");
  if(f){
    for(int i=0; i<trans->len; i++){
      storeVSTransformLS(f,&(trans->ts[i]));
    }
    fclose(f);
  }
}

#define X 0
#define Y 1
#define A 2
#define B 3

#define P 0
#define E1 1
#define E2 2
#define E3 3
#define CORNER 4
int getRowNum(int e, int t, int param, int upperorlower, int N);
int getColNum(int p_e, int t, int param, int N);
#define TESTSUCC(a,b,m1,m2,m3) if((a)!=(b)+1){ fprintf(stderr,"expected: %i, but got %i (%i,%i,%i)",(b)+1,a,m1,m2,m3);} test_bool((a)==(b)+1);
void test_l1optutils(){
  int i=0;
  int r;
  const int N=5;
  for(int t=0; t<N-1;t++){
    for(int p=X; p<=B; p++){
      r=getRowNum(E1,t,p,0,N); TESTSUCC(r,i,E1,t,p); i = r;
      r=getRowNum(E1,t,p,1,N); TESTSUCC(r,i,E1,t,p); i = r;
    }
  }
  for(int t=0; t<N-2;t++){
    for(int p=X; p<=B; p++){
      r=getRowNum(E2,t,p,0,N); TESTSUCC(r,i,E2,t,p); i = r;
      r=getRowNum(E2,t,p,1,N); TESTSUCC(r,i,E2,t,p); i = r;
    }
  }
  for(int t=0; t<N-3;t++){
    for(int p=X; p<=B; p++){
      r=getRowNum(E3,t,p,0,N); TESTSUCC(r,i,E3,t,p); i = r;
      r=getRowNum(E3,t,p,1,N); TESTSUCC(r,i,E3,t,p); i = r;
    }
  }
  for(int t=0; t<N;t++){
    for(int p=X; p<=B; p++){
      r=getRowNum(CORNER,t,p,0,N); TESTSUCC(r,i,CORNER,t,p); i = r;
    }
  }

  i=0;
  for(int t=0; t<N;t++){
    for(int p=X; p<=B; p++){
      r=getColNum(P,t,p,N); TESTSUCC(r,i,P,t,p); i = r;
    }
  }
  for(int t=0; t<N-1;t++){
    for(int p=X; p<=B; p++){
      r=getColNum(E1,t,p,N); TESTSUCC(r,i,E1,t,p); i = r;
    }
  }
  for(int t=0; t<N-2;t++){
    for(int p=X; p<=B; p++){
      r=getColNum(E2,t,p,N); TESTSUCC(r,i,E2,t,p); i = r;
    }
  }
  for(int t=0; t<N-3;t++){
    for(int p=X; p<=B; p++){
      r=getColNum(E3,t,p,N); TESTSUCC(r,i,E3,t,p); i = r;
    }
  }
}

extern int cameraPathOptimalL1Internal(VSTransformData* td, VSTransformationsLS* trans);

void test_campathopt_LS(TestData* testdata, VSCamPathAlgo algo, int num, int store, int min_perturb){
  VSTransformConfig tdconf = vsTransformGetDefaultConfig("test_campathopt");
  VSTransformData td;

  test_bool(vsTransformDataInit(&td, &tdconf, &testdata->fi, &testdata->fi) == VS_OK);

  VSTransformations trans  = genSmoothTransforms(num, min_perturb);
  VSTransformations transP = genSmoothTransforms(num, min_perturb);
  if(store) writeTransforms(&trans,"transforms1.log");

  test_bool(transformsDifference(&trans,&transP)==0.0);
  perturbTransforms(&transP,2,min_perturb);
  if(store) writeTransforms(&transP,"transformsP.log");

  VSTransformationsLS transLSP = transformationsAZtoLS(&transP);

  VSTransformationsLS beforeD1 = derivativeTransformsLS(&transLSP);
  VSTransformationsLS beforeD2 = derivativeTransformsLS(&beforeD1);
  VSTransformationsLS beforeD3 = derivativeTransformsLS(&beforeD2);

  VSTransformationsLS transO = copyTransformsLS(&transLSP);

  //  td.conf.pathD1Weight=1;
  //  td.conf.pathD2Weight=0;
  //  td.conf.pathD3Weight=0;
  //  td.conf.maxZoom=10;
  //  TODO other algos
  cameraPathOptimalL1Internal(&td, &transO);

  if(store) writeTransformsLS(&transO,"transformsO.log");
  VSTransformationsLS transS = stabilizeTransformsLS(&transLSP,&transO);
  if(store) writeTransformsLS(&transS,"transformsS.log");

  VSTransformationsLS afterD1 = derivativeTransformsLS(&transS);
  VSTransformationsLS afterD2g = getGrundmannDerivative2(&transLSP,&transO);
  VSTransformationsLS afterD2 = derivativeTransformsLS(&afterD1);
  VSTransformationsLS afterD3 = derivativeTransformsLS(&afterD2);
  if(store)   writeTransformsLS(&afterD1,"transformsD1a.log");
  if(store)   writeTransformsLS(&afterD2,"transformsD2a.log");
  if(store)   writeTransformsLS(&afterD2g,"transformsD2ag.log");
  if(store)   writeTransformsLS(&afterD3,"transformsD3a.log");

  double d1_before= totalSumTransformsLS(&beforeD1);
  double d1_after= totalSumTransformsLS(&afterD1);
  double d2_before= totalSumTransformsLS(&beforeD2);
  double d2_after= totalSumTransformsLS(&afterD2);
  double d2_afterg= totalSumTransformsLS(&afterD2g);
  double d3_before= totalSumTransformsLS(&beforeD3);
  double d3_after= totalSumTransformsLS(&afterD3);
  test_bool(d1_after < d1_before);
  test_bool(d2_after < d2_before);
  test_bool(d3_after < d3_before);
  printf("cam-path optimizer: %s: D1: %f -> %f \t D2: %f -> %f (%f) \t D3: %f -> %f \n",
         vsGetCamPathAlgoName(algo),
         d1_before, d1_after, d2_before, d2_after, d2_afterg, d3_before, d3_after);

  if(store)   writeTransformsLS(&transS,"transformsS.log");
  vsTransformDataCleanup(&td);
}

void test_campathopt_AZ(TestData* testdata, VSCamPathAlgo algo, int num, int store, int min_perturb){
  VSTransformConfig tdconf = vsTransformGetDefaultConfig("test_campathopt");
  VSTransformData td;

  test_bool(vsTransformDataInit(&td, &tdconf, &testdata->fi, &testdata->fi) == VS_OK);

  VSTransformations trans  = genSmoothTransforms(num, min_perturb);
  VSTransformations transP = genSmoothTransforms(num, min_perturb);
  if(store) writeTransforms(&trans,"transforms1.log");

  test_bool(transformsDifference(&trans,&transP)==0.0);
  perturbTransforms(&transP, 2, min_perturb);
  if(store) writeTransforms(&transP,"transformsP.log");

  VSTransformations beforeD1 = derivativeTransforms(&transP);
  VSTransformations beforeD2 = derivativeTransforms(&beforeD1);
  VSTransformations beforeD3 = derivativeTransforms(&beforeD2);

  td.conf.camPathAlgo=algo;

  VSTransformations transO = copyTransforms(&transP);

  cameraPathOptimization(&td, &transO);
  if(store) writeTransforms(&transO,"transformsO.log");
  VSTransformations transS = stabilizeTransforms(&transP,&transO);
  if(store) writeTransforms(&transS,"transformsS.log");

  VSTransformations afterD1 = derivativeTransforms(&transS);
  VSTransformations afterD2 = derivativeTransforms(&afterD1);
  VSTransformations afterD3 = derivativeTransforms(&afterD2);
  if(store)   writeTransforms(&afterD1,"transformsD1a.log");
  if(store)   writeTransforms(&afterD2,"transformsD2a.log");
  if(store)   writeTransforms(&afterD3,"transformsD3a.log");

  double d1_before= totalSumTransforms(&beforeD1);
  double d1_after= totalSumTransforms(&afterD1);
  double d2_before= totalSumTransforms(&beforeD2);
  double d2_after= totalSumTransforms(&afterD2);
  double d3_before= totalSumTransforms(&beforeD3);
  double d3_after= totalSumTransforms(&afterD3);
  test_bool(d1_after < d1_before);
  test_bool(d2_after < d2_before);
  test_bool(d3_after < d3_before);
  printf("cam-path optimizer: %s:\t D1: %f -> %f \t D2: %f -> %f \t D3: %f -> %f \n",
         vsGetCamPathAlgoName(algo),
         d1_before, d1_after, d2_before, d2_after, d3_before, d3_after);

  if(store) writeTransforms(&transS,"transformsS.log");
  vsTransformDataCleanup(&td);
}

void test_campathopt(TestData* testdata){
  // FOR L1: Number of Frames
  // 2000: 200 MB < 5 sec
  // 20000: 1.7GB RAM, 30 sec

  test_campathopt_AZ(testdata, VSAvg,       200, 0, 0);
  test_campathopt_AZ(testdata, VSGaussian,  200, 0, 0);
  test_campathopt_AZ(testdata, VSOptimalL1, 200, 1, 0);
  test_campathopt_AZ(testdata, VSOptimalL1, 200, 0, 1);
}
