/* Regression test for issue #111: the output of vidstabdetect/vidstabtransform
   must not depend on the state of the global RNG nor on the number of threads. */

static double determinism_quadratic(VSArray p, void* _dat){
  double val=0;
  for(int k=0; k<p.len; k++){
    val += (k+1)*3*(p.dat[k]-0.3*k)*(p.dat[k]-0.3*k);
  }
  return val;
}

/* run vsGradientDescent on a fixed problem; the RNG is seeded differently
   before each run, so any use of rand() inside shows up as a difference */
static VSArray determinism_run_gd(unsigned seed, double* residual){
  srand(seed);
  int dim=4;
  VSArray params = vs_array_new(dim);
  VSArray stepsizes = vs_array_new(dim);
  for(int k=0; k<dim; k++){
    params.dat[k]= 7.0-k;
    stepsizes.dat[k]= 0.1;
  }
  VSArray result = vsGradientDescent(determinism_quadratic, params, NULL, 12,
                                     stepsizes, 1e-15, residual);
  vs_array_free(params);
  return result;
}

/* the second pass (vidstabtransform) must produce identical transforms
   for identical local motions, whatever the RNG state is */
static VSTransform determinism_run_m2t(VSTransformData* td,
                                      const LocalMotions* lms, unsigned seed){
  srand(seed);
  return vsMotionsToTransform(td, lms, 0);
}

static int determinism_lm_equal(const LocalMotion* a, const LocalMotion* b){
  return a->v.x==b->v.x && a->v.y==b->v.y
    && a->f.x==b->f.x && a->f.y==b->f.y
    && a->f.size==b->f.size
    && a->contrast==b->contrast && a->match==b->match;
}

static void determinism_print_lm(const LocalMotion* m, FILE* f){
  fprintf(f,"LM: field (%i,%i,%i) v (%i,%i) contrast %f match %f\n",
          m->f.x, m->f.y, m->f.size, m->v.x, m->v.y, m->contrast, m->match);
}

void test_determinism(TestData* testdata){
  fprintf(stderr,"********** Determinism Test (issue #111):\n");

  /* --- cause A: the gradient descent must not use randomness --- */
  double res1, res2;
  VSArray r1 = determinism_run_gd(1, &res1);
  VSArray r2 = determinism_run_gd(987654321u, &res2);
  fprintf(stderr,"gradient descent: residual run1 %lg, run2 %lg\n", res1, res2);
  test_bool(r1.len == r2.len);
  int identical=1;
  for(int k=0; k<r1.len && k<r2.len; k++){
    if(r1.dat[k] != r2.dat[k]){
      identical=0;
      fprintf(stderr,"  param %i differs: %.17g != %.17g\n", k, r1.dat[k], r2.dat[k]);
    }
  }
  if(res1 != res2)
    fprintf(stderr,"  residual differs: %.17g != %.17g\n", res1, res2);
  test_bool(identical);
  test_bool(res1 == res2);
  vs_array_free(r1);
  vs_array_free(r2);

  /* --- cause B: the order of the local motions must not depend on threads --- */
  int threads2=2;
#ifdef USE_OMP
  if(omp_get_max_threads() > 2) threads2 = omp_get_max_threads();
  omp_set_dynamic(1);
#endif
  fprintf(stderr,"motion detection with 1 vs %i threads\n", threads2);

  int loglevel=vs_log_level;
  vs_log_level=0;
  VSMotionDetectConfig mdconf1 = vsMotionDetectGetDefaultConfig("test_determinism1");
  VSMotionDetectConfig mdconf2 = vsMotionDetectGetDefaultConfig("test_determinism2");
  VSMotionDetect md1, md2;
  test_bool(vsMotionDetectInit(&md1, &mdconf1, &testdata->fi) == VS_OK);
  test_bool(vsMotionDetectInit(&md2, &mdconf2, &testdata->fi) == VS_OK);
  md1.conf.numThreads=1;
  md2.conf.numThreads=threads2;
  vs_log_level=loglevel;

  VSTransformConfig tdconf = vsTransformGetDefaultConfig("test_determinism-trans");
  VSTransformData td;
  test_bool(vsTransformDataInit(&td, &tdconf, &testdata->fi, &testdata->fi) == VS_OK);

  int numruns=3;
  for(int i=0; i<numruns; i++){
    LocalMotions lms1, lms2;
    test_bool(vsMotionDetection(&md1, &lms1, &testdata->frames[i]) == VS_OK);
    test_bool(vsMotionDetection(&md2, &lms2, &testdata->frames[i]) == VS_OK);

    int n1=vs_vector_size(&lms1);
    int n2=vs_vector_size(&lms2);
    if(n1!=n2)
      fprintf(stderr,"  frame %i: different number of motions %i != %i\n", i, n1, n2);
    test_bool(n1==n2);
    int sameorder=1;
    for(int k=0; k<n1 && k<n2; k++){
      if(!determinism_lm_equal(LMGet(&lms1,k), LMGet(&lms2,k))){
        if(sameorder){ /* report only the first difference */
          fprintf(stderr,"  frame %i: motion %i differs (order depends on threads):\n    ",i,k);
          determinism_print_lm(LMGet(&lms1,k),stderr);
          fprintf(stderr,"    ");
          determinism_print_lm(LMGet(&lms2,k),stderr);
        }
        sameorder=0;
      }
    }
    test_bool(sameorder);

    /* and the resulting transform must be bit-identical, too */
    VSTransform t1 = determinism_run_m2t(&td, &lms1, 4711);
    VSTransform t2 = determinism_run_m2t(&td, &lms2, 42);
    int samet = t1.x==t2.x && t1.y==t2.y && t1.alpha==t2.alpha
      && t1.zoom==t2.zoom && t1.extra==t2.extra;
    if(!samet){
      fprintf(stderr,"  frame %i: transforms differ:\n    ",i);
      storeVSTransform(stderr,&t1);
      fprintf(stderr,"    ");
      storeVSTransform(stderr,&t2);
    }
    test_bool(samet);

    vs_vector_del(&lms1);
    vs_vector_del(&lms2);
  }

  vsMotionDetectionCleanup(&md1);
  vsMotionDetectionCleanup(&md2);
  fprintf(stderr,"*** determinism test done ***\n");
}
