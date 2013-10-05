

double square_test (VSArray p, void* _dat){
  double val=0;
  for(int k=0; k<p.len;k++){
    val+=(k+1)*3*(p.dat[k]-k)*(p.dat[k]-k);
  }
  return val;
}

void test_gradientoptimizer(){
  int numruns=10;

  fprintf(stderr,"********** Gradient Optimizer Test:\n");

  for(int i=0; i<numruns; i++){
    double residual;
    VSArray params = vs_array_new(i+1);
    VSArray stepsizes = vs_array_new(i+1);
    for(int k=0; k<i+1; k++){
      params.dat[k]= 20-k;
      stepsizes.dat[k]= 0.1;
    }

    VSArray result = vsGradientDescent(square_test, params, NULL, 50, stepsizes, 1e-15, &residual);
    fprintf(stderr,"** %iD: residual %lg :", i+1, residual);
    vs_array_print(result, stderr);
    fprintf(stderr,"***\n");
    test_bool(residual<1e-10);
    vs_array_free(result);
    vs_array_free(params);
  }
}
