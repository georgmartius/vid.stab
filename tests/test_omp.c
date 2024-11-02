#ifdef USE_OMP
int openmptest(){
  int start = timeOfDayinMS();
  long int sum=0;
  int i,j;

#pragma omp parallel for shared(sum) private(i,j)
  for (i=0; i<10;i++){
    printf("num threads: %i\n",omp_get_thread_num());
    long int k=0;
    for (j=0; j<4000000;j++){
      k+=sqrt(j);
    }
#pragma omp atomic
    sum+=k;
  }
  int end = timeOfDayinMS();
  fprintf(stderr, "Sum: %li\n",sum);
  return end-start;
}
int openmp(){
  fprintf(stderr, "Processors: %i, Max # threads: %i\n", omp_get_num_procs(), omp_get_max_threads());
  int num = omp_get_max_threads();

  int time, timeref;
  omp_set_dynamic( 0 );
  omp_set_num_threads( 1 );
  fprintf(stderr,"********** omp speedtest:\n");
  time = openmptest();
  fprintf(stderr,"***C    time: %i ms\n",  time);
  timeref=time;
  omp_set_dynamic( 1 );
  omp_set_num_threads( num);
  time = openmptest();
  fprintf(stderr,"***C (%i)time: %i ms, Speedup %f\n", num, time,
          (double)timeref/time);
  omp_set_dynamic( 1 );
  return 1;
}
#endif
