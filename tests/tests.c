
#include <stdio.h>
#include <assert.h>
#include <sys/time.h>
#include <stdint.h>
#include <limits.h>

#ifdef USE_OMP
#include <omp.h>
#endif

#include "libdeshake.h"
// load optimized functions
#include "motiondetect_opt.h"

#ifndef TESTING
#error TESTING must be defined
#endif

#include "testutils.c"

int	num		     = 2000;
int	test_checkCompareImg = 0;
int	test_motionDetect    = 1;
int	test_transform	     = 1;
int	test_compareImg	     = 1;
int	test_contrastImg     = 1;
int	test_boxblur	     = 1;

int	loadTestFrames       = 0;

/* struct iterdata { */
/*     FILE *f; */
/*     int  counter; */
/* }; */

/* static int dump_trans(DSListItem *item, void *userdata) */
/* { */
/*   struct iterdata *ID = (struct iterdata *)userdata; */

/*   if (item->data) { */
/*     Transform* t = (Transform*)item->data; */
/*     fprintf(ID->f, "%i %6.4lf %6.4lf %8.5lf %6.4lf %i\n", */
/* 	    ID->counter, t->x, t->y, t->alpha, t->zoom, t->extra); */
/*     ID->counter++; */
/*   } */
/*   return 0; /\* never give up *\/ */
/* } */

void testImageStripeYUV(int size, DSFrameInfo* fi, unsigned char** img){
  int i,j;
  fi->width=size;
  fi->height=4;
  fi->strive=size;
  fi->framesize=3*(size*fi->height)/2;
  fi->pFormat = PF_YUV;
  *img = (unsigned char*)malloc(sizeof(char)*fi->framesize);
  memset(*img,100,sizeof(char)*fi->framesize);
  for(j=0; j<fi->height; j++){
    for(i=0; i<size; i++){
      (*img)[i+j*size]= sin(((double)i)/size/(double)j)*128+128;
    }
  }
  for(j=0; j<fi->height/2; j++){
    for(i=0; i<size/2; i++){
      (*img)[i+fi->height*size+j*size/2]= sin(((double)i)/size/j*2.0)*128+128;
    }
  }
}

long timeOfDayinMS() {
  struct timeval t;
  gettimeofday(&t, 0);
  return t.tv_sec*1000 + t.tv_usec/1000;
}

int checkCompareImg(MotionDetect* md, unsigned char* frame){
  int i;
  int error;
  uint8_t *Y_c;
  Field field;
  field.x=400;
  field.y=400;
  field.size=12;

  Y_c = frame;

  for(i=-10;i<10; i+=2){
    printf("\nCheck: shiftX = %i\n",i);
    error = compareSubImg(Y_c, Y_c, &field, md->fi.width, md->fi.height,
													1, i, 0, INT_MAX);
    fprintf(stderr,"mismatch %i: %i\n", i, error);
  }
  return 1;
}



int test_transform_implementation(){
  DSFrameInfo fi;
  unsigned char* src;
  testImageStripeYUV(128,&fi,&src);
  unsigned char* dest= (unsigned char*)malloc(fi.framesize);
  unsigned char* cfinal = (unsigned char*)malloc(fi.framesize);
  TransformData td;
  fprintf(stderr,"--- Validate Interpolations ----\n");

  int it;
  int i;
  int sum;
  Transform t;
  t.x = 10;
  t.alpha = 2*M_PI/(180.0);

  for(it=Zero; it<=BiCubic; it++){
    memcpy(dest, src, fi.framesize);
    assert(initTransformData(&td, &fi, &fi, "test") == DS_OK);
    td.interpolType=it;
    assert(configureTransformData(&td)== DS_OK);

    fprintf(stderr,"Transform: %s\n", interpolTypes[it]);
    assert(transformPrepare(&td,dest,dest)== DS_OK);
    assert(transformYUV_float(&td, t)== DS_OK);

    memcpy(cfinal,td.dest,fi.framesize);

    memcpy(dest, src, fi.framesize);
    assert(initTransformData(&td, &fi, &fi, "test") == DS_OK);
    td.interpolType=it;
    assert(configureTransformData(&td)== DS_OK);
    assert(transformPrepare(&td,dest,dest)== DS_OK);
    assert(transformYUV(&td, t)== DS_OK);

    // validate
    sum=0;
    for(i=0; i<fi.framesize; i++){
      if(abs(cfinal[i] - td.dest[i])>2){
				sum+=abs(cfinal[i] - td.dest[i]);
				printf("%i,%i: %i\n", i/fi.width, i%fi.width ,cfinal[i] - td.dest[i]);
      }
    }
    printf("***Difference: %i\n", sum);
  }
	free(dest);
	free(cfinal);
	free(src);
  return sum;
}

typedef unsigned int (*cmpSubImgFunc)(unsigned char* const I1, unsigned char* const I2,
																			const Field* field,
																			int width, int height, int bytesPerPixel,
																			int d_x, int d_y, unsigned int threshold);

// runs the compareSubImg routine and returns the time and stores the difference.
//  if diffsRef is given than the results are validated
int runcompare( cmpSubImgFunc cmpsubfunc,
								unsigned char* frame1, unsigned char* frame2, Field f,
								DSFrameInfo fi, int* diffs, int* diffsRef, int numruns){
  int start = timeOfDayinMS();
  int i;
  for(i=0; i<numruns; i++){
    diffs[i]=cmpsubfunc(frame1, frame2,
												&f, fi.width, fi.height, 2, i%200, i/200, INT_MAX);
  }
  int end = timeOfDayinMS();
  if(diffsRef)
    for(i=0; i<numruns; i++){
      if(diffs[i]!=diffsRef[i]){
				fprintf(stderr, "ERROR! Ref difference %i, Opt difference %i\n",
								diffsRef[i], diffs[i]);
      }
    }
  return end-start;
}

// runs the boxblur routine and returns the time
int runboxblur( unsigned char* frame1, unsigned char* dest,
								DSFrameInfo fi, int numruns){
  int start = timeOfDayinMS();
  int i;
  boxblurYUV(dest, frame1, 0, &fi, 15, BoxBlurColor);
  for(i=1; i<numruns; i++){
    boxblurYUV(dest, dest, 0, &fi, 15, BoxBlurColor);
  }
  int end = timeOfDayinMS();
  return end-start;
}

#ifdef USE_OMP
int openmptest(){
  int start = timeOfDayinMS();
  long int sum=0;
  int i,j;

#pragma omp parallel for shared(sum)
  for (i=0; i<10;i++){
    printf("num theads: %i\n",omp_get_thread_num());
    long int k=0;
    for (j=0; j<40000;j++){
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
	fprintf(stderr, "Processors: %i, Max # theads: %i\n", omp_get_num_procs(), omp_get_max_threads());

	int time, timeref;
	omp_set_dynamic( 0 );
	omp_set_num_threads( 1 );
	fprintf(stderr,"********** omp speedtest:\n");
	time = openmptest();
	fprintf(stderr,"***C    time: %i ms\n",  time);
	timeref=time;
	omp_set_dynamic( 0 );
	omp_set_num_threads( 2 );
	time = openmptest();
	fprintf(stderr,"***C (2)time: %i ms, Speedup %f\n", time,
					(double)timeref/time);
	omp_set_dynamic( 1 );
	return 1;
}
#endif


int main(int argc, char** argv){

  MotionDetect md;
  DSFrameInfo fi;
  fi.width=1280;
  fi.height=720;
  fi.strive=1280;
  fi.framesize=1382400;
  fi.pFormat = PF_YUV;
  DSFrameInfo fi_color;
  fi_color.width=640;
  fi_color.height=360;
  fi_color.strive=640 ;

#ifdef USE_OMP
  openmp();
#endif

  assert(initMotionDetect(&md, &fi, "test") == DS_OK);

  TransformData td;
  assert(initTransformData(&td, &fi, &fi, "test") == DS_OK);

  int i;
  unsigned char* frames[5];
  if(loadTestFrames){
    FILE* file;
    char name[128];

    for(i=0; i<5; i++){
      frames[i] = (unsigned char*)malloc(fi.framesize);
      sprintf(name,"../frames/frame%03i.raw",i+4);
      fprintf(stderr, "load file %s\n", name);
      file = fopen(name,"rb");
      assert(file!=0);
      fprintf(stderr,"read %li bytes\n",
							(unsigned long)fread(frames[i], 1, fi.framesize,file));
      fclose(file);
    }
  }else{
    generateFrames(frames, 5, fi);
  }
  storePGMImage("test1.pgm", frames[0], fi);
  storePGMImage("test2.pgm", frames[1], fi);
  storePGMImage("test3.pgm", frames[2], fi);
  storePGMImage("test4.pgm", frames[3], fi);
  storePGMImage("test5.pgm", frames[4], fi);



  md.shakiness=6;
  md.accuracy=12;

  fflush(stdout);
  printf("\n");
  if(test_checkCompareImg){
		assert(configureMotionDetect(&md)== DS_OK);
    checkCompareImg(&md,frames[0]);
		cleanupMotionDetection(&md);
  }

  if(test_motionDetect){
    assert(initMotionDetect(&md, &fi, "test") == DS_OK);
    assert(configureMotionDetect(&md)== DS_OK);
    fprintf(stderr,"MotionDetect:\n");
    int numruns =5;
    //int t;
    //        for(t = 1; t <= 4; t++){
    int start = timeOfDayinMS();
    //      omp_set_dynamic( 0 );
    //      omp_set_num_threads( t );

    for(i=0; i<numruns; i++){
			LocalMotions localmotions;
			Transform t;
      assert(motionDetection(&md, &localmotions,frames[i])== DS_OK);
			store_localmotions(stderr,&localmotions);
			FILE* f = fopen("lmtest","w");
			store_localmotions(f,&localmotions);
			fclose(f);
			f = fopen("lmtest","r");
			LocalMotions test = restore_localmotions(f);
			fclose(f);
			store_localmotions(stderr,&test);


			/* for(k=0; k < ds_vector_size(&localmotions); k++){ */
			/* 	localmotion_print(LMGet(&localmotions,k),stderr); */
			/* } */
			t = simpleMotionsToTransform(&md, &localmotions);

			ds_vector_del(&localmotions);
      fprintf(stderr, "%i %6.4lf %6.4lf %8.5lf %6.4lf %i\n",
							i, t.x, t.y, t.alpha, t.zoom, t.extra);
    }
    int end = timeOfDayinMS();

    //    fprintf(stderr,"\n*** elapsed time for %i runs: (%i theads) %i ms ****\n", numruns, t, end-start );
    fprintf(stderr,"\n*** elapsed time for %i runs: %i ms ****\n", numruns, end-start );
    // }
    cleanupMotionDetection(&md);
  }


  assert(test_transform_implementation()<1);
  if(test_transform){

    fprintf(stderr,"--- Performance of Transforms ----\n");
    unsigned char* dest = (unsigned char*)malloc(fi.framesize);
    unsigned char* cfinal = (unsigned char*)malloc(fi.framesize);
    int it;
    int start, numruns;
    int timeC, timeCFP; //, timeOrc;
    numruns = 5;

    for(it=Zero; it<=BiCubic; it++){
      assert(initTransformData(&td, &fi, &fi, "test") == DS_OK);
      td.interpolType=it;
      assert(configureTransformData(&td)== DS_OK);

      fprintf(stderr,"Transform: %s", interpolTypes[it]);
      start = timeOfDayinMS();
      for(i=0; i<numruns; i++){
				Transform t = null_transform();
				t.x = i*10+10;
				t.alpha = (i+1)*2*M_PI/(180.0);
				t.zoom = 0;
				memcpy(dest, frames[0], fi.framesize);
				assert(transformPrepare(&td,dest,dest)== DS_OK);
				assert(transformYUV_float(&td, t)== DS_OK);
      }
      timeC = timeOfDayinMS() - start;
      fprintf(stderr,"\n***C   elapsed time for %i runs: %i ms ****\n",
							numruns, timeC );

      if(it==BiLinear){
				storePGMImage("transformed.pgm", td.dest, fi);
				storePGMImage("transformed_u.pgm", td.dest+fi.width*fi.height, fi_color);
				fprintf(stderr,"stored transformed.pgm\n");
      }
      memcpy(cfinal,td.dest,fi.framesize);

      assert(initTransformData(&td, &fi, &fi, "test") == DS_OK);
      td.interpolType=it;
      assert(configureTransformData(&td)== DS_OK);
      start = timeOfDayinMS();
      for(i=0; i<numruns; i++){
				Transform t = null_transform();
				t.x = i*10+10;
				t.alpha = (i+1)*2*M_PI/(180.0);
				t.zoom = 0;
				memcpy(dest, frames[0], fi.framesize);
				assert(transformPrepare(&td,dest,dest)== DS_OK);
				assert(transformYUV(&td, t)== DS_OK);
      }
      timeCFP = timeOfDayinMS() - start;
      fprintf(stderr,"***FP  elapsed time for %i runs: %i ms ****\n",
							numruns, timeCFP );
      if(it==BiLinear){
				storePGMImage("transformed_FP.pgm", td.dest, fi);
				storePGMImage("transformed_u_FP.pgm", td.dest+fi.width*fi.height, fi_color);
				fprintf(stderr,"stored transformed_FP.pgm\n");
      }
      fprintf(stderr,"***Speedup %3.2f\n", (double)timeC/timeCFP);
      // validate
      int sum=0;
      for(i=0; i<fi.framesize; i++){
				if(abs(cfinal[i] - td.dest[i])>2){
					sum+=cfinal[i] - td.dest[i];
					//printf("%i,%i: %i\n", i/fi.width, i%fi.width ,cfinal[i] - td.dest[i]);
				}
      }
      printf("***Difference: %i\n", sum);
    }

		free(dest);
		free(cfinal);
  }

  if(test_boxblur){
    int time; //, timeref;
    int numruns=2;
    unsigned char* dest = (unsigned char*)malloc(fi.framesize);
    //    omp_set_dynamic( 0 );
    //    omp_set_num_threads( 1 );
    fprintf(stderr,"********** boxblur speedtest:\n");
    time = runboxblur(frames[4], dest, fi, numruns);
    fprintf(stderr,"***C    time for %i runs: %i ms\n", numruns, time);
    storePGMImage("boxblured.pgm", dest, fi);
    storePGMImage("orig4.pgm", frames[4], fi);
    // timeref=time;
    /* omp_set_dynamic( 0 ); */
    /* omp_set_num_threads( 2); */
    /* time = runboxblur(frames[4], dest, fi, numruns); */
    /* fprintf(stderr,"***C (2)time for %i runs: %i ms, Speedup %f\n", numruns, time, */
    /* 	    (double)timeref/time); */
    /* omp_set_dynamic( 1 ); */
		free(dest);
  }
  if(test_compareImg){
    Field f;
    f.size=128;
    f.x = 400;
    f.y = 300;
    fprintf(stderr,"********** Compare speedtest:\n");

    int numruns = num;
    int diffsC[numruns];
    int diffsO[numruns];
    int timeC, timeO;
    timeC=runcompare(compareSubImg_thr, frames[0], frames[1], f, fi, diffsC, 0, numruns);
    fprintf(stderr,"***C        time for %i runs: %i ms ****\n", numruns, timeC);
#ifdef USE_ORC
    timeO=runcompare(compareSubImg_orc, frames[0], frames[1], f, fi, diffsO, diffsC, numruns);
    fprintf(stderr,"***orc      time for %i runs: %i ms \tSpeedup %3.2f\n",
						numruns, timeO, (double)timeC/timeO);
    timeO=runcompare(compareSubImg_thr_orc, frames[0], frames[1], f, fi, diffsO, diffsC, numruns);
    fprintf(stderr,"***thr_orc  time for %i runs: %i ms \tSpeedup %3.2f\n",
						numruns, timeO, (double)timeC/timeO);
#endif
#ifdef USE_SSE2
    timeO=runcompare(compareSubImg_thr_sse2, frames[0], frames[1], f, fi, diffsO, diffsC, numruns);
    fprintf(stderr,"***thr_sse2 time for %i runs: %i ms \tSpeedup %3.2f\n",
						numruns, timeO, (double)timeC/timeO);
#endif
#ifdef USE_SSE2_ASM
    timeO=runcompare(compareSubImg_thr_sse2_asm, frames[0], frames[1], f, fi, diffsO, diffsC, numruns);
    fprintf(stderr,"***thr_asm  time for %i runs: %i ms \tSpeedup %3.2f\n",
						numruns, timeO, (double)timeC/timeO);
#endif
  }
  if(test_contrastImg){
    Field f;
    // difference between michelson and absolute differences from mean
    //  is large for 100x100 at 500,300
    f.size=128;
    f.x = 400;
    f.y = 300;
    fprintf(stderr,"********** Contrast:\n");
    int numruns = num;
    double contrastC[numruns];
    double contrastOpt[numruns];
    int timeC, timeOpt;
#ifdef USE_ORC
    fprintf(stderr,"********** Variance - based Contrast (with ORC):\n");
    {
      int start = timeOfDayinMS();
      for(i=0; i<numruns; i++){
				contrastC[i]=contrastSubImg_variance_C(frames[0], &f, fi.width, fi.height);
      }
      int end = timeOfDayinMS();
      timeC=end-start;
      fprintf(stderr,"***C    time for %i runs: %i ms ****\n", numruns, timeC);
    }
    {
      int start = timeOfDayinMS();
      for(i=0; i<numruns; i++){
				contrastOpt[i]=contrastSubImg_variance_orc(frames[0], &f, fi.width, fi.height);
      }
      int end = timeOfDayinMS();
      timeOpt=end-start;
      fprintf(stderr,"***Orc  time for %i runs: %i ms ****\n", numruns, timeOpt);
    }
    fprintf(stderr,"***Speedup %3.2f\n", timeC/timeOpt);
    for(i=0; i<numruns; i++){
      if(i==0){
				printf("Orc contrast %3.2f, C contrast %3.2f\n",contrastOpt[i], contrastC[i]);
      }
      assert(contrastC[i]==contrastOpt[i]);
    }
#endif
    fprintf(stderr,"********** Michelson Contrast (with SSE2):\n");
    {
      int start = timeOfDayinMS();
      for(i=0; i<numruns; i++){
				contrastC[i]=contrastSubImg(frames[0], &f, fi.width, fi.height,1);
      }
      int end = timeOfDayinMS();
      timeC=end-start;
      fprintf(stderr,"***C    time for %i runs: %i ms ****\n", numruns, timeC);
    }
#ifdef USE_SSE2
    {
      int start = timeOfDayinMS();
      for(i=0; i<numruns; i++){
				contrastOpt[i]=contrastSubImg1_SSE(frames[0], &f, fi.width, fi.height);
      }
      int end = timeOfDayinMS();
      timeOpt=end-start;
      fprintf(stderr,"***SSE2 time for %i runs: %i ms ****\n", numruns, timeOpt);
    }
    fprintf(stderr,"***Speedup %3.2f\n", (float)timeC/(float)timeOpt);
    for(i=0; i<numruns; i++){
      if(i==0){
				printf("SSE2 contrast %3.2f, C contrast %3.2f\n",contrastOpt[i], contrastC[i]);
      }
      assert(contrastC[i]==contrastOpt[i]);
    }
#endif
  }


  return 1;
}
