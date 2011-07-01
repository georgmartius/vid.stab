
#include <stdio.h>
#include <assert.h>
#include <sys/time.h>
#include <stdint.h>

#include "transform.h"
#include "dslist.h"
#include "libdeshake.h"

#include "testutils.c"

int num=2000;
int test_checkCompareImg=0;
int test_motionDetect=0;
int test_transform=1;
int test_compareImg=1;
int test_contrastImg=1;

struct iterdata {
    FILE *f;
    int  counter;
};

static int dump_trans(DSListItem *item, void *userdata)
{
  struct iterdata *ID = (struct iterdata *)userdata;
  
  if (item->data) {
    Transform* t = (Transform*)item->data;
    fprintf(ID->f, "%i %6.4lf %6.4lf %8.5lf %6.4lf %i\n",
	    ID->counter, t->x, t->y, t->alpha, t->zoom, t->extra);
    ID->counter++;
  }
  return 0; /* never give up */
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
			  1, i, 0);
    fprintf(stderr,"mismatch %i: %i\n", i, error);
  }
  return 1;
}

int main(int argc, char** argv){

  MotionDetect md;
  DSFrameInfo fi;
  fi.width=1280;
  fi.height=720;
  fi.strive=1280;
  fi.framesize=1382400;
  fi.pFormat = PF_YUV;
  assert(initMotionDetect(&md, &fi, "test") == DS_OK);

  TransformData td;
  assert(initTransformData(&td, &fi, &fi, "test") == DS_OK);  

  unsigned char* frames[5];
  FILE* file;
  char name[128];
  int i;
  
  for(i=0; i<5; i++){
    frames[i] = (unsigned char*)malloc(fi.framesize);
    sprintf(name,"../frames/frame%003i.raw",i+4);
    fprintf(stderr, "load file %s\n", name);
    file = fopen(name,"rb");
    assert(file!=0);
    fprintf(stderr,"read %i bytes\n", fread(frames[i], 1, fi.framesize,file));
    fclose(file);    
  }
  
  md.shakiness=6;
  assert(configureMotionDetect(&md)== DS_OK);

  fflush(stdout);
  printf("\n");
  if(test_checkCompareImg){
    checkCompareImg(&md,frames[0]);
  }
  
  if(test_motionDetect){
    fprintf(stderr,"MotionDetect:");
    int start = timeOfDayinMS();
    int numruns =3;
    for(i=0; i<numruns; i++){
      assert(motionDetection(&md, frames[i])== DS_OK);
    }
    int end = timeOfDayinMS();
    

    struct iterdata ID;
    ID.counter = 0;
    ID.f       = stdout;
    ds_list_foreach(md.transs, dump_trans, &ID);
    fprintf(stderr,"\n*** elapsed time for %i runs: %i ms ****\n\n", numruns, end-start );
  }

  if(test_transform){
    unsigned char* dest = (unsigned char*)malloc(fi.framesize);
    int start, numruns;
    int timeC, timeCFP, timeOrc;
    numruns =3;

    assert(initTransformData(&td, &fi, &fi, "test") == DS_OK);  
    memcpy(dest, frames[0], fi.framesize);
    assert(configureTransformData(&td)== DS_OK);
    
    fprintf(stderr,"Transform:");
    start = timeOfDayinMS();
    for(i=0; i<numruns; i++){
      Transform t = null_transform();
      t.x = i*10+10;
      t.alpha = (i+1)*2*M_PI/(180.0);
      assert(transformPrepare(&td,dest)== DS_OK);
      assert(transformYUV(&td, t)== DS_OK);      
    }
    timeC = timeOfDayinMS() - start;  
    fprintf(stderr,"\n***C   elapsed time for %i runs: %i ms ****\n", 
	    numruns, timeC );    

    storePGMImage("transformed.pgm", td.dest, fi);
    fprintf(stderr,"stored transformed.pgm\n");

    assert(initTransformData(&td, &fi, &fi, "test") == DS_OK);  
    memcpy(dest, frames[0], fi.framesize);
    assert(configureTransformData(&td)== DS_OK);
    start = timeOfDayinMS();
    for(i=0; i<numruns; i++){
      Transform t = null_transform();
      t.x = i*10+10;
      t.alpha = (i+1)*2*M_PI/(180.0);
      assert(transformPrepare(&td,dest)== DS_OK);
      assert(transformYUVFP(&td, t)== DS_OK);      
    }
    timeCFP = timeOfDayinMS() - start;  
    fprintf(stderr,"\n***FP  elapsed time for %i runs: %i ms ****\n", 
	    numruns, timeCFP );

    storePGMImage("transformed_FP.pgm", td.dest, fi);
    fprintf(stderr,"stored transformed_FP.pgm\n");
    fprintf(stderr,"***Speedup %3.1f\n", (double)timeC/timeCFP);      

  }
  
  if(test_compareImg){
    Field f;
    f.size=144;
    f.x = 400;
    f.y = 300;
    fprintf(stderr,"********** orc Compare:\n");

    int numruns =num;
    int diffsC[numruns];
    int diffsOrc[numruns];
    int timeC, timeOrc;
    {
      int start = timeOfDayinMS();
      for(i=0; i<numruns; i++){
	diffsC[i]=compareSubImg_C(frames[0], frames[1], 
				&f, fi.width, fi.height, 2, i%200, i/200);
      }
      int end = timeOfDayinMS();   
      timeC=end-start;
      fprintf(stderr,"***C   time for %i runs: %i ms ****\n", numruns, timeC);
    }
    {
      int start = timeOfDayinMS();
      for(i=0; i<numruns; i++){
	diffsOrc[i]=compareSubImg(frames[0], frames[1], &f, 
				  fi.width, fi.height, 2, i%200, i/200);
      }
      int end = timeOfDayinMS();   
      timeOrc=end-start;
      fprintf(stderr,"***Orc time for %i runs: %i ms ****\n", numruns, timeOrc);      
    }
    fprintf(stderr,"***Speedup %3.1f\n", (double)timeC/timeOrc);      
    for(i=0; i<numruns; i++){
      if(i==0){
	fprintf(stderr, "Orc difference %i, C difference %i\n",diffsOrc[i], diffsC[i]); 
      }
      assert(diffsC[i]==diffsOrc[i]);
    }
  }
  if(test_contrastImg){
    Field f;
    // difference between michelson and absolute differences from mean 
    //  is large for 100x100 at 500,300
    f.size=144;
    f.x = 400;
    f.y = 300;
    fprintf(stderr,"********** orc Contrast:\n");
    int numruns = num;
    double contrastC[numruns];
    double contrastOrc[numruns];
    int timeC, timeOrc;
    {
      int start = timeOfDayinMS();
      for(i=0; i<numruns; i++){
	contrastC[i]=contrastSubImg_C(frames[0], &f, fi.width, fi.height);
      }
      int end = timeOfDayinMS();   
      timeC=end-start;
      fprintf(stderr,"***C   time for %i runs: %i ms ****\n", numruns, timeC);
    }
    {
      int start = timeOfDayinMS();
      for(i=0; i<numruns; i++){
	contrastOrc[i]=contrastSubImg(frames[0], &f, fi.width, fi.height);
      }
      int end = timeOfDayinMS();   
      timeOrc=end-start;
      fprintf(stderr,"***Orc time for %i runs: %i ms ****\n", numruns, timeOrc);      
    }
    double timeCD = timeC;
    fprintf(stderr,"***Speedup %3.1f\n", timeCD/timeOrc);      
    for(i=0; i<numruns; i++){
      if(i==0){
	printf("Orc contrast %3.2f, C contrast %3.2f\n",contrastOrc[i], contrastC[i]); 
      }
      assert(contrastC[i]==contrastOrc[i]);
    }



  }


  return 1;
}
