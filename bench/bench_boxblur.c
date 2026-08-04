/* bench_boxblur.c -- isolated timing for the two boxblur passes. */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "boxblur.h"
#include "frameinfo.h"

void boxblur_hori_C(unsigned char* dest, const unsigned char* src,
                    int width, int height, int dest_strive, int src_strive, int size);
void boxblur_vert_C(unsigned char* dest, const unsigned char* src,
                    int width, int height, int dest_strive, int src_strive, int size);

static double now_s(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  return ts.tv_sec + ts.tv_nsec*1e-9;
}

int main(void){
  const int W=1920,H=1080,SIZE=15,REPS=100;
  unsigned char *src=malloc((size_t)W*H), *dst=malloc((size_t)W*H);
  double t0,t1; int i;
  for(i=0;i<W*H;i++) src[i]=(unsigned char)(i*37);

  t0=now_s(); for(i=0;i<REPS;i++) boxblur_hori_C(dst,src,W,H,W,W,SIZE); t1=now_s();
  printf("boxblur_hori  %6.2f ms/frame\n",(t1-t0)*1000.0/REPS);
  t0=now_s(); for(i=0;i<REPS;i++) boxblur_vert_C(dst,src,W,H,W,W,SIZE); t1=now_s();
  printf("boxblur_vert  %6.2f ms/frame\n",(t1-t0)*1000.0/REPS);
  return 0;
}
