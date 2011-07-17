#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include "orc_bug_orc.h"

// cd ../ && orcc --implementation -o orc_bug_orc.c orc_bug_orc.orc && orcc --header -o orc_bug_orc.h orc_bug_orc.orc ; cd cmake

int main(){
  int x;
  int N = 512;

  // some random parameters
  int32_t zcos_a = 12345;
  int32_t zsin_a = 65432;
  int32_t c_tx   = 250;
  int32_t c_ty   = 6;
  int32_t c_d_x  = 256;
  int32_t y_d1 = 100;
  
  int32_t* x_ss = (int32_t*)malloc(sizeof(int32_t)*N);
  int32_t* y_ss = (int32_t*)malloc(sizeof(int32_t)*N);
  int32_t* xs   = (int32_t*)malloc(sizeof(int32_t)*N);        
  for (x = 0; x < N; x++) { 
    xs[x]=x;
  }
  
  test_orc (x_ss, y_ss, xs, 
	    y_d1, c_d_x, c_tx, c_ty, zcos_a, zsin_a, N);
  
  return 0;
}
