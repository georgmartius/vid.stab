#ifndef __TESTUTILS_H
#define __TESTUTILS_H

#include "libvidstab.h"

typedef struct _test_data {
  DSFrameInfo fi;
  DSFrameInfo fi_color;
  DSFrame frames[5];
} TestData;


Transform getTestFrameTransform(int i);

void fillArrayWithNoise(unsigned char* buffer, int length, float corr);

void paintRectangle(unsigned char* buffer, const DSFrameInfo* fi, int x, int y,
										int sizex, int sizey, unsigned char color);

inline static unsigned char randPixel(){
  return rand()%256;
}

inline static short randUpTo(short max){
  return rand()%max;
}


int loadPGMImage(const char* filename, DSFrame* frame, DSFrameInfo* fi);

int storePGMImage(const char* filename, const uint8_t* data, DSFrameInfo fi );

#endif
