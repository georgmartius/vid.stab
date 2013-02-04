#ifndef __TESTUTILS_H
#define __TESTUTILS_H

#include "libdeshake.h"

typedef struct _test_data {
  DSFrameInfo fi;
  DSFrameInfo fi_color;
  unsigned char* frames[5];
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


int loadPGMImage(const char* filename, char ** framebuffer, DSFrameInfo* fi);

int storePGMImage(const char* filename, unsigned char * image_data, DSFrameInfo fi );

#endif
