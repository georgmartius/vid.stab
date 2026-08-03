#ifndef __TESTUTILS_H
#define __TESTUTILS_H

#include "libvidstab.h"

typedef struct _test_data {
  VSFrameInfo fi;
  VSFrameInfo fi_color;
  VSFrame frames[5];
} TestData;


VSTransform getTestFrameTransform(int i);

void fillArrayWithNoise(unsigned char* buffer, int length, float corr);

void paintRectangle(unsigned char* buffer, const VSFrameInfo* fi, int x, int y,
                    int sizex, int sizey, unsigned char color);

void fillFrameRGB(VSFrame* frame, const VSFrameInfo* fi, uint8_t r, uint8_t g, uint8_t b);

void setPixelRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y,
                 uint8_t r, uint8_t g, uint8_t b);

void getPixelRGB(const VSFrame* frame, const VSFrameInfo* fi, int x, int y,
                 uint8_t* r, uint8_t* g, uint8_t* b);

void paintCircleRGB(VSFrame* frame, const VSFrameInfo* fi, int cx, int cy, int radius,
                    uint8_t r, uint8_t g, uint8_t b);

void paintSquareRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y, int size,
                    uint8_t r, uint8_t g, uint8_t b);

int storePPMImage(const char* filename, const VSFrame* frame, const VSFrameInfo* fi);

inline static unsigned char randPixel(){
  return rand()%256;
}

inline static short randUpTo(short max){
  return rand()%max;
}


int loadPGMImage(const char* filename, VSFrame* frame, VSFrameInfo* fi);

int storePGMImage(const char* filename, const uint8_t* data, VSFrameInfo fi );

#endif
