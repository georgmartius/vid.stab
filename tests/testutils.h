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

/** directory every file the test suite writes goes into, relative to the
    working directory the tests are run from, and gitignored */
#define TEST_OUTPUT_DIR "testout"

/** Builds TEST_OUTPUT_DIR "/" name and makes sure the directory it lives in
    exists; name may contain subdirectories.  Use it for every path the tests
    write to, so that a test run leaves nothing behind outside of
    TEST_OUTPUT_DIR.

    The result lives in a small ring of static buffers, so a few results can be
    in flight at once -- two calls in the same argument list are fine -- but the
    pointer must not be stored for later use. */
const char* testOut(const char* name);

#endif
