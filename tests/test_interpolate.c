/*
   test_interpolate.c

   Boundary tests for the interpolation kernels.

   The interesting coordinates are the ones exactly on the last row/column:
   there the "ceil" neighbour of a bi-linear interpolation is one pixel past
   the end of the image. Its interpolation weight is zero, so the *result* is
   correct either way -- only a sanitizer (or a guard page) can see the read.
   That is why these cases survived so long, and why this file is meant to be
   run under ASan/UBSan as well as normally.

   The image buffer is allocated to exactly width*height bytes so that any
   overrun lands in ASan's redzone rather than in slack the allocator handed
   out anyway.

   This file is part of vid.stab video stabilization library
   and distributed under the GNU GPL, see the top of transform.c.
*/

#define IP_W 16
#define IP_H 12

/* iToFp16() is private to transformfixedpoint.c; this is the same conversion,
   written so that it is defined for negative values too */
static fp16 iToFp16_t(int v){ return (fp16)((uint32_t)v << 16); }

/* img[x,y] = x + 16*y, so every pixel is distinct and the expected value of a
   bi-linear sample is easy to state */
static unsigned char* ip_make(void){
  unsigned char* img = (unsigned char*)vs_malloc(IP_W * IP_H);
  for (int y = 0; y < IP_H; y++)
    for (int x = 0; x < IP_W; x++)
      img[x + y*IP_W] = (unsigned char)(x + 16*y);
  return img;
}

void test_interpolate_borders(void){
  unsigned char* img = ip_make();
  uint8_t out;

  fprintf(stderr,"*** float bi-linear on the last row/column\n");
  /* exactly the bottom right pixel: both ceil neighbours are out of range */
  _FLT(interpolateBiLin)(&out, (float)(IP_W-1), (float)(IP_H-1),
                         img, IP_W, IP_W, IP_H, 0);
  test_bool(out == img[(IP_W-1) + (IP_H-1)*IP_W]);

  /* last column, interior row */
  _FLT(interpolateBiLin)(&out, (float)(IP_W-1), 4.0f,
                         img, IP_W, IP_W, IP_H, 0);
  test_bool(out == img[(IP_W-1) + 4*IP_W]);

  /* last row, interior column */
  _FLT(interpolateBiLin)(&out, 4.0f, (float)(IP_H-1),
                         img, IP_W, IP_W, IP_H, 0);
  test_bool(out == img[4 + (IP_H-1)*IP_W]);

  /* halfway along the last row: interpolates between two valid pixels */
  _FLT(interpolateBiLin)(&out, 4.5f, (float)(IP_H-1),
                         img, IP_W, IP_W, IP_H, 0);
  test_bool(out == (img[4 + (IP_H-1)*IP_W] + img[5 + (IP_H-1)*IP_W]) / 2);

  /* an ordinary interior sample must be unaffected by any of this */
  _FLT(interpolateBiLin)(&out, 4.0f, 4.0f, img, IP_W, IP_W, IP_H, 0);
  test_bool(out == img[4 + 4*IP_W]);
  _FLT(interpolateBiLin)(&out, 4.5f, 4.0f, img, IP_W, IP_W, IP_H, 0);
  test_bool(out == (img[4 + 4*IP_W] + img[5 + 4*IP_W]) / 2);

  fprintf(stderr,"*** fixed point bi-linear on the last row/column\n");
  interpolateBiLin(&out, iToFp16_t(IP_W-1), iToFp16_t(IP_H-1),
                   img, IP_W, IP_W, IP_H, 0);
  test_bool(abs((int)out - (int)img[(IP_W-1) + (IP_H-1)*IP_W]) <= 1);
  interpolateBiLin(&out, iToFp16_t(4), iToFp16_t(IP_H-1),
                   img, IP_W, IP_W, IP_H, 0);
  test_bool(abs((int)out - (int)img[4 + (IP_H-1)*IP_W]) <= 1);

  /* negative and past-the-end coordinates: must be well defined, and must not
     left shift a negative value (UBSan) */
  fprintf(stderr,"*** negative / out of range coordinates\n");
  interpolateLin(&out, iToFp16_t(-10), iToFp16_t(3), img, IP_W, IP_W, IP_H, 77);
  test_bool(out == 77);
  interpolateLin(&out, iToFp16_t(IP_W+5), iToFp16_t(3), img, IP_W, IP_W, IP_H, 77);
  test_bool(out == 77);

  /* interpolateBiLinBorder blurs the border pixel outwards over w=10 pixels
     before def takes over completely, so def is only reached beyond that */
  interpolateBiLin(&out, iToFp16_t(-10), iToFp16_t(-10), img, IP_W, IP_W, IP_H, 77);
  test_bool(out == img[0]);                       /* exactly w out: border pixel */
  interpolateBiLin(&out, iToFp16_t(-30), iToFp16_t(-30), img, IP_W, IP_W, IP_H, 77);
  test_bool(out == 77);                           /* well beyond w: def */
  interpolateBiLin(&out, iToFp16_t(IP_W+30), iToFp16_t(IP_H+30),
                   img, IP_W, IP_W, IP_H, 77);
  test_bool(out == 77);

  _FLT(interpolateLin)(&out, -10.0f, 3.0f, img, IP_W, IP_W, IP_H, 77);
  test_bool(out == 77);
  _FLT(interpolateBiLin)(&out, -10.0f, -10.0f, img, IP_W, IP_W, IP_H, 77);
  test_bool(out == 77);

  /* sweep every coordinate on and just past the border, both kernels; the
     point is the memory access pattern, so the assertion is only that the
     call returns a value in range */
  fprintf(stderr,"*** border sweep\n");
  for (int i = -2; i <= IP_W + 1; i++){
    float fx = (float)i;
    _FLT(interpolateBiLin)(&out, fx, (float)(IP_H-1), img, IP_W, IP_W, IP_H, 0);
    _FLT(interpolateBiLin)(&out, fx, 0.0f,            img, IP_W, IP_W, IP_H, 0);
    _FLT(interpolateBiCub)(&out, fx, (float)(IP_H-1), img, IP_W, IP_W, IP_H, 0);
    interpolateBiLin(&out, iToFp16_t(i), iToFp16_t(IP_H-1), img, IP_W, IP_W, IP_H, 0);
    interpolateBiCub(&out, iToFp16_t(i), iToFp16_t(IP_H-1), img, IP_W, IP_W, IP_H, 0);
  }
  for (int i = -2; i <= IP_H + 1; i++){
    float fy = (float)i;
    _FLT(interpolateBiLin)(&out, (float)(IP_W-1), fy, img, IP_W, IP_W, IP_H, 0);
    _FLT(interpolateBiLin)(&out, 0.0f,            fy, img, IP_W, IP_W, IP_H, 0);
    _FLT(interpolateBiCub)(&out, (float)(IP_W-1), fy, img, IP_W, IP_W, IP_H, 0);
    interpolateBiLin(&out, iToFp16_t(IP_W-1), iToFp16_t(i), img, IP_W, IP_W, IP_H, 0);
    interpolateBiCub(&out, iToFp16_t(IP_W-1), iToFp16_t(i), img, IP_W, IP_W, IP_H, 0);
  }
  test_bool(1);  /* reaching here without a sanitizer report is the result */

  vs_free(img);
}
