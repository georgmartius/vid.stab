/* internal to boxblur.c, not part of boxblur.h */
void boxblur_vert_C(unsigned char* dest, const unsigned char* src,
                    int width, int height, int dest_strive, int src_strive, int size);
void boxblur_hori_C(unsigned char* dest, const unsigned char* src,
                    int width, int height, int dest_strive, int src_strive, int size);

/* Straightforward column-at-a-time vertical pass, kept as the reference the
   row-major boxblur_vert_C must reproduce bit for bit. */
static void boxblur_vert_reference(unsigned char* dest, const unsigned char* src,
                                   int width, int height, int dest_strive,
                                   int src_strive, int size){
  int i,j,k;
  int acc;
  const unsigned char *start, *end;
  unsigned char *current;
  int size2 = size/2;
  for(i=0; i< width; i++){
    start = end = src + i;
    current = dest + i;
    acc= (*start)*(size2+1);
    for(k=0; k<size2; k++){
      acc+=(*end);
      end+=src_strive;
    }
    for(j=0; j< height; j++){
      acc = acc - (*start) + (*end);
      if(j > size2) start+=src_strive;
      if(j < height - size2 - 1) end+=src_strive;
      *current = acc/size;
      current+=dest_strive;
    }
  }
}

static void boxblur_hori_reference(unsigned char* dest, const unsigned char* src,
                                   int width, int height, int dest_strive,
                                   int src_strive, int size){
  int i,j,k;
  unsigned int acc;
  const unsigned char *start, *end;
  unsigned char *current;
  int size2 = size/2;
  for(j=0; j< height; j++){
    start = end = src + j*src_strive;
    current = dest + j*dest_strive;
    acc= (*start)*(size2+1);
    for(k=0; k<size2; k++){
      acc+=(*end);
      end++;
    }
    for(i=0; i< width; i++){
      acc = acc + (*end) - (*start);
      if(i > size2) start++;
      if(i < width - size2 - 1) end++;
      (*current) = acc/size;
      current++;
    }
  }
}

/* Bit-exact equivalence of both passes with the reference implementations
   above, over a spread of sizes and geometries: odd and even widths, widths
   that are not a multiple of any vector width, strides wider than the image,
   and the smallest heights the kernel size still permits.

   The sizes deliberately straddle 265/267, where the magic-number reciprocal
   that replaced `acc/size` stops being provably exact and both passes fall
   back to a real division -- so both code paths are covered here. */
static void test_boxblur_vert_equivalence(void){
  static const int widths[]  = { 1, 2, 7, 16, 17, 31, 64, 100, 127, 256 };
  static const int heights[] = { 4, 5, 16, 33, 64, 127, 560 };
  static const int sizes[]   = { 3, 5, 9, 15, 31, 265, 267 };
  int wi, hi, si, pad, i;

  fprintf(stderr,"*** boxblur_{vert,hori}_C must be bit identical to the original\n");
  for(wi=0; wi<(int)(sizeof(widths)/sizeof(widths[0])); wi++){
    for(hi=0; hi<(int)(sizeof(heights)/sizeof(heights[0])); hi++){
      for(si=0; si<(int)(sizeof(sizes)/sizeof(sizes[0])); si++){
        for(pad=0; pad<2; pad++){        /* stride == width, and stride > width */
          int w = widths[wi], h = heights[hi], size = sizes[si];
          int stride = w + (pad ? 13 : 0);
          unsigned char *src, *d1, *d2;

          /* the caller (boxblurPlanar) clamps size this way; respect it so we
             only test geometries that can actually occur -- and so that the
             kernel never walks off the end of a row/column */
          if(size/2 >= h && size/2 >= w) continue;

          src = (unsigned char*)malloc((size_t)stride*h);
          d1  = (unsigned char*)malloc((size_t)stride*h);
          d2  = (unsigned char*)malloc((size_t)stride*h);
          for(i=0; i<stride*h; i++)
            src[i] = (unsigned char)((i*37 + i/stride*11 + (i%stride)*7) & 0xFF);

          if(size/2 < h){
            int differs = 0;
            memset(d1, 0xAA, (size_t)stride*h);
            memset(d2, 0xAA, (size_t)stride*h);
            boxblur_vert_reference(d1, src, w, h, stride, stride, size);
            boxblur_vert_C        (d2, src, w, h, stride, stride, size);
            for(i=0; i<h && !differs; i++)
              if(memcmp(d1 + (size_t)i*stride, d2 + (size_t)i*stride, w) != 0)
                differs = 1;
            if(differs)
              fprintf(stderr,"  BOXBLUR VERT MISMATCH w=%i h=%i size=%i stride=%i\n",
                      w, h, size, stride);
            test_bool(!differs);
          }
          if(size/2 < w){
            int differs = 0;
            memset(d1, 0xAA, (size_t)stride*h);
            memset(d2, 0xAA, (size_t)stride*h);
            boxblur_hori_reference(d1, src, w, h, stride, stride, size);
            boxblur_hori_C        (d2, src, w, h, stride, stride, size);
            for(i=0; i<h && !differs; i++)
              if(memcmp(d1 + (size_t)i*stride, d2 + (size_t)i*stride, w) != 0)
                differs = 1;
            if(differs)
              fprintf(stderr,"  BOXBLUR HORI MISMATCH w=%i h=%i size=%i stride=%i\n",
                      w, h, size, stride);
            test_bool(!differs);
          }

          free(src); free(d1); free(d2);
        }
      }
    }
  }
}

// runs the boxblur routine and returns the time
int runboxblur( VSFrame frame1, VSFrame dest,
                VSFrameInfo fi, int numruns){
  int start = timeOfDayinMS();
  int i;
  boxblurPlanar(&dest, &frame1, 0, &fi, 15, BoxBlurColor);
  for(i=1; i<numruns; i++){
    boxblurPlanar(&dest, &dest, 0, &fi, 15, BoxBlurColor);
  }
  int end = timeOfDayinMS();
  return end-start;
}


void test_boxblur(const TestData* testdata){
  int time; //, timeref;
  int numruns=2;
  VSFrame dest;
  vsFrameAllocate(&dest,&testdata->fi);
  //    omp_set_dynamic( 0 );
  //    omp_set_num_threads( 1 );
  fprintf(stderr,"********** boxblur:\n");
  test_boxblur_vert_equivalence();
  time = runboxblur(testdata->frames[4], dest, testdata->fi, numruns);
  fprintf(stderr,"***C    time for %i runs: %i ms\n", numruns, time);
  storePGMImage(testOut("boxblured.pgm"), dest.data[0], testdata->fi);
  storePGMImage(testOut("orig4.pgm"), testdata->frames[4].data[0], testdata->fi);
  // timeref=time;
  /* omp_set_dynamic( 0 ); */
  /* omp_set_num_threads( 2); */
  /* time = runboxblur(testdata->frames[4], dest, testdata->fi, numruns); */
  /* fprintf(stderr,"***C (2)time for %i runs: %i ms, Speedup %f\n", numruns, time, */
  /*       (double)timeref/time); */
  /* omp_set_dynamic( 1 ); */
  vsFrameFree(&dest);
}
