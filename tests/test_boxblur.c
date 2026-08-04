/* internal to boxblur.c, not part of boxblur.h */
void boxblur_vert_C(unsigned char* dest, const unsigned char* src,
                    int width, int height, int dest_strive, int src_strive, int size);

/* Reference implementations: the original column-at-a-time vertical pass and
   the horizontal pass, transcribed verbatim from boxblur.c before the vertical
   pass was turned inside out for cache locality and vectorization.  The
   rewrite is required to be bit identical, so these are what it is checked
   against -- any difference at all is a regression. */
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

/* Bit-exact equivalence of boxblur_vert_C with the original column walk, over
   a spread of sizes and geometries: odd and even widths, widths that are not a
   multiple of any vector width, strides wider than the image, and the smallest
   heights the kernel size still permits. */
static void test_boxblur_vert_equivalence(void){
  static const int widths[]  = { 1, 2, 7, 16, 17, 31, 64, 100, 127, 256 };
  static const int heights[] = { 4, 5, 16, 33, 64, 127 };
  static const int sizes[]   = { 3, 5, 9, 15, 31 };
  int wi, hi, si, pad, i;

  fprintf(stderr,"*** boxblur_vert_C must be bit identical to the column walk\n");
  for(wi=0; wi<(int)(sizeof(widths)/sizeof(widths[0])); wi++){
    for(hi=0; hi<(int)(sizeof(heights)/sizeof(heights[0])); hi++){
      for(si=0; si<(int)(sizeof(sizes)/sizeof(sizes[0])); si++){
        for(pad=0; pad<2; pad++){        /* stride == width, and stride > width */
          int w = widths[wi], h = heights[hi], size = sizes[si];
          int stride = w + (pad ? 13 : 0);
          unsigned char *src, *d1, *d2;
          int differs = 0;

          /* the caller (boxblurPlanar) clamps size this way; respect it so we
             only test geometries that can actually occur */
          if(size/2 >= h) continue;

          src = (unsigned char*)malloc((size_t)stride*h);
          d1  = (unsigned char*)malloc((size_t)stride*h);
          d2  = (unsigned char*)malloc((size_t)stride*h);
          for(i=0; i<stride*h; i++)
            src[i] = (unsigned char)((i*37 + i/stride*11 + (i%stride)*7) & 0xFF);
          memset(d1, 0xAA, (size_t)stride*h);
          memset(d2, 0xAA, (size_t)stride*h);

          boxblur_vert_reference(d1, src, w, h, stride, stride, size);
          boxblur_vert_C        (d2, src, w, h, stride, stride, size);

          for(i=0; i<h && !differs; i++)
            if(memcmp(d1 + (size_t)i*stride, d2 + (size_t)i*stride, w) != 0)
              differs = 1;
          if(differs)
            fprintf(stderr,"  BOXBLUR MISMATCH w=%i h=%i size=%i stride=%i\n",
                    w, h, size, stride);
          test_bool(!differs);

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
