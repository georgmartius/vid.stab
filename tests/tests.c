
#include <stdio.h>
#include <limits.h>
#include <features.h>

#ifdef USE_OMP
#include <omp.h>
#endif

#include "libdeshake.h"
// load optimized functions
#include "motiondetect_opt.h"
#include "serialize.h"

#ifndef TESTING
#error TESTING must be defined
#endif

#include "testframework.h"
#include "testutils.h"

#include "generate.c"

#include "test_transform.c"
#include "test_compareimg.c"
#include "test_motiondetect.c"
#include "test_store_restore.c"
#include "test_contrast.c"
#include "test_boxblur.c"
#include "test_omp.c"

int main(int argc, char** argv){

	if(contains(argv,argc,"-h", "help")!=0){
		printf("Usage: %s [--store --load] [--all| --testX ...]\n", argv[0]);
		unittest_help_mode();
	}

	unittest_init();

	int all = contains(argv,argc,"--all", "Perform all tests")!=0;

	TestData testdata;
	testdata.fi.width=1280;
  testdata.fi.height=720;
  testdata.fi.strive=1280;
  testdata.fi.framesize=1382400;
  testdata.fi.pFormat = PF_YUV;
  testdata.fi_color.width=640;
  testdata.fi_color.height=360;

  if(contains(argv,argc,"--load",
							"Load frames from files from frames/frame001.raw (def: generate)")!=0){
    FILE* file;
    char name[128];
		int i;
    for(i=0; i<5; i++){
      testdata.frames[i] = (unsigned char*)malloc(testdata.fi.framesize);
      sprintf(name,"../frames/frame%03i.raw",i+4);
      fprintf(stderr, "load file %s\n", name);
      file = fopen(name,"rb");
      test_bool(file!=0);
      fprintf(stderr,"read %li bytes\n",
							(unsigned long)fread(testdata.frames[i], 1, testdata.fi.framesize,file));
      fclose(file);
    }
  }else{
		UNIT(generateFrames(&testdata, 5));
  }
	if(contains(argv,argc,"--store", "Store frames to files")!=0){
		storePGMImage("test1.pgm", testdata.frames[0], testdata.fi);
		storePGMImage("test2.pgm", testdata.frames[1], testdata.fi);
		storePGMImage("test3.pgm", testdata.frames[2], testdata.fi);
		storePGMImage("test4.pgm", testdata.frames[3], testdata.fi);
		storePGMImage("test5.pgm", testdata.frames[4], testdata.fi);
	}

#ifdef USE_OMP
  openmp();
#endif

  if(all || contains(argv,argc,"--testTI", "transform_implementation")){
		UNIT(test_transform_implementation(&testdata));
	}

  if(all || contains(argv,argc,"--testTP", "transform_performance")){
		UNIT(test_transform_performance(&testdata));
	}

  if(all || contains(argv,argc,"--testBB", "boxblur")){
		UNIT(test_boxblur(&testdata));
	}

  if(all || contains(argv,argc,"--testCCI", "checkCompareImg")){
		UNIT(test_checkCompareImg(&testdata));
	}

  if(all || contains(argv,argc,"--testCIP", "compareImg_performance")){
		UNIT(test_compareImg_performance(&testdata));
	}

  if(all || contains(argv,argc,"--testMD", "motionDetect")){
		UNIT(test_motionDetect(&testdata));
	}

  if(all || contains(argv,argc,"--testSR", "store_restore")){
		UNIT(test_store_restore(&testdata));
	}

  if(all || contains(argv,argc,"--testCT", "contrastImg")){
		UNIT(test_contrastImg(&testdata));
	}

	return unittest_summary();
}
