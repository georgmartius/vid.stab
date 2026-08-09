
#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <math.h>

#if defined(__linux__)
#include <features.h>
#endif

#ifdef USE_OMP
#include <omp.h>
#endif

#include "libvidstab.h"
// load optimized functions
#include "motiondetect_internal.h"
#include "motiondetect_opt.h"
#include "boxblur.h"
#include "transformfixedpoint.h"
#include "transformfloat.h"
#include "transformtype_operations.h"

#ifndef TESTING
#error TESTING must be defined
#endif

#include "testframework.h"
#include "testutils.h"

#include "generate.c"
#include "generate_synthetic.c"

#include "test_frameinfo.c"
#include "test_transform.c"
#include "test_interpolate.c"
#include "test_transform_prepare.c"
#include "test_compareimg.c"
#include "test_simd_equivalence.c"
#include "test_motiondetect.c"
#include "test_store_restore.c"
#include "test_serialize_robust.c"
#include "test_vsvector.c"
#include "test_contrast.c"
#include "test_boxblur.c"
#include "test_omp.c"
#include "test_gradientoptimizer.c"
#include "test_localmotion2transform.c"
#include "test_determinism.c"
#include "test_packed.c"
#include "test_draw.c"
#include "test_synthetic.c"
#include "test_tripod.c"
#include "test_lensdistortion.c"
#include "test_lensmap.c"
#include "test_transform_baseline.c"
#ifdef VS_HAVE_LPSOLVER
#include "test_campathopt.c"
#endif

#define FRAMENUM 5

int main(int argc, char** argv){

  if(contains(argv,argc,"-h", "help")!=0){
    printf("Usage: %s [--store --load] [--all| --testX ...]\n", argv[0]);
    unittest_help_mode();
  }

  unittest_init();

  int all = contains(argv,argc,"--all", "Perform all tests")!=0;

  TestData testdata;
  memset(&testdata,0,sizeof(TestData));
  vsFrameInfoInit(&testdata.fi,1280, 720, PF_YUV420P);
  vsFrameInfoInit(&testdata.fi_color, 640, 360, PF_GRAY8);

  if(contains(argv,argc,"--load",
              "Load frames from files from frames/frame001.raw (def: generate)")!=0){
    FILE* file;
    char name[128];
    int i;
    for(i=0; i<FRAMENUM; i++){
      vsFrameAllocate(&testdata.frames[i],&testdata.fi);
      sprintf(name,"../frames/frame%03i.raw",i+4);
      fprintf(stderr, "load file %s\n", name);
      file = fopen(name,"rb");
      test_bool(file!=0);
      fprintf(stderr,"read %li bytes\n",
              (unsigned long)fread(testdata.frames[i].data[0], 1,
                                   testdata.fi.width*testdata.fi.height,file));
      fclose(file);
    }
  }else{
    UNIT(generateFrames(&testdata, FRAMENUM));
  }
  if(contains(argv,argc,"--store", "Store frames to files")!=0){
    storePGMImage(testOut("test1.pgm"), testdata.frames[0].data[0], testdata.fi);
    storePGMImage(testOut("test2.pgm"), testdata.frames[1].data[0], testdata.fi);
    storePGMImage(testOut("test3.pgm"), testdata.frames[2].data[0], testdata.fi);
    storePGMImage(testOut("test4.pgm"), testdata.frames[3].data[0], testdata.fi);
    storePGMImage(testOut("test5.pgm"), testdata.frames[4].data[0], testdata.fi);
  }

#ifdef USE_OMP
  if(all || contains(argv,argc,"--testOMP", "openmp")){
    UNIT(openmp());
  }
#endif

  if(all || contains(argv,argc,"--testFI", "frameinfo dimension validation")){
    UNIT(test_frameinfo());
  }

  if(all || contains(argv,argc,"--testTI", "transform_implementation")){
    UNIT(test_transform_implementation(&testdata));
  }

  if(all || contains(argv,argc,"--testIP", "interpolation borders")){
    UNIT(test_interpolate_borders());
  }

  if(all || contains(argv,argc,"--testTP", "transform_performance")){
    UNIT(test_transform_performance(&testdata));
  }

  if(all || contains(argv,argc,"--testTPR", "transform_prepare buffer ownership")){
    UNIT(test_transform_prepare());
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

  if(all || contains(argv,argc,"--testSIMD", "SSE2 vs C equivalence")){
    UNIT(test_simd_equivalence(&testdata));
  }

  if(all || contains(argv,argc,"--testMD", "motionDetect")){
    UNIT(test_motionDetect(&testdata));
  }

  if(all || contains(argv,argc,"--testLM", "localmotion2transform")){
    UNIT(test_localmotion2transform(&testdata));
  }

  if(all || contains(argv,argc,"--testSR", "store_restore")){
    UNIT(test_store_restore(&testdata, ASCII_SERIALIZATION_MODE));
    UNIT(test_store_restore(&testdata, BINARY_SERIALIZATION_MODE));
    UNIT(test_store_restore(&testdata, 0)); // test default binary selection
  }

  if(all || contains(argv,argc,"--testSRO", "serialization robustness")){
    UNIT(test_serialize_robust());
  }

  if(all || contains(argv,argc,"--testVEC", "vsvector bounds")){
    UNIT(test_vsvector_bounds());
  }

  if(all || contains(argv,argc,"--testCT", "contrastImg")){
    UNIT(test_contrastImg(&testdata));
  }

  if(all || contains(argv,argc,"--testPK", "packed pixel formats")){
    UNIT(test_packed());
  }

  if(all || contains(argv,argc,"--testDRAW", "overlay drawing primitives")){
    UNIT(test_draw_geometry());
    UNIT(test_draw_clipping());
    UNIT(test_draw_packed());
    UNIT(test_draw_planar_show());
  }

  if(all || contains(argv,argc,"--testSYN", "synthetic circles across pixel formats")){
    UNIT(test_synthetic_circles());
  }

  if(all || contains(argv,argc,"--testSYNSQ", "synthetic circles+squares across pixel formats")){
    UNIT(test_synthetic_circles_squares());
  }

  if(all || contains(argv,argc,"--testTRIPOD", "virtual tripod mode")){
    UNIT(test_tripod_transforms());
    UNIT(test_tripod_detection());
  }

  if(all || contains(argv,argc,"--testLENS", "barrel distortion model and estimation")){
    UNIT(test_lensdistortion_model());
    UNIT(test_lensdistortion_generator());
    UNIT(test_lensdistortion_fit());
    UNIT(test_lensdistortion_estimate());
    UNIT(test_lensdistortion_robustness());
    UNIT(test_lensdistortion_outliers());
    UNIT(test_lensdistortion_endtoend());
    UNIT(test_lensdistortion_phase2());
  }

  if(all || contains(argv,argc,"--testLMAP", "render-path lens map and LUTs")){
    UNIT(test_lensmap_scales());
    UNIT(test_lensmap_identity_transform());
    UNIT(test_lensmap_lut());
    UNIT(test_lensmap_domain());
    UNIT(test_lensmap_inactive());
    UNIT(test_lensmap_chroma_consistency());
    UNIT(test_lensmap_wobble_identity_takes_fast_path());
    UNIT(test_lensmap_wobble_cancellation_through_loop());
    UNIT(test_lensmap_removes_wobble());
    UNIT(test_lensmap_zero_k_stays_inactive());
    UNIT(test_lensmap_nonzero_k_builds_on_first_call());
    UNIT(test_lensmap_chroma_render());
    UNIT(test_lensmap_fixed_float_equivalence());
    UNIT(test_lensmap_fixed_reference());
    UNIT(test_lensmap_packed());
    UNIT(test_lensmap_fixed_reference_packed());
    UNIT(test_lensmap_required_zoom());
    UNIT(test_lensmap_required_zoom_off());
  }

  if(all || contains(argv,argc,"--testBASE", "warp-loop output baseline (k=0 guard)")){
    UNIT(test_transform_baseline());
  }

  if(contains(argv,argc,"--dumpSynthetic", "dump synthetic frames as PPM for visual inspection")){
    int fmt;
    char prefix[512];
    for(fmt=0; fmt<SYN_NUM_FORMATS; fmt++){
      VSFrameInfo fi;
      VSFrame frames[SYN_NUM_FRAMES];
      int i;
      /* testOut() creates the per-format directory on the way */
      const char* fmtname = synFormatName(SYN_FORMATS[fmt]);

      generateCircleFrames(frames, &fi, SYN_FORMATS[fmt], SYN_WIDTH, SYN_HEIGHT, SYN_NUM_FRAMES);
      sprintf(prefix, "synthetic/%s/circles", fmtname);
      dumpFramesAsPPM(frames, &fi, SYN_NUM_FRAMES, testOut(prefix));
      for(i=0; i<SYN_NUM_FRAMES; i++) vsFrameFree(&frames[i]);

      generateCircleSquareFrames(frames, &fi, SYN_FORMATS[fmt], SYN_WIDTH, SYN_HEIGHT, SYN_NUM_FRAMES);
      sprintf(prefix, "synthetic/%s/circles_squares", fmtname);
      dumpFramesAsPPM(frames, &fi, SYN_NUM_FRAMES, testOut(prefix));
      for(i=0; i<SYN_NUM_FRAMES; i++) vsFrameFree(&frames[i]);

      fprintf(stderr, "dumped synthetic PPM frames to %s/synthetic/%s\n",
              TEST_OUTPUT_DIR, fmtname);
    }
  }

  if(all || contains(argv,argc,"--testGO", "gradient optimizer")){
    UNIT(test_gradientoptimizer());
  }

  if(all || contains(argv,argc,"--testDET", "deterministic output")){
    UNIT(test_determinism(&testdata));
  }

#ifdef VS_HAVE_LPSOLVER
  if(all || contains(argv,argc,"--testL1", "L1 optimal camera path")){
    UNIT(test_l1_indices());
    UNIT(test_l1_transformLS());
    UNIT(test_l1_campath());
    UNIT(test_l1_reference());
    UNIT(test_l1_campath_transforms(&testdata));
    UNIT(test_l1_synthetic_detection());
  }
#endif

  // free
  /* testdata was zeroed above, so unallocated frames have NULL planes and
     vsFrameFree() handles those safely; no NULL check on the address needed. */
  for(int i=0; i<FRAMENUM; i++){
    vsFrameFree(&testdata.frames[i]);
  }

  return unittest_summary();
}
