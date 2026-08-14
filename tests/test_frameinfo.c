/* Regression test for issue #43 ("Cannot stabilize video with odd height").

   vsFrameInfoInit() used to guard the even-dimension requirement with a plain
   assert(), which -DNDEBUG (i.e. any Release build) compiles away. Odd
   dimensions then silently proceeded and the chroma plane arithmetic walked
   off the end of the allocation made by vsFrameAllocate().

   Note: vsFrameInfoInit() uses a boolean return convention -- non-zero means
   success, 0 means failure -- not VS_OK/VS_ERROR. Downstream callers (e.g.
   ffmpeg's vf_vidstabtransform.c) test it with `if(!vsFrameInfoInit(...))`.
*/

// writes to every byte of every plane, so ASAN/valgrind catch undersized
// allocations, and checks the plane geometry against the expectation
static void checkFrameUsable(VSFrameInfo* fi, int expectedChromaW, int expectedChromaH){
  VSFrame frame;
  int plane;
  test_bool(fi->log2ChromaW == expectedChromaW);
  test_bool(fi->log2ChromaH == expectedChromaH);
  vsFrameAllocate(&frame, fi);
  test_bool(!vsFrameIsNull(&frame));
  for(plane=0; plane < fi->planes; plane++){
    int w = fi->width  >> vsGetPlaneWidthSubS(fi, plane);
    int h = fi->height >> vsGetPlaneHeightSubS(fi, plane);
    // the truncating shift used for allocation must agree with the rounding-up
    // CHROMA_SIZE() macro that the transform code uses, otherwise the
    // transform writes past the end of this buffer
    test_bool(w == CHROMA_SIZE(fi->width,  vsGetPlaneWidthSubS(fi, plane)));
    test_bool(h == CHROMA_SIZE(fi->height, vsGetPlaneHeightSubS(fi, plane)));
    test_bool(frame.linesize[plane] == w);
    test_bool(frame.data[plane] != 0);
    memset(frame.data[plane], 0x7f, (size_t)w * (size_t)h);
  }
  vsFrameFree(&frame);
}

void test_frameinfo(void){
  VSFrameInfo fi;

  fprintf(stderr,"********** frameinfo dimension validation:\n");

  fprintf(stderr,"* subsampled formats must reject incompatible dimensions\n");
  fprintf(stderr,"  (the library's \"Error: invalid frame dimensions\" lines"
                 " below are the expected result)\n");
  // this is issue #43: odd height with 4:2:0
  test_bool(vsFrameInfoInit(&fi, 640, 481, PF_YUV420P) == 0);
  test_bool(vsFrameInfoInit(&fi, 641, 480, PF_YUV420P) == 0);
  test_bool(vsFrameInfoInit(&fi, 641, 481, PF_YUV420P) == 0);
  test_bool(vsFrameInfoInit(&fi, 641, 480, PF_YUVA420P) == 0);
  // 4:2:2 subsamples horizontally only -> odd width is invalid
  test_bool(vsFrameInfoInit(&fi, 641, 480, PF_YUV422P) == 0);
  // 4:4:0 subsamples vertically only -> odd height is invalid
  test_bool(vsFrameInfoInit(&fi, 640, 481, PF_YUV440P) == 0);
  // 4:1:0 / 4:1:1 need multiples of 4
  test_bool(vsFrameInfoInit(&fi, 642, 480, PF_YUV410P) == 0);
  test_bool(vsFrameInfoInit(&fi, 642, 480, PF_YUV411P) == 0);

  fprintf(stderr,"* non-positive dimensions are rejected\n");
  test_bool(vsFrameInfoInit(&fi, 0, 480, PF_YUV420P) == 0);
  test_bool(vsFrameInfoInit(&fi, 640, -2, PF_YUV420P) == 0);
  test_bool(vsFrameInfoInit(&fi, 640, 480, PF_NONE) == 0);

  fprintf(stderr,"* formats without subsampling accept odd dimensions\n");
  test_bool(vsFrameInfoInit(&fi, 641, 481, PF_GRAY8) != 0);
  test_bool(fi.planes == 1);
  checkFrameUsable(&fi, 0, 0);

  test_bool(vsFrameInfoInit(&fi, 641, 481, PF_YUV444P) != 0);
  test_bool(fi.planes == 3);
  checkFrameUsable(&fi, 0, 0);

  fprintf(stderr,"* partially subsampled formats accept the free dimension odd\n");
  // 4:2:2 does not subsample vertically -> odd height is fine
  test_bool(vsFrameInfoInit(&fi, 640, 481, PF_YUV422P) != 0);
  checkFrameUsable(&fi, 1, 0);
  // 4:4:0 does not subsample horizontally -> odd width is fine
  test_bool(vsFrameInfoInit(&fi, 641, 480, PF_YUV440P) != 0);
  checkFrameUsable(&fi, 0, 1);
  // 4:1:1 does not subsample vertically -> odd height is fine
  test_bool(vsFrameInfoInit(&fi, 640, 481, PF_YUV411P) != 0);
  checkFrameUsable(&fi, 2, 0);

  fprintf(stderr,"* the usual even sizes keep working\n");
  test_bool(vsFrameInfoInit(&fi, 1280, 720, PF_YUV420P) != 0);
  test_bool(fi.planes == 3);
  checkFrameUsable(&fi, 1, 1);
  test_bool(vsFrameInfoInit(&fi, 640, 480, PF_YUV410P) != 0);
  checkFrameUsable(&fi, 2, 2);
  test_bool(vsFrameInfoInit(&fi, 1280, 720, PF_YUVA420P) != 0);
  test_bool(fi.planes == 4);
  checkFrameUsable(&fi, 1, 1);
}
