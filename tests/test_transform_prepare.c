/* Tests the buffer ownership contract of vsTransformPrepare().
 *
 * A host may call vsTransformPrepare() with separate src/dest buffers on one
 * frame and in-place (src==dest) on the next. FFmpeg's vf_vidstabtransform does
 * exactly this: it picks the path per frame with av_frame_is_writable(), so with
 * a B-frame source the two paths interleave.
 *
 * In that case vid.stab must never write into the buffer the caller passed as
 * src: for FFmpeg those are decoder reference frames that are still in use.
 * See issue #144.
 */

/* number of bytes in all planes that differ from the expected fill value */
static int countDirtyBytes(const VSFrame* frame, const VSFrameInfo* fi,
                           unsigned char expected){
  int dirty = 0;
  for(int plane=0; plane < fi->planes; plane++){
    int w = CHROMA_SIZE(fi->width , vsGetPlaneWidthSubS(fi, plane));
    int h = CHROMA_SIZE(fi->height, vsGetPlaneHeightSubS(fi, plane));
    for(int y=0; y<h; y++)
      for(int x=0; x<w; x++)
        if(frame->data[plane][x + y*frame->linesize[plane]] != expected)
          dirty++;
  }
  return dirty;
}

/* Counts near-black luma pixels. Note this cannot test for exactly 0: the
   interpolators add 1 to compensate for truncation, so a black-filled border
   comes out as luma 1. */
static int countLumaAtMost(const VSFrame* frame, const VSFrameInfo* fi,
                           unsigned char value){
  int cnt = 0;
  for(int y=0; y<fi->height; y++)
    for(int x=0; x<fi->width; x++)
      if(frame->data[0][x + y*frame->linesize[0]] <= value)
        cnt++;
  return cnt;
}

#define TP_NEAR_BLACK 2

static void fillFrame(VSFrame* frame, const VSFrameInfo* fi, unsigned char v){
  for(int plane=0; plane < fi->planes; plane++){
    int h = CHROMA_SIZE(fi->height, vsGetPlaneHeightSubS(fi, plane));
    memset(frame->data[plane], v, (size_t)frame->linesize[plane] * h);
  }
}

#define TP_REF_FILL  0x11   /* the "decoder reference frame" */
#define TP_NEXT_FILL 0x22   /* the following, writable frame */

/* Runs: frame A with separate src/dest, then (optionally) frame B in-place.
   Returns the number of bytes of A's buffer that vid.stab modified. */
static int runPrepareSequence(VSBorderType crop, short mixInPlaceFrame,
                              int* lumaBlackInOutput){
  VSFrameInfo fi;
  vsFrameInfoInit(&fi, 64, 64, PF_YUV420P);

  VSTransformConfig conf = vsTransformGetDefaultConfig("test_transform_prepare");
  conf.crop      = crop;
  conf.relative  = 0;
  conf.smoothing = 0;
  conf.optZoom   = 0;

  VSTransformData td;
  test_bool(vsTransformDataInit(&td, &conf, &fi, &fi) == VS_OK);

  VSFrame ref, out, next;
  vsFrameAllocate(&ref,  &fi);
  vsFrameAllocate(&out,  &fi);
  vsFrameAllocate(&next, &fi);
  fillFrame(&ref,  &fi, TP_REF_FILL);
  fillFrame(&out,  &fi, 0);
  fillFrame(&next, &fi, TP_NEXT_FILL);

  /* A translation large enough to leave an uncovered border. It has to exceed
     the 10 pixel outward feathering done by interpolateBiLinBorder(), otherwise
     the uncovered pixels are still dominated by the source edge pixel and never
     reach the border fill value. */
  VSTransform t = new_transform(20, 20, 0, 0, 0, 0, 0);

  /* frame A: not writable for the host, so src and dest are distinct */
  test_bool(vsTransformPrepare(&td, &ref, &out) == VS_OK);
  test_bool(vsDoTransform(&td, t) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);

  if(lumaBlackInOutput)
    *lumaBlackInOutput = countLumaAtMost(&out, &fi, TP_NEAR_BLACK);

  if(mixInPlaceFrame){
    /* frame B: writable, so the host transforms it in place */
    test_bool(vsTransformPrepare(&td, &next, &next) == VS_OK);
    /* once the in-place path is taken, the source buffer must be owned by us */
    test_bool(td.srcMalloced == 1);
    test_bool(td.src.data[0] != ref.data[0]);
    test_bool(vsDoTransform(&td, t) == VS_OK);
    test_bool(vsTransformFinish(&td) == VS_OK);
  }

  int dirty = countDirtyBytes(&ref, &fi, TP_REF_FILL);

  vsTransformDataCleanup(&td);
  vsFrameFree(&ref);
  vsFrameFree(&out);
  vsFrameFree(&next);
  return dirty;
}

void test_transform_prepare(void){
  int dirty;
  int black = -1;

  fprintf(stderr,"--- vsTransformPrepare must not write into the caller's src ---\n");

  /* control: only the separate-buffer path is ever used (source has no B-frames).
     The src buffer is only ever read, so it has to come back untouched. */
  dirty = runPrepareSequence(VSKeepBorder, 0, &black);
  fprintf(stderr,"separate buffers only        : %i bytes of caller's frame changed\n",
          dirty);
  test_bool(dirty == 0);

  /* the keep-border mechanism must still fill uncovered areas from the previous
     frame held in destbuf, not with black */
  fprintf(stderr,"keep border  : %i near-black luma pixels in output (expect 0)\n", black);
  test_bool(black == 0);

  /* and with crop the uncovered border must be black, so the two modes differ */
  runPrepareSequence(VSCropBorder, 0, &black);
  fprintf(stderr,"crop border  : %i near-black luma pixels in output (expect > 0)\n", black);
  test_bool(black > 0);

  /* the regression: the two paths interleave, as they do for a B-frame source */
  dirty = runPrepareSequence(VSKeepBorder, 1, NULL);
  fprintf(stderr,"mixed, keep border           : %i bytes of caller's frame changed\n",
          dirty);
  test_bool(dirty == 0);

  dirty = runPrepareSequence(VSCropBorder, 1, NULL);
  fprintf(stderr,"mixed, crop border           : %i bytes of caller's frame changed\n",
          dirty);
  test_bool(dirty == 0);
}
