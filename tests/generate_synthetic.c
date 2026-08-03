#define SYN_WIDTH 320
#define SYN_HEIGHT 240
#define SYN_NUM_FRAMES 6
#define SYN_NUM_CIRCLES 5

#define SYN_BG_R 90
#define SYN_BG_G 90
#define SYN_BG_B 90
#define SYN_CIRCLE_R 230
#define SYN_CIRCLE_G 60
#define SYN_CIRCLE_B 40

typedef struct { int cx, cy, radius; } SynCircle;

static const SynCircle SYN_CIRCLES[SYN_NUM_CIRCLES] = {
  {80,60,18}, {240,60,15}, {160,120,20}, {80,180,16}, {240,180,14}
};

static void paintSynBase(VSFrame* frame, const VSFrameInfo* fi){
  int k;
  fillFrameRGB(frame, fi, SYN_BG_R, SYN_BG_G, SYN_BG_B);
  for(k=0; k<SYN_NUM_CIRCLES; k++)
    paintCircleRGB(frame, fi, SYN_CIRCLES[k].cx, SYN_CIRCLES[k].cy, SYN_CIRCLES[k].radius,
                   SYN_CIRCLE_R, SYN_CIRCLE_G, SYN_CIRCLE_B);
}

/* frame 0: uniform background + SYN_NUM_CIRCLES filled circles. frames 1..N-1:
   each is produced by warping the previous frame with the public transform
   API using getTestFrameTransform(i) -- the same mechanism generateFrames()
   in generate.c uses, generalized here via vsDoTransform() so it dispatches
   correctly for both planar and packed pixel formats. */
void generateCircleFrames(VSFrame* frames, VSFrameInfo* fi, VSPixelFormat pf,
                          int width, int height, int numFrames){
  int i;
  VSTransformConfig conf;
  VSTransformData td;

  test_bool(vsFrameInfoInit(fi, width, height, pf) != 0);
  for(i=0; i<numFrames; i++)
    vsFrameAllocate(&frames[i], fi);

  paintSynBase(&frames[0], fi);

  conf = vsTransformGetDefaultConfig("gen_syn_circles");
  conf.interpolType = VS_Zero;
  test_bool(vsTransformDataInit(&td, &conf, fi, fi) == VS_OK);
  for(i=1; i<numFrames; i++){
    VSTransform t = getTestFrameTransform(i);
    test_bool(vsTransformPrepare(&td, &frames[i-1], &frames[i]) == VS_OK);
    test_bool(vsDoTransform(&td, t) == VS_OK);
    test_bool(vsTransformFinish(&td) == VS_OK);
  }
  vsTransformDataCleanup(&td);
}
