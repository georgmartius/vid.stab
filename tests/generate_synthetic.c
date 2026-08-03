/* Canvas size and circle layout (see "Amendment: scene rework" in the plan doc):
   the original 320x240/5-circle scene only gave vsSimpleMotionsToTransform's
   affine least-squares fit ~24-40 usable measurement fields to work with, which
   isn't enough spatial signal to resolve the rotation-angle (alpha) component to
   within tolAlpha=0.005 rad on frames 3-5. Rotation-angle estimation error scales
   inversely with each sample's radius from the rotation center, so this scene
   trades the original cluster of small circles near the frame center for fewer,
   bigger circles pushed out toward the corners/edges of a much larger canvas --
   maximizing both per-sample tangential displacement and total measurement-field
   count. */
#define SYN_WIDTH 640
#define SYN_HEIGHT 480
#define SYN_NUM_FRAMES 6
#define SYN_NUM_CIRCLES 8

/* Chosen so the circle/background luma difference clears vid.stab's default
   Michelson-contrast field threshold (contrastThreshold = 0.25, see
   motiondetect.c): background luma ~20, circle luma ~198, giving
   (198-20)/(198+20) ~= 0.81, well above threshold. The original (90,90,90) /
   (230,60,40) pair only reached a luma contrast of ~0.09 and made motion
   detection fail with "too low contrast" on every non-RGB pixel format. */
#define SYN_BG_R 20
#define SYN_BG_G 20
#define SYN_BG_B 20
#define SYN_CIRCLE_R 250
#define SYN_CIRCLE_G 200
#define SYN_CIRCLE_B 50

typedef struct { int cx, cy, radius; } SynCircle;

/* 8 circles at the corners/edge-midpoints of a safe inset rectangle, all far
   (~190-260px) from the frame center (320,240) -- large lever arms for the
   rotation-angle signal -- while staying well clear of the canvas edges so
   cumulative translation across the sequence never clips them out. */
static const SynCircle SYN_CIRCLES[SYN_NUM_CIRCLES] = {
  {100,100,40}, {320,100,40}, {540,100,40},
  {100,240,40},                {540,240,40},
  {100,380,40}, {320,380,40}, {540,380,40}
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
