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

#define SYN_NUM_SQUARES 4
#define SYN_SQUARE_SIZE 16
#define SYN_SQUARE_R 40
#define SYN_SQUARE_G 90
#define SYN_SQUARE_B 230
#define SYN_SEED 12345u

typedef struct { double x, y, vx, vy; } SynSquare;

static void synSquareVelocity(unsigned int seed, int idx, double* vx, double* vy){
  double angle, speed;
  srand(seed + (unsigned int)idx*7919u);
  angle = (rand()%360) * M_PI/180.0;
  speed = 3.0 + (rand()%5); /* 3..7 px/frame */
  *vx = cos(angle)*speed;
  *vy = sin(angle)*speed;
}

/* Same base sequence as generateCircleFrames (uniform-motion camera shake),
   but with SYN_NUM_SQUARES squares of a third color stamped on top of every
   frame, each moving along its own fixed-seed, per-square constant velocity
   that is independent of the camera-shake transform (bouncing off frame
   edges). These model independently-moving foreground distractors that the
   stabilizer must not lock onto. */
void generateCircleSquareFrames(VSFrame* frames, VSFrameInfo* fi, VSPixelFormat pf,
                                int width, int height, int numFrames){
  SynSquare squares[SYN_NUM_SQUARES];
  int i, k;

  generateCircleFrames(frames, fi, pf, width, height, numFrames);

  srand(SYN_SEED);
  for(k=0; k<SYN_NUM_SQUARES; k++){
    squares[k].x = 20 + rand()%(width - 40 - SYN_SQUARE_SIZE);
    squares[k].y = 20 + rand()%(height - 40 - SYN_SQUARE_SIZE);
    synSquareVelocity(SYN_SEED, k, &squares[k].vx, &squares[k].vy);
  }

  for(i=0; i<numFrames; i++){
    for(k=0; k<SYN_NUM_SQUARES; k++){
      if(i>0){
        squares[k].x += squares[k].vx;
        squares[k].y += squares[k].vy;
        if(squares[k].x<0){ squares[k].x=0; squares[k].vx=-squares[k].vx; }
        if(squares[k].y<0){ squares[k].y=0; squares[k].vy=-squares[k].vy; }
        if(squares[k].x>width-SYN_SQUARE_SIZE){ squares[k].x=width-SYN_SQUARE_SIZE; squares[k].vx=-squares[k].vx; }
        if(squares[k].y>height-SYN_SQUARE_SIZE){ squares[k].y=height-SYN_SQUARE_SIZE; squares[k].vy=-squares[k].vy; }
      }
      paintSquareRGB(&frames[i], fi, (int)squares[k].x, (int)squares[k].y, SYN_SQUARE_SIZE,
                    SYN_SQUARE_R, SYN_SQUARE_G, SYN_SQUARE_B);
    }
  }
}

/* A longer, bounded-amplitude "shake" path: sway plus jitter, no net drift.
   getTestFrameTransform() (used above) grows without bound frame over frame,
   which is fine for the short 6-frame sequences elsewhere but would carry the
   content off-canvas well before frame 30 -- this instead orbits around the
   identity so the synthetic scene stays trackable for the whole sequence,
   which is what a real detection -> L1-camera-path smoothing test needs. */
#define SYN_L1_NUM_FRAMES 30

static VSTransform synL1ShakeTransform(int i){
  double s = (double)i;
  VSTransform t = null_transform();
  t.x = 15.0 * sin(s * 0.35) + 4.0 * sin(s * 1.3);
  t.y = 10.0 * sin(s * 0.27 + 1.0) + 3.0 * sin(s * 1.7);
  t.alpha = (2.0 * sin(s * 0.19) + 0.5 * sin(s * 2.1)) * M_PI / 180.0;
  t.zoom = 0;
  return t;
}

/* Same base scene and warp mechanism as generateCircleFrames(), but driven by
   synL1ShakeTransform() instead of getTestFrameTransform(). */
void generateL1ShakeFrames(VSFrame* frames, VSFrameInfo* fi, VSPixelFormat pf,
                           int width, int height, int numFrames){
  int i;
  VSTransformConfig conf;
  VSTransformData td;

  test_bool(vsFrameInfoInit(fi, width, height, pf) != 0);
  for(i=0; i<numFrames; i++)
    vsFrameAllocate(&frames[i], fi);

  paintSynBase(&frames[0], fi);

  conf = vsTransformGetDefaultConfig("gen_syn_l1shake");
  conf.interpolType = VS_Zero;
  test_bool(vsTransformDataInit(&td, &conf, fi, fi) == VS_OK);
  for(i=1; i<numFrames; i++){
    VSTransform t = synL1ShakeTransform(i);
    test_bool(vsTransformPrepare(&td, &frames[i-1], &frames[i]) == VS_OK);
    test_bool(vsDoTransform(&td, t) == VS_OK);
    test_bool(vsTransformFinish(&td) == VS_OK);
  }
  vsTransformDataCleanup(&td);
}

void dumpFramesAsPPM(const VSFrame* frames, const VSFrameInfo* fi, int numFrames,
                     const char* prefix){
  int i;
  char name[512];
  for(i=0; i<numFrames; i++){
    snprintf(name, sizeof(name), "%s%03i.ppm", prefix, i);
    test_bool(storePPMImage(name, &frames[i], fi));
  }
}
