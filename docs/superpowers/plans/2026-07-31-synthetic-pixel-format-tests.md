# Synthetic Pixel-Format Stabilizer Tests Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add deterministic, visually-inspectable synthetic test content (uniform background + moving circles, and a variant with independently-moving distractor squares) across 6 representative `VSPixelFormat`s, plus unit tests that verify `vsMotionDetection` recovers the injected camera motion.

**Architecture:** Extend the existing custom C test framework under `tests/` (single `tests` binary built by textually `#include`-ing `.c` files into `tests/tests.c`, run via `UNIT()`/`test_bool()` macros and `--testXXX` CLI flags). Add format-agnostic RGB pixel read/write helpers to `tests/testutils.{h,c}`, a new synthetic-sequence generator (`tests/generate_synthetic.c`) that reuses the existing `getTestFrameTransform()` + `vsTransformPrepare`/`vsDoTransform`/`vsTransformFinish` warp pipeline (same mechanism `tests/generate.c` already uses, generalized via the public `vsDoTransform` so it dispatches correctly for both planar and packed formats), and a new test file (`tests/test_synthetic.c`) wired into `tests/tests.c`.

**Tech Stack:** C (gnu99), CMake, the project's existing custom test framework (no external test library).

## Global Constraints

- Target pixel formats (from the spec): `PF_GRAY8`, `PF_YUV420P`, `PF_YUV422P`, `PF_YUV444P`, `PF_RGB24`, `PF_RGBA`.
- Canvas size `320x240` for all synthetic sequences (even width/height satisfies every subsampling constraint checked by `vsFrameInfoInit`, see `tests/test_frameinfo.c`).
- Motion tolerance for circles-only recovery: `fabs(diff.x) < 2 && fabs(diff.y) < 2 && fabs(diff.alpha) < 0.005` (same as `tests/test_motiondetect.c`).
- All new C files follow the existing convention: no header files for `generate_synthetic.c`/`test_synthetic.c` — they are textually `#include`d into `tests/tests.c` in a fixed order, so declaration order = include order.
- Follow existing code style in `tests/`: 2-space indent, `test_bool()` for assertions, `fprintf(stderr, ...)` for diagnostics.
- Do not modify `tests/CMakeLists.txt` (the `#include`-based build needs no new translation units).
- This work happens in the current git worktree/branch (already set up).

---

## File Structure

- **Modify** `tests/testutils.h` — declare new RGB pixel helpers.
- **Modify** `tests/testutils.c` — implement new RGB pixel helpers (RGB↔native-format conversion, shape drawing, PPM writer).
- **Create** `tests/generate_synthetic.c` — `generateCircleFrames()` / `generateCircleSquareFrames()` / `dumpFramesAsPPM()`.
- **Create** `tests/test_synthetic.c` — `test_synthetic_circles()` / `test_synthetic_circles_squares()`.
- **Modify** `tests/tests.c` — `#include` the two new `.c` files, add `--testSYN` / `--testSYNSQ` / `--dumpSynthetic` CLI flags.
- **Create** `.gitignore` (repo root, none exists yet) — ignore local CMake build directories and the optional PPM dump output directory.

---

## Task 1: RGB pixel helpers in testutils + roundtrip sanity test

**Files:**
- Modify: `tests/testutils.h`
- Modify: `tests/testutils.c`
- Create: `tests/test_synthetic.c`
- Modify: `tests/tests.c`

**Interfaces:**
- Produces (used by later tasks):
  - `void fillFrameRGB(VSFrame* frame, const VSFrameInfo* fi, uint8_t r, uint8_t g, uint8_t b);`
  - `void setPixelRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y, uint8_t r, uint8_t g, uint8_t b);`
  - `void getPixelRGB(const VSFrame* frame, const VSFrameInfo* fi, int x, int y, uint8_t* r, uint8_t* g, uint8_t* b);`
  - `void paintCircleRGB(VSFrame* frame, const VSFrameInfo* fi, int cx, int cy, int radius, uint8_t r, uint8_t g, uint8_t b);`
  - `void paintSquareRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y, int size, uint8_t r, uint8_t g, uint8_t b);`
  - `int storePPMImage(const char* filename, const VSFrame* frame, const VSFrameInfo* fi);`
  - `const VSPixelFormat SYN_FORMATS[6]` and `#define SYN_NUM_FORMATS 6` (defined in `tests/test_synthetic.c`, used by later tasks too — placed at file scope near the top so it's visible to functions added in Tasks 2-4).
  - `const char* synFormatName(VSPixelFormat pf)` (in `tests/test_synthetic.c`).

- [ ] **Step 1: Add declarations to `tests/testutils.h`**

Edit `tests/testutils.h`, right after the existing `paintRectangle` declaration:

```c
void paintRectangle(unsigned char* buffer, const VSFrameInfo* fi, int x, int y,
                    int sizex, int sizey, unsigned char color);

void fillFrameRGB(VSFrame* frame, const VSFrameInfo* fi, uint8_t r, uint8_t g, uint8_t b);

void setPixelRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y,
                 uint8_t r, uint8_t g, uint8_t b);

void getPixelRGB(const VSFrame* frame, const VSFrameInfo* fi, int x, int y,
                 uint8_t* r, uint8_t* g, uint8_t* b);

void paintCircleRGB(VSFrame* frame, const VSFrameInfo* fi, int cx, int cy, int radius,
                    uint8_t r, uint8_t g, uint8_t b);

void paintSquareRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y, int size,
                    uint8_t r, uint8_t g, uint8_t b);

int storePPMImage(const char* filename, const VSFrame* frame, const VSFrameInfo* fi);
```

- [ ] **Step 2: Implement the helpers in `tests/testutils.c`**

Add after the existing `paintRectangle` function (before `fillArrayWithNoise`):

```c
static uint8_t clip255(int v){
  return (uint8_t)(v<0 ? 0 : (v>255 ? 255 : v));
}

/* BT.601-style fixed point RGB<->YUV conversion. Not required to be bit-exact:
   only used to give synthetic test shapes a consistent, visually distinct
   color across every pixel format, and for the optional PPM dump. */
static void rgbToYuv(uint8_t r, uint8_t g, uint8_t b, uint8_t* y, uint8_t* u, uint8_t* v){
  int yy = (77*r + 150*g + 29*b) >> 8;
  int uu = 128 + ((-43*(int)r - 85*(int)g + 128*(int)b) >> 8);
  int vv = 128 + ((128*(int)r - 107*(int)g - 21*(int)b) >> 8);
  *y = clip255(yy);
  *u = clip255(uu);
  *v = clip255(vv);
}

static void yuvToRgb(uint8_t y, uint8_t u, uint8_t v, uint8_t* r, uint8_t* g, uint8_t* b){
  int d = (int)u - 128;
  int e = (int)v - 128;
  *r = clip255((int)y + ((91881*e) >> 16));
  *g = clip255((int)y - ((22554*d + 46802*e) >> 16));
  *b = clip255((int)y + ((116130*d) >> 16));
}

void setPixelRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y,
                 uint8_t r, uint8_t g, uint8_t b){
  if(x<0 || y<0 || x>=fi->width || y>=fi->height) return;
  if(fi->pFormat < PF_PACKED){
    uint8_t yy,uu,vv;
    rgbToYuv(r,g,b,&yy,&uu,&vv);
    frame->data[0][y*frame->linesize[0] + x] = yy;
    if(fi->planes >= 3){
      int cx = x >> vsGetPlaneWidthSubS(fi,1);
      int cy = y >> vsGetPlaneHeightSubS(fi,1);
      frame->data[1][cy*frame->linesize[1] + cx] = uu;
      frame->data[2][cy*frame->linesize[2] + cx] = vv;
    }
  }else{
    uint8_t* p = frame->data[0] + y*frame->linesize[0] + x*fi->bytesPerPixel;
    switch(fi->pFormat){
     case PF_RGB24: p[0]=r; p[1]=g; p[2]=b; break;
     case PF_BGR24: p[0]=b; p[1]=g; p[2]=r; break;
     case PF_RGBA:  p[0]=r; p[1]=g; p[2]=b; p[3]=255; break;
     default: break;
    }
  }
}

void getPixelRGB(const VSFrame* frame, const VSFrameInfo* fi, int x, int y,
                 uint8_t* r, uint8_t* g, uint8_t* b){
  if(fi->pFormat < PF_PACKED){
    uint8_t yy = frame->data[0][y*frame->linesize[0] + x];
    uint8_t uu = 128, vv = 128;
    if(fi->planes >= 3){
      int cx = x >> vsGetPlaneWidthSubS(fi,1);
      int cy = y >> vsGetPlaneHeightSubS(fi,1);
      uu = frame->data[1][cy*frame->linesize[1] + cx];
      vv = frame->data[2][cy*frame->linesize[2] + cx];
    }
    yuvToRgb(yy,uu,vv,r,g,b);
  }else{
    const uint8_t* p = frame->data[0] + y*frame->linesize[0] + x*fi->bytesPerPixel;
    switch(fi->pFormat){
     case PF_RGB24: *r=p[0]; *g=p[1]; *b=p[2]; break;
     case PF_BGR24: *b=p[0]; *g=p[1]; *r=p[2]; break;
     case PF_RGBA:  *r=p[0]; *g=p[1]; *b=p[2]; break;
     default: *r=*g=*b=0; break;
    }
  }
}

void fillFrameRGB(VSFrame* frame, const VSFrameInfo* fi, uint8_t r, uint8_t g, uint8_t b){
  int x,y;
  for(y=0; y<fi->height; y++)
    for(x=0; x<fi->width; x++)
      setPixelRGB(frame, fi, x, y, r, g, b);
}

void paintCircleRGB(VSFrame* frame, const VSFrameInfo* fi, int cx, int cy, int radius,
                    uint8_t r, uint8_t g, uint8_t b){
  int x,y;
  int r2 = radius*radius;
  for(y=cy-radius; y<=cy+radius; y++){
    for(x=cx-radius; x<=cx+radius; x++){
      int dx=x-cx, dy=y-cy;
      if(dx*dx+dy*dy <= r2)
        setPixelRGB(frame, fi, x, y, r, g, b);
    }
  }
}

void paintSquareRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y, int size,
                    uint8_t r, uint8_t g, uint8_t b){
  int i,j;
  for(j=y; j<y+size; j++)
    for(i=x; i<x+size; i++)
      setPixelRGB(frame, fi, i, j, r, g, b);
}

int storePPMImage(const char* filename, const VSFrame* frame, const VSFrameInfo* fi){
  FILE* f = fopen(filename, "wb");
  int x,y;
  if(!f){
    vs_log_error("TEST", "Can't open image file '%s'", filename);
    return 0;
  }
  fprintf(f, "P6\n# CREATOR test suite of vid.stab\n%i %i\n255\n", fi->width, fi->height);
  for(y=0; y<fi->height; y++){
    for(x=0; x<fi->width; x++){
      uint8_t rgb[3];
      getPixelRGB(frame, fi, x, y, &rgb[0], &rgb[1], &rgb[2]);
      if(fwrite(rgb, 3, 1, f) != 1){
        vs_log_error("TEST", "Can't write to image file '%s'", filename);
        fclose(f);
        return 0;
      }
    }
  }
  fclose(f);
  return 1;
}
```

- [ ] **Step 3: Create `tests/test_synthetic.c` with the pixel-helper roundtrip test**

```c
static const VSPixelFormat SYN_FORMATS[6] = {
  PF_GRAY8, PF_YUV420P, PF_YUV422P, PF_YUV444P, PF_RGB24, PF_RGBA
};
#define SYN_NUM_FORMATS 6

static const char* synFormatName(VSPixelFormat pf){
  switch(pf){
   case PF_GRAY8:    return "PF_GRAY8";
   case PF_YUV420P:  return "PF_YUV420P";
   case PF_YUV422P:  return "PF_YUV422P";
   case PF_YUV444P:  return "PF_YUV444P";
   case PF_RGB24:    return "PF_RGB24";
   case PF_RGBA:     return "PF_RGBA";
   default:          return "?";
  }
}

static void test_synthetic_pixelhelpers(void){
  int fmt;
  fprintf(stderr, "--- synthetic pixel helper roundtrip ---\n");
  for(fmt=0; fmt<SYN_NUM_FORMATS; fmt++){
    VSFrameInfo fi;
    VSFrame frame;
    uint8_t r,g,b;
    int isGray = (SYN_FORMATS[fmt] == PF_GRAY8);

    test_bool(vsFrameInfoInit(&fi, 16, 16, SYN_FORMATS[fmt]) != 0);
    vsFrameAllocate(&frame, &fi);
    test_bool(!vsFrameIsNull(&frame));

    fillFrameRGB(&frame, &fi, 90, 90, 90);
    setPixelRGB(&frame, &fi, 4, 4, 230, 60, 40);

    getPixelRGB(&frame, &fi, 4, 4, &r, &g, &b);
    fprintf(stderr, "%s: wrote (230,60,40) read (%i,%i,%i)\n",
            synFormatName(SYN_FORMATS[fmt]), r, g, b);
    if(!isGray){
      test_bool(abs((int)r-230) < 30);
      test_bool(abs((int)g-60)  < 30);
      test_bool(abs((int)b-40)  < 30);
    }

    getPixelRGB(&frame, &fi, 0, 0, &r, &g, &b);
    test_bool(abs((int)r-90) < 10 && abs((int)g-90) < 10 && abs((int)b-90) < 10);

    vsFrameFree(&frame);
  }
}

void test_synthetic_circles(void){
  test_synthetic_pixelhelpers();
}
```

- [ ] **Step 4: Wire into `tests/tests.c`**

Add the include after `#include "test_packed.c"`:

```c
#include "test_packed.c"
#include "test_synthetic.c"
```

Add the CLI flag after the `--testPK` block:

```c
  if(all || contains(argv,argc,"--testPK", "packed pixel formats")){
    UNIT(test_packed());
  }

  if(all || contains(argv,argc,"--testSYN", "synthetic circles across pixel formats")){
    UNIT(test_synthetic_circles());
  }
```

- [ ] **Step 5: Build and run to verify it passes**

```bash
mkdir -p tests/build && cd tests/build && cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null && make -j"$(nproc)"
./tests --testSYN
```

Expected: build succeeds, `UNIT TEST test_synthetic_circles()` prints `PASSED`, all 6 formats show a roundtrip read close to `(230,60,40)` (or the gray-only luma check for `PF_GRAY8`).

- [ ] **Step 6: Commit**

```bash
git add tests/testutils.h tests/testutils.c tests/test_synthetic.c tests/tests.c
git commit -m "tests: add pixel-format-agnostic RGB drawing helpers"
```

---

## Task 2: Circle sequence generator + content sanity check

**Files:**
- Create: `tests/generate_synthetic.c`
- Modify: `tests/tests.c` (add one `#include`)
- Modify: `tests/test_synthetic.c` (extend `test_synthetic_circles()`)

**Interfaces:**
- Consumes: `fillFrameRGB`, `paintCircleRGB` (Task 1); `getTestFrameTransform(int)` (existing, in `tests/testutils.c`); `vsTransformGetDefaultConfig`, `vsTransformDataInit`, `vsTransformPrepare`, `vsDoTransform`, `vsTransformFinish`, `vsTransformDataCleanup` (existing public API, `src/transform.h`).
- Produces:
  - `#define SYN_WIDTH 320`, `#define SYN_HEIGHT 240`, `#define SYN_NUM_FRAMES 6` (used by Tasks 3-5).
  - `void generateCircleFrames(VSFrame* frames, VSFrameInfo* fi, VSPixelFormat pf, int width, int height, int numFrames);` — caller-allocated `frames` array of size >= `numFrames`; the function calls `vsFrameInfoInit`/`vsFrameAllocate` for each entry. Caller is responsible for `vsFrameFree`-ing each frame afterwards (same convention as `generateFrames` in `tests/generate.c`).

- [ ] **Step 1: Create `tests/generate_synthetic.c`**

```c
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
```

- [ ] **Step 2: Wire the include into `tests/tests.c`**

```c
#include "generate.c"
#include "generate_synthetic.c"
```

- [ ] **Step 3: Extend `test_synthetic_circles()` in `tests/test_synthetic.c` with a content sanity check**

Replace the body of `test_synthetic_circles()`:

```c
void test_synthetic_circles(void){
  int fmt;
  test_synthetic_pixelhelpers();

  fprintf(stderr, "--- synthetic circle sequence content ---\n");
  for(fmt=0; fmt<SYN_NUM_FORMATS; fmt++){
    VSFrameInfo fi;
    VSFrame frames[SYN_NUM_FRAMES];
    uint8_t r,g,b;
    int i;

    generateCircleFrames(frames, &fi, SYN_FORMATS[fmt], SYN_WIDTH, SYN_HEIGHT, SYN_NUM_FRAMES);

    /* frame 0: background color away from any circle, circle color at a circle center */
    getPixelRGB(&frames[0], &fi, 5, 5, &r, &g, &b);
    test_bool(abs((int)r-SYN_BG_R)<20 && abs((int)g-SYN_BG_G)<20 && abs((int)b-SYN_BG_B)<20);
    getPixelRGB(&frames[0], &fi, SYN_CIRCLES[2].cx, SYN_CIRCLES[2].cy, &r, &g, &b);
    test_bool(abs((int)r-SYN_CIRCLE_R)<30 && abs((int)g-SYN_CIRCLE_G)<30 && abs((int)b-SYN_CIRCLE_B)<30);

    fprintf(stderr, "%s: frame 0 background/circle colors OK\n", synFormatName(SYN_FORMATS[fmt]));

    for(i=0; i<SYN_NUM_FRAMES; i++)
      vsFrameFree(&frames[i]);
  }
}
```

- [ ] **Step 4: Build and run**

```bash
cd tests/build && make -j"$(nproc)" && ./tests --testSYN
```

Expected: PASSED, one "frame 0 background/circle colors OK" line per format.

- [ ] **Step 5: Commit**

```bash
git add tests/generate_synthetic.c tests/tests.c tests/test_synthetic.c
git commit -m "tests: generate synthetic circle sequences across pixel formats"
```

---

## Task 3: Motion-detection correctness check for the circle sequence

**Files:**
- Modify: `tests/test_synthetic.c`

**Interfaces:**
- Consumes: `generateCircleFrames` (Task 2); `vsMotionDetectGetDefaultConfig`, `vsMotionDetectInit`, `vsMotionDetection`, `vsMotionDetectionCleanup` (existing, `src/motiondetect.h`); `vsSimpleMotionsToTransform` (existing, `src/localmotion2transform.h`, signature `VSTransform vsSimpleMotionsToTransform(VSFrameInfo fi, const char* modname, const LocalMotions* motions)`); `mult_transform_`, `sub_transforms`, `storeVSTransform` (existing, `src/transformtype_operations.h`); `getTestFrameTransform` (existing).
- Produces: `static void checkRecoveredMotion(const VSFrameInfo* fi, VSFrame* frames, int numFrames, const char* label, double tolXY, double tolAlpha);` — used again by Task 4.

- [ ] **Step 1: Add `checkRecoveredMotion` and call it from `test_synthetic_circles()`**

Add above `test_synthetic_circles()` in `tests/test_synthetic.c`:

```c
/* Runs vsMotionDetection frame-by-frame (same one-md-instance-per-sequence
   convention as test_motionDetect() in test_motiondetect.c: call once per
   frame starting at frame 0, which establishes the internal reference with
   an expected ~zero motion) and asserts the recovered transform matches
   getTestFrameTransform(i) within tolerance. */
static void checkRecoveredMotion(const VSFrameInfo* fi, VSFrame* frames, int numFrames,
                                 const char* label, double tolXY, double tolAlpha){
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig(label);
  VSMotionDetect md;
  int i;

  test_bool(vsMotionDetectInit(&md, &mdconf, fi) == VS_OK);
  md.conf.numThreads = 1;

  for(i=0; i<numFrames; i++){
    LocalMotions lms;
    VSTransform t, orig, diff;
    int success;

    test_bool(vsMotionDetection(&md, &lms, &frames[i]) == VS_OK);
    t = vsSimpleMotionsToTransform(*fi, label, &lms);
    vs_vector_del(&lms);

    orig = mult_transform_(getTestFrameTransform(i), -1.0);
    diff = sub_transforms(&t, &orig);
    success = fabs(diff.x)<tolXY && fabs(diff.y)<tolXY && fabs(diff.alpha)<tolAlpha;

    fprintf(stderr, "%s frame %i: ", label, i);
    storeVSTransform(stderr, &t);
    if(!success){
      fprintf(stderr, "  Difference: ");
      storeVSTransform(stderr, &diff);
    }
    test_bool(success);
  }
  vsMotionDetectionCleanup(&md);
}
```

Extend the format loop body in `test_synthetic_circles()` (after the existing content-sanity checks, before `vsFrameFree`-ing the frames):

```c
    fprintf(stderr, "%s: frame 0 background/circle colors OK\n", synFormatName(SYN_FORMATS[fmt]));

    checkRecoveredMotion(&fi, frames, SYN_NUM_FRAMES, synFormatName(SYN_FORMATS[fmt]), 2.0, 0.005);

    for(i=0; i<SYN_NUM_FRAMES; i++)
      vsFrameFree(&frames[i]);
```

- [ ] **Step 2: Build and run**

```bash
cd tests/build && make -j"$(nproc)" && ./tests --testSYN
```

Expected: PASSED for all 6 formats. If a specific format fails tolerance, inspect the printed "Difference:" line — first suspect is insufficient circle/background contrast for that format's motion-detection block matching (adjust `SYN_CIRCLE_*`/`SYN_BG_*` constants in `tests/generate_synthetic.c` if needed, rebuild, rerun).

- [ ] **Step 3: Commit**

```bash
git add tests/test_synthetic.c
git commit -m "tests: verify stabilizer recovers injected motion on synthetic circles"
```

---

## Task 4: Circle+squares distractor sequence and its motion-detection test

**Files:**
- Modify: `tests/generate_synthetic.c`
- Modify: `tests/test_synthetic.c`
- Modify: `tests/tests.c`

**Interfaces:**
- Consumes: `paintSquareRGB` (Task 1), `generateCircleFrames` (Task 2), `checkRecoveredMotion` (Task 3).
- Produces: `void generateCircleSquareFrames(VSFrame* frames, VSFrameInfo* fi, VSPixelFormat pf, int width, int height, int numFrames);`, `void test_synthetic_circles_squares(void);`

- [ ] **Step 1: Add the squares generator to `tests/generate_synthetic.c`**

```c
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
```

- [ ] **Step 2: Add `test_synthetic_circles_squares()` to `tests/test_synthetic.c`**

```c
void test_synthetic_circles_squares(void){
  int fmt;
  fprintf(stderr, "--- synthetic circles+squares sequence ---\n");
  for(fmt=0; fmt<SYN_NUM_FORMATS; fmt++){
    VSFrameInfo fi;
    VSFrame frames[SYN_NUM_FRAMES];
    int i;

    generateCircleSquareFrames(frames, &fi, SYN_FORMATS[fmt], SYN_WIDTH, SYN_HEIGHT, SYN_NUM_FRAMES);

    /* squares are distractors: the dominant global motion (background +
       circles) must still be recovered despite their independent motion */
    checkRecoveredMotion(&fi, frames, SYN_NUM_FRAMES, synFormatName(SYN_FORMATS[fmt]), 2.0, 0.005);

    for(i=0; i<SYN_NUM_FRAMES; i++)
      vsFrameFree(&frames[i]);
  }
}
```

- [ ] **Step 3: Wire the new flag into `tests/tests.c`**

```c
  if(all || contains(argv,argc,"--testSYN", "synthetic circles across pixel formats")){
    UNIT(test_synthetic_circles());
  }

  if(all || contains(argv,argc,"--testSYNSQ", "synthetic circles+squares across pixel formats")){
    UNIT(test_synthetic_circles_squares());
  }
```

- [ ] **Step 4: Build and run**

```bash
cd tests/build && make -j"$(nproc)" && ./tests --testSYNSQ
```

Expected: PASSED for all 6 formats. If a format fails, the tolerance may need loosening for this test specifically (per the spec, this is expected to need empirical tuning) — widen the `tolXY`/`tolAlpha` arguments passed from `test_synthetic_circles_squares()` only (leave `test_synthetic_circles()`'s tolerance untouched), rebuild, rerun.

- [ ] **Step 5: Run the full suite to check for regressions**

```bash
cd tests/build && ./tests --all
```

Expected: `UNIT TESTs succeeded: N/N` with no failures (N should be 2 more than the pre-existing baseline count).

- [ ] **Step 6: Commit**

```bash
git add tests/generate_synthetic.c tests/test_synthetic.c tests/tests.c
git commit -m "tests: add circles+squares distractor sequence and its motion test"
```

---

## Task 5: Optional PPM dump for visual inspection

**Files:**
- Modify: `tests/generate_synthetic.c`
- Modify: `tests/tests.c`
- Create: `.gitignore` (repo root)

**Interfaces:**
- Consumes: `storePPMImage` (Task 1), `generateCircleFrames`/`generateCircleSquareFrames` (Tasks 2/4).
- Produces: `void dumpFramesAsPPM(const VSFrame* frames, const VSFrameInfo* fi, int numFrames, const char* prefix);`

- [ ] **Step 1: Add the dump helper to `tests/generate_synthetic.c`**

```c
void dumpFramesAsPPM(const VSFrame* frames, const VSFrameInfo* fi, int numFrames,
                     const char* prefix){
  int i;
  char name[512];
  for(i=0; i<numFrames; i++){
    sprintf(name, "%s%03i.ppm", prefix, i);
    test_bool(storePPMImage(name, &frames[i], fi));
  }
}
```

- [ ] **Step 2: Add the `--dumpSynthetic` flag to `tests/tests.c`**

Add near the other optional/manual flags (after the `--testSYNSQ` block):

```c
  if(contains(argv,argc,"--dumpSynthetic", "dump synthetic frames as PPM for visual inspection")){
    int fmt;
    char dir[256], prefix[512];
    for(fmt=0; fmt<SYN_NUM_FORMATS; fmt++){
      VSFrameInfo fi;
      VSFrame frames[SYN_NUM_FRAMES];
      int i;

      sprintf(dir, "testdata/synthetic/%s", synFormatName(SYN_FORMATS[fmt]));
      sprintf(prefix, "mkdir -p %s", dir);
      system(prefix);

      generateCircleFrames(frames, &fi, SYN_FORMATS[fmt], SYN_WIDTH, SYN_HEIGHT, SYN_NUM_FRAMES);
      sprintf(prefix, "%s/circles", dir);
      dumpFramesAsPPM(frames, &fi, SYN_NUM_FRAMES, prefix);
      for(i=0; i<SYN_NUM_FRAMES; i++) vsFrameFree(&frames[i]);

      generateCircleSquareFrames(frames, &fi, SYN_FORMATS[fmt], SYN_WIDTH, SYN_HEIGHT, SYN_NUM_FRAMES);
      sprintf(prefix, "%s/circles_squares", dir);
      dumpFramesAsPPM(frames, &fi, SYN_NUM_FRAMES, prefix);
      for(i=0; i<SYN_NUM_FRAMES; i++) vsFrameFree(&frames[i]);

      fprintf(stderr, "dumped synthetic PPM frames to %s\n", dir);
    }
  }
```

- [ ] **Step 3: Create `.gitignore` at the repo root**

The repo currently has no `.gitignore`; the worktree already accumulated an untracked local CMake build directory during this plan's testing (`tests/build/`). Add one covering that and the new dump output:

```gitignore
tests/build/
tests/testdata/
```

- [ ] **Step 4: Build, dump, and manually inspect**

```bash
cd tests/build && make -j"$(nproc)"
cd .. && ./build/tests --dumpSynthetic
ls testdata/synthetic/PF_RGB24/
```

Expected: `circles000.ppm` .. `circles005.ppm` and `circles_squares000.ppm` .. `circles_squares005.ppm` under each of the 6 `testdata/synthetic/<format>/` directories. Open a couple of `.ppm` files (e.g. `testdata/synthetic/PF_RGB24/circles000.ppm` and `circles003.ppm`) in an image viewer to confirm: uniform gray background, 5 orange circles in the first frame that have visibly rotated/translated by frame 3, and (in the `circles_squares` set) additional blue squares that have moved independently between frames.

- [ ] **Step 5: Run the full suite one more time**

```bash
cd tests/build && ./tests --all
```

Expected: all tests still pass; `--dumpSynthetic` is intentionally excluded from `--all` (manual-inspection-only, matches the existing `--store`/`--load` convention).

- [ ] **Step 6: Commit**

```bash
git add tests/generate_synthetic.c tests/tests.c .gitignore
git commit -m "tests: add optional PPM dump of synthetic frames for visual inspection"
```

---

## Self-Review Notes

- **Spec coverage:** background+circles across 6 formats (Task 2), clean translate/rotate/zoom motion via `getTestFrameTransform` (Task 2), squares distractor variant with fixed-seed-per-square constant random velocity (Task 4), motion-detection correctness tests (Tasks 3 & 4), optional PPM dump (Task 5), worktree isolation (done prior to Task 1).
- **Type consistency:** `generateCircleFrames`/`generateCircleSquareFrames` signatures match their call sites in `test_synthetic.c` and the `--dumpSynthetic` block; `checkRecoveredMotion` takes `const VSFrameInfo*` consistently in both callers.
- **Tolerance note:** Task 4's tolerance is called out as possibly needing empirical loosening — this is intentional per the spec ("tolerance may be loosened during implementation if empirically needed") and is not a placeholder; the mechanism (separate `tolXY`/`tolAlpha` args to the shared `checkRecoveredMotion`) is fully specified.
