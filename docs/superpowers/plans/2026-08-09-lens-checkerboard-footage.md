# Checkerboard lens footage and pixel round trip — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate a synthetic checkerboard clip distorted by a known lens, and
assert that the render path's lens correction takes it back to the undistorted
original — while producing the PPMs used as the documentation figure.

**Architecture:** The scene is an *analytic* function of continuous image
coordinates, not a painted bitmap. Every frame is evaluated directly at its own
final coordinates through `pattern(S_i(U_k(x)))`, so the footage carries no
resampling blur and the ground truth is exact. The correction is then run
through the public transform API with the known `k` and the exact inverse
shake, and the result must be the base image again.

**Tech Stack:** C99, the existing `tests/` harness (`testframework.h`,
`testutils.h`), CMake. No new dependencies.

Design: `docs/superpowers/specs/2026-08-09-lens-checkerboard-footage-design.md`

## Global Constraints

- Build and run the tests from `build/tests-debug`, configured with plain
  `cmake ../../tests` — **no** `-DCMAKE_BUILD_TYPE=Release`. Release adds
  `-ffast-math` (`tests/CMakeLists.txt:81`) which breaks three pre-existing
  tests. Baseline on this branch is **68/68 UNIT tests** in the default build.
- C99 with the file style already in `tests/`: two-space indent, no tabs,
  `/* */` comments, declarations at the top of a block.
- New `.c` files under `tests/` are `#include`d from `tests.c`. **Do not touch
  `tests/CMakeLists.txt`** — it lists only `tests.c`, which pulls the rest in.
- Every path written to must go through `testOut()` (`tests/testutils.h:61`),
  so nothing lands outside `testout/`.
- Symbol prefix for everything new: `lc` / `LC_`. `ld` belongs to
  `test_lensdistortion.c` and `lm` to `test_lensmap.c`.
- Commit with `git -c user.email=georg.martius@web.de commit`, and end each
  message with `Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>`.
- No tolerance constant may be chosen by guessing. Implement, print the
  measured value, then set the constant with a clear margin and record the
  measured number in a comment beside it.

## Reference: the two maps this plan composes

Both are already in the tree; the plan only ever *transcribes* them.

**The render path's backward affine**, from `src/transformfloat.c:352-387`,
for the luma plane of a frame whose source and destination sizes are equal:

```
z      = 1 - t.zoom/100
M_t(x) = z * A(t.alpha) * (x - c) + c - (t.x, t.y)
A(a)   = [[cos a, -sin a], [sin a, cos a]]        (standard CCW rotation)
c      = (width/2, height/2)                       note: w/2, NOT (w-1)/2
```

Read it off the code as follows: `zcos_a = z*cos(-alpha)` and
`zsin_a = z*sin(-alpha)`, and line 386 computes
`x_s - c = zcos_a*x_d1 + zsin_a*y_d1`, i.e. `z*(cos a * x_d1 - sin a * y_d1)`
— the first row of `z*A(a)`.

**Its inverse, for `t.zoom == 0`** (which is all this plan needs):

```
M_t^-1 = M_t'  with   t'.alpha = -t.alpha
                      (t'.x, t'.y) = -A(-t.alpha) * (t.x, t.y)
                      t'.zoom = 0
```

**The lens**, from `src/lensdistortion.c:35-45`: `vsLensDistortionInit(fi, k)`
uses centre `(w/2, h/2)` and `rho` = half the diagonal. `src/lensmap.c:62-69`
uses the *identical* centre and `rho`, which is why a generator built on
`vsLensUndistortPoint()` composes exactly with the render path's map.

## File Structure

- **Create `tests/generate_lensclip.c`** — the pattern function, the
  supersampled renderer, the shake path, and the clip generator. No assertions;
  it is a generator, mirroring the role of `generate_synthetic.c`.
- **Create `tests/test_lenscorrect_roundtrip.c`** — every assertion, plus the
  comparison metrics and the dump helpers.
- **Modify `tests/tests.c`** — two `#include`s, one `--testLCR` unit block, one
  `--dumpLensClip` dump block.

---

### Task 1: The pattern and the supersampled renderer

**Files:**
- Create: `tests/generate_lensclip.c`
- Create: `tests/test_lenscorrect_roundtrip.c`
- Modify: `tests/tests.c`

**Interfaces:**
- Consumes: `setPixelRGB`, `getPixelRGB` (`tests/testutils.h`), `vsFrameAllocate`,
  `vsFrameInfoInit`, `test_bool` (`tests/testframework.h`).
- Produces:
  - `int lcPatternTone(double x, double y)` → 0 or 1
  - `void lcToneRGB(int tone, uint8_t* rgb)`
  - `typedef void (*LCPointMap)(const void* ctx, double xd, double yd, double* xs, double* ys)`
  - `void lcRenderMapped(VSFrame* frame, const VSFrameInfo* fi, LCPointMap map, const void* ctx)`
  - `void lcIdentityMap(const void* ctx, double xd, double yd, double* xs, double* ys)`
  - constants `LC_WIDTH LC_HEIGHT LC_CELL LC_BAND LC_SS`
  - `void test_lenscorrect_pattern(void)`

- [ ] **Step 1: Write the failing test**

Create `tests/test_lenscorrect_roundtrip.c` with exactly this content:

```c
/*
 * test_lenscorrect_roundtrip.c
 *
 *  Image-level test of the render path's lens correction: a synthetic
 *  checkerboard clip is distorted by a known k, corrected back, and compared
 *  against the analytic original.  The generator lives in generate_lensclip.c.
 *
 *  See docs/superpowers/specs/2026-08-09-lens-checkerboard-footage-design.md
 */

/* --- the pattern ---------------------------------------------------------- */

/* Cell centres alternate tone with both indices, and the tone flips again in
   every other radial band.  Checked at hand-computed points so a sign error in
   either term cannot hide behind the other. */
static void test_lenscorrect_pattern_tones(void){
  double cx = LC_WIDTH/2.0, cy = LC_HEIGHT/2.0;
  int t00, t10, t01;

  fprintf(stderr, "--- lens clip pattern: cells and bands ---\n");

  /* Cells, sampled at the centre of the frame's centre cell and its two
     neighbours.  Adjacent cells must differ; diagonal neighbours must match. */
  t00 = lcPatternTone(cx + 0.5*LC_CELL, cy + 0.5*LC_CELL);
  t10 = lcPatternTone(cx + 1.5*LC_CELL, cy + 0.5*LC_CELL);
  t01 = lcPatternTone(cx + 0.5*LC_CELL, cy + 1.5*LC_CELL);
  test_bool(t10 != t00);
  test_bool(t01 != t00);
  test_bool(lcPatternTone(cx + 1.5*LC_CELL, cy + 1.5*LC_CELL) == t00);

  /* Bands.  Two points in the same cell column and row parity but either side
     of the first band boundary (r = LC_BAND) must differ, purely from the band
     term.  Walk along +x from the centre, staying inside one cell by choosing
     LC_BAND to fall inside a cell -- assert that precondition too, so the test
     fails loudly rather than silently if LC_CELL or LC_BAND is retuned. */
  {
    double rIn = LC_BAND - 2.0, rOut = LC_BAND + 2.0;
    test_bool(floor((cx + rIn)/LC_CELL) == floor((cx + rOut)/LC_CELL));
    test_bool(lcPatternTone(cx + rIn, cy + 0.5) !=
              lcPatternTone(cx + rOut, cy + 0.5));
  }

  /* Deterministic and side-effect free. */
  test_bool(lcPatternTone(cx + 0.5*LC_CELL, cy + 0.5*LC_CELL) == t00);
}

/* A pixel wholly inside one cell must come out as that exact tone; a pixel
   straddling a cell edge must come out strictly between the two tones.  That
   is what says the supersampling actually runs -- with LC_SS == 1 the second
   assertion fails. */
static void test_lenscorrect_pattern_supersampling(void){
  VSFrameInfo fi;
  VSFrame f;
  uint8_t rgb[3], want[3];
  int xEdge, yFlat;

  fprintf(stderr, "--- lens clip pattern: supersampled edges ---\n");

  test_bool(vsFrameInfoInit(&fi, LC_WIDTH, LC_HEIGHT, PF_RGB24) != 0);
  vsFrameAllocate(&f, &fi);
  lcRenderMapped(&f, &fi, lcIdentityMap, NULL);

  /* x = 128 is a cell boundary (a multiple of LC_CELL), and the pixel there
     covers [127.5, 128.5) so it straddles.  x = 144, y = 144 sits in the
     interior of one cell in both axes, and its radius from the centre (200.5)
     is 20 px clear of the nearest band boundary at 180, so neither term is
     near a transition.  lcRenderMapped treats integer index x as coordinate
     x, so the pixel centre IS 144.0 -- no half-pixel offset when asking the
     pattern what should be there. */
  xEdge = (int)(4*LC_CELL);            /* 128, a cell boundary column */
  yFlat = (int)(4*LC_CELL + LC_CELL/2);/* 144, mid-cell                */
  getPixelRGB(&f, &fi, xEdge + (int)(LC_CELL/2), yFlat, &rgb[0], &rgb[1], &rgb[2]);
  lcToneRGB(lcPatternTone(xEdge + LC_CELL/2, yFlat), want);
  test_bool(rgb[0] == want[0] && rgb[1] == want[1] && rgb[2] == want[2]);

  /* The pixel straddling the boundary column: strictly between the tones in
     the red channel, which differs by 215 between them. */
  getPixelRGB(&f, &fi, xEdge, yFlat, &rgb[0], &rgb[1], &rgb[2]);
  test_bool(rgb[0] > 25 && rgb[0] < 240);

  vsFrameFree(&f);
}

void test_lenscorrect_pattern(void){
  test_lenscorrect_pattern_tones();
  test_lenscorrect_pattern_supersampling();
}
```

Add to `tests/tests.c`, immediately after the existing
`#include "generate_synthetic.c"` line:

```c
#include "generate_lensclip.c"
```

and immediately after the existing `#include "test_lensmap.c"` line:

```c
#include "test_lenscorrect_roundtrip.c"
```

and a new unit block immediately after the closing brace of the `--testLMAP`
block (which ends just before the `--testBASE` block, `tests/tests.c:233`):

```c
  if(all || contains(argv,argc,"--testLCR", "checkerboard lens clip round trip")){
    UNIT(test_lenscorrect_pattern());
  }
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
cd build/tests-debug && make -j8 tests
```
Expected: FAIL to compile, `lcPatternTone` / `lcRenderMapped` / `LC_WIDTH`
undeclared. That is the failing state for this task — the generator does not
exist yet.

- [ ] **Step 3: Write the minimal implementation**

Create `tests/generate_lensclip.c`:

```c
/*
 * generate_lensclip.c
 *
 *  A synthetic clip for the image-level lens correction test: a checkerboard
 *  whose two tones swap inside every other radial band, distorted by a known
 *  lens and shaken by a known camera path.
 *
 *  The scene is an analytic function of continuous image coordinates rather
 *  than a painted bitmap, and every frame is evaluated directly at its own
 *  final coordinates.  No source image is ever resampled to build the footage,
 *  so the clip carries no generation blur and the ground truth for the round
 *  trip is the pattern function itself.
 *
 *  See docs/superpowers/specs/2026-08-09-lens-checkerboard-footage-design.md
 */

#define LC_WIDTH   640
#define LC_HEIGHT  480
#define LC_CELL    32.0   /* checkerboard cell size in px: 20x15 cells       */
#define LC_BAND    60.0   /* radial inversion period in px; half the frame
                             diagonal is 400, so ~6 boundaries cross the
                             picture.  Not a divisor or multiple of LC_CELL,
                             so band and cell edges stay distinguishable.    */
#define LC_SS      4      /* supersamples per axis per pixel                 */

/* Both tones are coloured, not grey: the PPM dumps double as the figure in
   docs/lens-distortion.md.  Their luma differs enough (~57 vs ~200) that the
   clip would also clear motion detection's contrast threshold, though nothing
   here depends on that. */
static const uint8_t LC_TONE[2][3] = {
  { 25,  35,  90},   /* dark blue  */
  {240, 205,  90}    /* light amber */
};

static void lcToneRGB(int tone, uint8_t* rgb){
  rgb[0] = LC_TONE[tone][0];
  rgb[1] = LC_TONE[tone][1];
  rgb[2] = LC_TONE[tone][2];
}

/* Which of the two tones the ideal scene shows at continuous point (x,y).

   The checkerboard term gives straight lines, which distortion visibly bends
   -- but only far from the centre, where the bend has room to accumulate.
   The band term gives circles, which distortion moves purely radially: what
   changes is the *spacing* between successive boundaries, compressed at the
   edge under barrel and stretched under pincushion.  That signal is present
   at every radius, including where the line bending is invisible. */
static int lcPatternTone(double x, double y){
  double dx = x - LC_WIDTH/2.0;
  double dy = y - LC_HEIGHT/2.0;
  int cell = ((int)floor(x/LC_CELL) + (int)floor(y/LC_CELL)) & 1;
  int band = ((int)floor(sqrt(dx*dx + dy*dy)/LC_BAND)) & 1;
  return cell ^ band;
}

/* A backward map from destination pixel (xd,yd) to the scene point it shows.
   ctx carries whatever the particular map needs; it is const because no map
   here mutates it. */
typedef void (*LCPointMap)(const void* ctx, double xd, double yd,
                           double* xs, double* ys);

static void lcIdentityMap(const void* ctx, double xd, double yd,
                          double* xs, double* ys){
  (void)ctx;
  *xs = xd; *ys = yd;
}

/* Renders pattern(map(x,y)) into frame, averaging LC_SS x LC_SS subsamples
   over each destination pixel's footprint so that edges are antialiased
   instead of stair-stepped.

   The supersampling is done in DESTINATION space and each subsample is mapped
   individually, which is what keeps the antialiasing correct where the map
   compresses the picture -- averaging in scene space would not.

   Pixel (x,y) covers [x-0.5, x+0.5) x [y-0.5, y+0.5); the render path's own
   loops treat integer index x as coordinate x (transformfloat.c:378), so the
   two conventions agree. */
static void lcRenderMapped(VSFrame* frame, const VSFrameInfo* fi,
                           LCPointMap map, const void* ctx){
  int x, y, i, j;
  for(y=0; y<fi->height; y++){
    for(x=0; x<fi->width; x++){
      int acc[3] = {0,0,0};
      for(j=0; j<LC_SS; j++){
        for(i=0; i<LC_SS; i++){
          double sx, sy;
          uint8_t rgb[3];
          double ox = x + (i + 0.5)/LC_SS - 0.5;
          double oy = y + (j + 0.5)/LC_SS - 0.5;
          map(ctx, ox, oy, &sx, &sy);
          lcToneRGB(lcPatternTone(sx, sy), rgb);
          acc[0] += rgb[0]; acc[1] += rgb[1]; acc[2] += rgb[2];
        }
      }
      setPixelRGB(frame, fi, x, y,
                  (uint8_t)((acc[0] + LC_SS*LC_SS/2)/(LC_SS*LC_SS)),
                  (uint8_t)((acc[1] + LC_SS*LC_SS/2)/(LC_SS*LC_SS)),
                  (uint8_t)((acc[2] + LC_SS*LC_SS/2)/(LC_SS*LC_SS)));
    }
  }
}
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
cd build/tests-debug && make -j8 tests && ./tests --testLCR
```
Expected: PASS, `Tests checks succeeded: 8/8`.

If the band assertion's precondition fails (`floor((cx+rIn)/LC_CELL) ==
floor((cx+rOut)/LC_CELL)`), the first band boundary happens to fall on a cell
edge — nudge `LC_BAND` by 2 px and re-run rather than weakening the assertion.

- [ ] **Step 5: Commit**

```bash
git add tests/generate_lensclip.c tests/test_lenscorrect_roundtrip.c tests/tests.c
git -c user.email=georg.martius@web.de commit -m "test: analytic checkerboard pattern with radial inversion bands

The pattern is a function of continuous coordinates rather than a painted
bitmap, so every frame of the clip to come can be evaluated directly at its
own final coordinates -- no resampling, and the ground truth stays exact.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: The distorted, shaken clip

**Files:**
- Modify: `tests/generate_lensclip.c`
- Modify: `tests/test_lenscorrect_roundtrip.c`
- Modify: `tests/tests.c:~236` (the `--testLCR` block)

**Interfaces:**
- Consumes: `lcPatternTone`, `lcRenderMapped`, `LCPointMap`, `lcIdentityMap`,
  `LC_WIDTH`, `LC_HEIGHT`, `LC_BAND` from Task 1;
  `vsLensDistortionInit`, `vsLensUndistortPoint` (`src/lensdistortion.h`);
  `null_transform` (`src/transformtype.h`).
- Produces:
  - `#define LC_NUM_FRAMES 6`, `#define LC_K_BARREL (-0.25)`, `#define LC_K_PIN 0.15`
  - `VSTransform lcClipTransform(int i)`
  - `void lcBackwardAffine(const VSTransform* t, double xd, double yd, double* xs, double* ys)`
  - `VSTransform lcInverseTransform(VSTransform t)`
  - `typedef struct { VSLensDistortion ld; VSTransform t; int useLens; } LCClipMapCtx;`
  - `void lcClipMap(const void* ctx, double xd, double yd, double* xs, double* ys)`
  - `void lcGenerateClip(VSFrame* frames, VSFrameInfo* fi, VSPixelFormat pf, double k, int numFrames)`
  - `void test_lenscorrect_generator(void)`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_lenscorrect_roundtrip.c`, before nothing in particular —
put it after `test_lenscorrect_pattern()`:

```c
/* --- the clip generator --------------------------------------------------- */

/* Returns the x of the first pixel at or right of the frame centre, along the
   horizontal centre line, at which the tone changes -- i.e. the radius of the
   nearest band or cell boundary beyond `from`.  Used to measure where the
   pattern's features actually landed in a rendered frame. */
static int lcFirstEdgeRight(const VSFrame* f, const VSFrameInfo* fi, int from){
  int x, cy = fi->height/2;
  uint8_t r0,g0,b0, r,g,b;
  getPixelRGB(f, fi, from, cy, &r0,&g0,&b0);
  for(x=from+1; x<fi->width-1; x++){
    getPixelRGB(f, fi, x, cy, &r,&g,&b);
    if(abs((int)r - (int)r0) > 100) return x;
  }
  return -1;
}

/* With k == 0 and an identity camera pose, the generated frame must be the
   base image bit for bit: the generator's map collapses to the identity and
   nothing but the pattern function is left. */
static void test_lenscorrect_generator_null(void){
  VSFrameInfo fi;
  VSFrame base, frames[LC_NUM_FRAMES];
  int x, y, i, diff = 0;

  fprintf(stderr, "--- lens clip generator: k=0, identity pose ---\n");

  test_bool(vsFrameInfoInit(&fi, LC_WIDTH, LC_HEIGHT, PF_RGB24) != 0);
  vsFrameAllocate(&base, &fi);
  lcRenderMapped(&base, &fi, lcIdentityMap, NULL);

  lcGenerateClip(frames, &fi, PF_RGB24, 0.0, LC_NUM_FRAMES);

  /* frame 0 only: lcClipTransform(0) is the identity by construction, so with
     k == 0 the whole map is the identity. */
  for(y=0; y<fi.height; y++)
    for(x=0; x<fi.width; x++){
      uint8_t a[3], b[3];
      getPixelRGB(&base,      &fi, x, y, &a[0],&a[1],&a[2]);
      getPixelRGB(&frames[0], &fi, x, y, &b[0],&b[1],&b[2]);
      if(a[0]!=b[0] || a[1]!=b[1] || a[2]!=b[2]) diff++;
    }
  test_bool(diff == 0);

  vsFrameFree(&base);
  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

/* The physical signature of barrel distortion: the band boundaries, which are
   evenly spaced circles in the ideal scene, get pulled inward and their
   spacing compresses with radius.  Measured on frame 0, whose pose is the
   identity, so this is the lens and nothing else.

   This is the assertion that would catch a sign error in the generator -- a
   generator that applied D instead of U would stretch the spacing instead. */
static void test_lenscorrect_generator_barrel_compresses(void){
  VSFrameInfo fi;
  VSFrame frames[LC_NUM_FRAMES];
  int cx, e1, e2, e3, i;

  fprintf(stderr, "--- lens clip generator: barrel compresses band spacing ---\n");

  lcGenerateClip(frames, &fi, PF_RGB24, LC_K_BARREL, LC_NUM_FRAMES);
  cx = fi.width/2;

  /* Three successive tone changes walking right from the centre. */
  e1 = lcFirstEdgeRight(&frames[0], &fi, cx + 2);
  test_bool(e1 > 0);
  e2 = lcFirstEdgeRight(&frames[0], &fi, e1 + 2);
  test_bool(e2 > 0);
  e3 = lcFirstEdgeRight(&frames[0], &fi, e2 + 2);
  test_bool(e3 > 0);

  fprintf(stderr, "  barrel edges at %i %i %i (gaps %i %i)\n",
          e1, e2, e3, e2-e1, e3-e2);
  /* Each successive gap is smaller than the one before it. */
  test_bool(e3 - e2 < e2 - e1);

  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

/* Pincushion must do the opposite.  Without this, a generator that compressed
   the spacing for every k would pass the test above. */
static void test_lenscorrect_generator_pincushion_stretches(void){
  VSFrameInfo fi;
  VSFrame frames[LC_NUM_FRAMES];
  int cx, e1, e2, e3, i;

  fprintf(stderr, "--- lens clip generator: pincushion stretches band spacing ---\n");

  lcGenerateClip(frames, &fi, PF_RGB24, LC_K_PIN, LC_NUM_FRAMES);
  cx = fi.width/2;
  e1 = lcFirstEdgeRight(&frames[0], &fi, cx + 2);
  test_bool(e1 > 0);
  e2 = lcFirstEdgeRight(&frames[0], &fi, e1 + 2);
  test_bool(e2 > 0);
  e3 = lcFirstEdgeRight(&frames[0], &fi, e2 + 2);
  test_bool(e3 > 0);

  fprintf(stderr, "  pincushion edges at %i %i %i (gaps %i %i)\n",
          e1, e2, e3, e2-e1, e3-e2);
  test_bool(e3 - e2 > e2 - e1);

  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

/* lcInverseTransform must invert the render path's backward affine exactly.
   Checked numerically by composing the two maps at a scatter of points, which
   is independent of how the inverse is derived. */
static void test_lenscorrect_inverse_transform(void){
  int i, j;
  double pts[5][2] = {{0,0},{639,479},{320,240},{10,470},{500,30}};

  fprintf(stderr, "--- inverse of the render backward affine ---\n");

  for(i=1; i<LC_NUM_FRAMES; i++){
    VSTransform t  = lcClipTransform(i);
    VSTransform ti = lcInverseTransform(t);
    for(j=0; j<5; j++){
      double ax, ay, bx, by;
      lcBackwardAffine(&ti, pts[j][0], pts[j][1], &ax, &ay);
      lcBackwardAffine(&t,  ax, ay, &bx, &by);
      test_bool(fabs(bx - pts[j][0]) < 1e-9);
      test_bool(fabs(by - pts[j][1]) < 1e-9);
    }
  }
}

void test_lenscorrect_generator(void){
  test_lenscorrect_generator_null();
  test_lenscorrect_generator_barrel_compresses();
  test_lenscorrect_generator_pincushion_stretches();
  test_lenscorrect_inverse_transform();
}
```

Add to the `--testLCR` block in `tests/tests.c`:

```c
    UNIT(test_lenscorrect_generator());
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
cd build/tests-debug && make -j8 tests
```
Expected: FAIL to compile — `lcGenerateClip`, `lcClipTransform`,
`lcInverseTransform`, `lcBackwardAffine`, `LC_NUM_FRAMES`, `LC_K_BARREL`,
`LC_K_PIN` undeclared.

- [ ] **Step 3: Write the minimal implementation**

Append to `tests/generate_lensclip.c`:

```c
#define LC_NUM_FRAMES 6
#define LC_K_BARREL (-0.25)   /* the clip that gets dumped as the figure */
#define LC_K_PIN      0.15    /* assertion-only second pass             */

/* The camera path.  Bounded and drift free -- |x| <= 12, |y| <= 9,
   |alpha| <= 1.2 deg -- so the scene never leaves the canvas and the valid
   region stays large.  Every term vanishes at i == 0, deliberately: frame 0 is
   then the base scene seen through the lens and nothing else, which is what
   makes it the frame worth putting in the figure and what lets the Wobble
   assertion compare against it directly. */
static VSTransform lcClipTransform(int i){
  double s = (double)i;
  VSTransform t = null_transform();
  t.x     = 12.0 * sin(s * 0.7);
  t.y     =  9.0 * sin(s * 0.9);
  t.alpha =  1.2 * sin(s * 0.5) * M_PI / 180.0;
  t.zoom  = 0;
  return t;
}

/* The render path's own backward affine, transcribed from
   transformfloat.c:352-387 for a plane at full luma resolution and equal
   source and destination sizes:

     z      = 1 - t.zoom/100
     M_t(x) = z * A(alpha) * (x - c) + c - (t.x, t.y),  A the CCW rotation

   The code spells the rotation as cos(-alpha)/sin(-alpha) with the signs of
   the second row flipped, which is the same matrix.  Centre is w/2, NOT
   (w-1)/2, matching c_d_x there. */
static void lcBackwardAffine(const VSTransform* t, double xd, double yd,
                             double* xs, double* ys){
  double cx = LC_WIDTH/2.0, cy = LC_HEIGHT/2.0;
  double z  = 1.0 - t->zoom/100.0;
  double ca = cos(t->alpha), sa = sin(t->alpha);
  double dx = xd - cx, dy = yd - cy;
  *xs = z*(ca*dx - sa*dy) + cx - t->x;
  *ys = z*(sa*dx + ca*dy) + cy - t->y;
}

/* The transform whose backward affine is the inverse of t's.

   From M_t(x) = A(a)(x-c) + c - d  (with zoom 0, so z = 1),
        M_t^-1(y) = A(-a)((y-c) + d) + c = A(-a)(y-c) + c + A(-a)d,
   which is M_t' with alpha' = -a and d' = -A(-a)d.

   Only the zoom-free case is implemented, because lcClipTransform never sets
   zoom; the general case would additionally need z' = 1/z and
   zoom' = 100(1 - 1/z).  Asserting rather than silently mis-inverting. */
static VSTransform lcInverseTransform(VSTransform t){
  VSTransform r = null_transform();
  double ca = cos(-t.alpha), sa = sin(-t.alpha);
  test_bool(t.zoom == 0);
  r.alpha = -t.alpha;
  r.x = -( ca*t.x - sa*t.y);
  r.y = -( sa*t.x + ca*t.y);
  r.zoom = 0;
  return r;
}

/* Context for the clip's backward map: which pose, and which lens. */
typedef struct {
  VSLensDistortion ld;
  VSTransform      t;
  int              useLens;   /* 0 leaves U_k out, for the k == 0 case */
} LCClipMapCtx;

/* frame_i(x) = pattern( S_i( U_k(x) ) ): undistort the destination point to
   where an ideal lens would have put it, then move it by the camera pose.

   U_k comes from lensdistortion.c, which test_lensdistortion.c covers
   independently.  vsLensPlaneMapInit and its lookup tables are deliberately
   NOT used here -- building the footage with the same tables the round trip
   then checks would only confirm that the tables agree with themselves.

   Points outside the model's domain cannot occur for k <= 0, and for the
   pincushion pass they would sit far outside the frame; the pattern is
   defined everywhere, so the fallback simply leaves the point where it is. */
static void lcClipMap(const void* ctx, double xd, double yd,
                      double* xs, double* ys){
  const LCClipMapCtx* c = (const LCClipMapCtx*)ctx;
  double ux = xd, uy = yd;
  if(c->useLens)
    if(vsLensUndistortPoint(&c->ld, xd, yd, &ux, &uy) != VS_OK){ ux = xd; uy = yd; }
  lcBackwardAffine(&c->t, ux, uy, xs, ys);
}

/* Allocates and fills numFrames frames of the clip.  Caller frees them. */
static void lcGenerateClip(VSFrame* frames, VSFrameInfo* fi, VSPixelFormat pf,
                           double k, int numFrames){
  int i;
  test_bool(vsFrameInfoInit(fi, LC_WIDTH, LC_HEIGHT, pf) != 0);
  for(i=0; i<numFrames; i++){
    LCClipMapCtx ctx;
    ctx.ld      = vsLensDistortionInit(fi, k);
    ctx.t       = lcClipTransform(i);
    ctx.useLens = (k != 0.0);
    vsFrameAllocate(&frames[i], fi);
    lcRenderMapped(&frames[i], fi, lcClipMap, &ctx);
  }
}
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
cd build/tests-debug && make -j8 tests && ./tests --testLCR
```
Expected: PASS. The printed edge positions should show clearly shrinking gaps
for barrel and growing gaps for pincushion. If either gap comparison is a tie,
the three sampled boundaries are too close to the centre where the effect is
weakest — start the walk further out (`cx + LC_BAND`) rather than loosening
the comparison.

- [ ] **Step 5: Commit**

```bash
git add tests/generate_lensclip.c tests/test_lenscorrect_roundtrip.c tests/tests.c
git -c user.email=georg.martius@web.de commit -m "test: distorted, shaken checkerboard clip generator

Frame i is pattern(S_i(U_k(x))), evaluated analytically.  It uses
vsLensUndistortPoint but deliberately not vsLensPlaneMapInit: generating the
footage from the same lookup tables the round trip checks would confirm only
that the tables agree with themselves.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: Comparison metrics and the Full-mode round trip

**Files:**
- Modify: `tests/test_lenscorrect_roundtrip.c`
- Modify: `tests/tests.c:~236`

**Interfaces:**
- Consumes: everything from Tasks 1 and 2; `vsTransformGetDefaultConfig`,
  `vsTransformDataInit`, `vsTransformSetLensK`, `vsTransformPrepare`,
  `vsDoTransform`, `vsTransformFinish`, `vsTransformDataCleanup`
  (`src/transform.h`); `vsLensDistortPoint` (`src/lensdistortion.h`);
  `VSLensCorrectOff/Wobble/Full` (`src/lensmap.h`).
- Produces:
  - `typedef struct { int n, nFlat, maxFlat; double psnr; } LCCompare;`
  - `void lcValidMask(unsigned char* valid, const VSFrameInfo* fi, const VSLensDistortion* ld, const VSTransform* t, VSLensCorrectMode mode)`
  - `LCCompare lcCompare(const VSFrame* got, const VSFrame* ideal, const VSFrameInfo* fi, const unsigned char* valid)`
  - `void lcCorrect(VSFrame* out, const VSFrame* in, const VSFrameInfo* fi, VSLensCorrectMode mode, double k, VSTransform t)`
  - `void test_lenscorrect_roundtrip_full(void)`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_lenscorrect_roundtrip.c`:

```c
/* --- the round trip ------------------------------------------------------- */

/* Correcting frame i with the known k and the exact inverse of its pose must
   return the base image:

     out(x) = frame_i( D_k( M_t'(x) ) )
            = pattern( S_i( U_k( D_k( M_t'(x) ) ) ) )
            = pattern( M_t( M_t'(x) ) )  =  pattern(x)

   Bilinear resampling of a hard edge is off by up to half a step AT that edge
   however correct the geometry is, so the comparison is split: an exact
   tolerance on pixels whose neighbourhood in the ideal image is uniform, and
   a PSNR floor over the whole valid region to catch gross geometric error in
   the textured parts. */
static void test_lenscorrect_full_recovers_base(void){
  VSFrameInfo fi;
  VSFrame base, out, frames[LC_NUM_FRAMES];
  unsigned char* valid;
  VSLensDistortion ld;
  int i;

  fprintf(stderr, "--- Full correction recovers the base image ---\n");

  lcGenerateClip(frames, &fi, PF_RGB24, LC_K_BARREL, LC_NUM_FRAMES);
  ld = vsLensDistortionInit(&fi, LC_K_BARREL);
  vsFrameAllocate(&base, &fi);
  vsFrameAllocate(&out,  &fi);
  lcRenderMapped(&base, &fi, lcIdentityMap, NULL);
  valid = (unsigned char*)malloc(fi.width*fi.height);
  test_bool(valid != NULL);

  for(i=0; i<LC_NUM_FRAMES; i++){
    VSTransform ti = lcInverseTransform(lcClipTransform(i));
    LCCompare c;
    lcCorrect(&out, &frames[i], &fi, VSLensCorrectFull, LC_K_BARREL, ti);
    lcValidMask(valid, &fi, &ld, &ti, VSLensCorrectFull);
    c = lcCompare(&out, &base, &fi, valid);
    fprintf(stderr, "  frame %i: valid %i, flat %i, maxFlat %i, PSNR %.2f dB\n",
            i, c.n, c.nFlat, c.maxFlat, c.psnr);
    /* A mask that had collapsed to almost nothing would make the rest of the
       assertions vacuous, so pin its size first. */
    test_bool(c.n    > fi.width*fi.height/2);
    test_bool(c.nFlat > c.n/4);
    test_bool(c.maxFlat <= LC_MAX_FLAT_DELTA);
    test_bool(c.psnr    >= LC_MIN_PSNR);
  }

  free(valid);
  vsFrameFree(&base); vsFrameFree(&out);
  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

/* The control that stops all of the above from passing with the lens map
   stubbed out to a no-op: the same clip, same inverse pose, correction OFF.
   The distortion is then still in the picture and the score must collapse. */
static void test_lenscorrect_off_is_much_worse(void){
  VSFrameInfo fi;
  VSFrame base, outOn, outOff, frames[LC_NUM_FRAMES];
  unsigned char* valid;
  VSLensDistortion ld;
  VSTransform ti;
  LCCompare on, off;
  int i;

  fprintf(stderr, "--- control: correction off scores far worse ---\n");

  lcGenerateClip(frames, &fi, PF_RGB24, LC_K_BARREL, LC_NUM_FRAMES);
  ld = vsLensDistortionInit(&fi, LC_K_BARREL);
  vsFrameAllocate(&base, &fi);
  vsFrameAllocate(&outOn, &fi);
  vsFrameAllocate(&outOff, &fi);
  lcRenderMapped(&base, &fi, lcIdentityMap, NULL);
  valid = (unsigned char*)malloc(fi.width*fi.height);
  test_bool(valid != NULL);

  ti = lcInverseTransform(lcClipTransform(0));
  lcCorrect(&outOn,  &frames[0], &fi, VSLensCorrectFull, LC_K_BARREL, ti);
  lcCorrect(&outOff, &frames[0], &fi, VSLensCorrectOff,  LC_K_BARREL, ti);
  /* Both scored over the SAME mask -- the one the corrected run uses -- so the
     comparison is of picture quality, not of how much of the frame each run
     happened to fill. */
  lcValidMask(valid, &fi, &ld, &ti, VSLensCorrectFull);
  on  = lcCompare(&outOn,  &base, &fi, valid);
  off = lcCompare(&outOff, &base, &fi, valid);
  fprintf(stderr, "  PSNR on %.2f dB, off %.2f dB\n", on.psnr, off.psnr);
  test_bool(on.psnr > off.psnr + 6.0);
  test_bool(off.maxFlat > LC_MAX_FLAT_DELTA);

  free(valid);
  vsFrameFree(&base); vsFrameFree(&outOn); vsFrameFree(&outOff);
  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

void test_lenscorrect_roundtrip_full(void){
  test_lenscorrect_full_recovers_base();
  test_lenscorrect_off_is_much_worse();
}
```

Add to the `--testLCR` block in `tests/tests.c`:

```c
    UNIT(test_lenscorrect_roundtrip_full());
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
cd build/tests-debug && make -j8 tests
```
Expected: FAIL to compile — `LCCompare`, `lcCompare`, `lcValidMask`,
`lcCorrect`, `LC_MAX_FLAT_DELTA`, `LC_MIN_PSNR` undeclared.

- [ ] **Step 3: Write the minimal implementation**

Insert into `tests/test_lenscorrect_roundtrip.c`, *above* the tests just added
(the helpers must be declared before use — this file has no forward
declarations, matching the style of `test_lensmap.c`):

```c
/* Placeholders; Step 5 replaces them with values measured from the run. */
#define LC_MAX_FLAT_DELTA 2
#define LC_MIN_PSNR       20.0

typedef struct {
  int    n;        /* pixels inside the valid mask                          */
  int    nFlat;    /* of those, pixels whose ideal neighbourhood is uniform  */
  int    maxFlat;  /* largest per-channel |delta| over the flat pixels       */
  double psnr;     /* over the whole valid mask, all three channels          */
} LCCompare;

/* Runs one frame through the render path with the given mode, k and pose. */
static void lcCorrect(VSFrame* out, const VSFrame* in, const VSFrameInfo* fi,
                      VSLensCorrectMode mode, double k, VSTransform t){
  VSTransformData td;
  VSTransformConfig cfg = vsTransformGetDefaultConfig("lensclip-test");
  cfg.interpolType   = VS_BiLinear;
  cfg.crop           = VSCropBorder;
  cfg.optZoom        = 0;
  cfg.lensCorrection = mode;
  test_bool(vsTransformDataInit(&td, &cfg, fi, fi) == VS_OK);
  vsTransformSetLensK(&td, k);
  test_bool(vsTransformPrepare(&td, in, out) == VS_OK);
  test_bool(vsDoTransform(&td, t) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);
  vsTransformDataCleanup(&td);
}

/* Marks the destination pixels whose backward map lands inside the source
   frame with room for the bilinear tap, computed from the same composition
   the render path uses:

     Off    x_src = M_t(x)
     Full   x_src = D_k(M_t(x))
     Wobble x_src = D_k(M_t(U_k(x)))

   Everything else is border fill on one side or the other and carries no
   information.  This is exact rather than an inset rectangle, which matters:
   under barrel correction the invalid region is a curved apron whose depth
   varies along the edge, and a rectangle big enough to contain it would throw
   away a large part of the picture. */
static void lcValidMask(unsigned char* valid, const VSFrameInfo* fi,
                        const VSLensDistortion* ld, const VSTransform* t,
                        VSLensCorrectMode mode){
  const double margin = 2.0;
  int x, y;
  for(y=0; y<fi->height; y++)
    for(x=0; x<fi->width; x++){
      double ax = x, ay = y, sx, sy;
      int ok = 1;
      if(mode == VSLensCorrectWobble)
        ok = (vsLensUndistortPoint(ld, x, y, &ax, &ay) == VS_OK);
      if(ok) lcBackwardAffine(t, ax, ay, &sx, &sy);
      if(ok && mode != VSLensCorrectOff){
        double dx, dy;
        ok = (vsLensDistortPoint(ld, sx, sy, &dx, &dy) == VS_OK);
        sx = dx; sy = dy;
      }
      if(ok)
        ok = sx >= margin && sy >= margin &&
             sx <= fi->width-1-margin && sy <= fi->height-1-margin;
      valid[y*fi->width + x] = (unsigned char)(ok ? 1 : 0);
    }
}

/* True when every pixel of the 5x5 neighbourhood of (x,y) in `ideal` carries
   the identical colour.  5x5 rather than 3x3 because the map's local scale is
   not exactly 1: near the frame edge under k = -0.25 it compresses by up to
   about 1.3x, so an output neighbourhood draws on a slightly larger source
   neighbourhood, and the wider window keeps the guarantee honest.

   Using exact equality rather than a gradient threshold means there is no
   magic number to tune here: inside a checkerboard cell the pattern is
   constant to the byte, and any antialiased edge pixel fails the test. */
static int lcIsFlat(const VSFrame* ideal, const VSFrameInfo* fi, int x, int y){
  uint8_t r0,g0,b0, r,g,b;
  int i, j;
  if(x < 2 || y < 2 || x >= fi->width-2 || y >= fi->height-2) return 0;
  getPixelRGB(ideal, fi, x, y, &r0,&g0,&b0);
  for(j=-2; j<=2; j++)
    for(i=-2; i<=2; i++){
      getPixelRGB(ideal, fi, x+i, y+j, &r,&g,&b);
      if(r!=r0 || g!=g0 || b!=b0) return 0;
    }
  return 1;
}

static LCCompare lcCompare(const VSFrame* got, const VSFrame* ideal,
                           const VSFrameInfo* fi, const unsigned char* valid){
  LCCompare c;
  double se = 0;
  int x, y;
  c.n = c.nFlat = c.maxFlat = 0;
  c.psnr = 0;
  for(y=0; y<fi->height; y++)
    for(x=0; x<fi->width; x++){
      uint8_t a[3], b[3];
      int ch, flat;
      if(!valid[y*fi->width + x]) continue;
      getPixelRGB(got,   fi, x, y, &a[0],&a[1],&a[2]);
      getPixelRGB(ideal, fi, x, y, &b[0],&b[1],&b[2]);
      c.n++;
      flat = lcIsFlat(ideal, fi, x, y);
      if(flat) c.nFlat++;
      for(ch=0; ch<3; ch++){
        int d = (int)a[ch] - (int)b[ch];
        se += (double)d*d;
        if(flat && abs(d) > c.maxFlat) c.maxFlat = abs(d);
      }
    }
  if(c.n > 0){
    double mse = se / (3.0*c.n);
    c.psnr = mse > 0 ? 10.0*log10(255.0*255.0/mse) : 99.0;
  }
  return c;
}
```

- [ ] **Step 4: Run and read off the real numbers**

Run:
```bash
cd build/tests-debug && make -j8 tests && ./tests --testLCR 2>&1 | grep -E "frame|PSNR"
```

The placeholders in Step 3 are almost certainly wrong. Record the worst
`maxFlat` and the worst `PSNR` printed across all six frames.

- [ ] **Step 5: Set the tolerances from the measurement**

Replace the two placeholder defines with values that clear the measured worst
case with a clear margin, and record what was measured:

```c
/* Measured worst case over the six frames at k = -0.25: maxFlat = <M>,
   PSNR = <P> dB.  Set with margin; the `off` control scores <O> dB, so the
   floor is nowhere near loose enough to let an uncorrected frame through. */
#define LC_MAX_FLAT_DELTA <M rounded up, plus a little>
#define LC_MIN_PSNR       <P rounded down, minus a little>
```

Then re-run and confirm both the round trip and the control still pass:
```bash
cd build/tests-debug && make -j8 tests && ./tests --testLCR
```
Expected: PASS.

If `maxFlat` comes back large (say above 10) the geometry is wrong, not the
tolerance — do **not** raise the constant to accommodate it. Check the sign of
`lcInverseTransform` and that `lcBackwardAffine` matches
`transformfloat.c:386` term for term.

- [ ] **Step 6: Commit**

```bash
git add tests/test_lenscorrect_roundtrip.c tests/tests.c
git -c user.email=georg.martius@web.de commit -m "test: Full lens correction returns the clip to the base image

Split comparison: exact tolerance on pixels whose ideal neighbourhood is
uniform, PSNR floor over the whole valid region.  Bilinear resampling of a
hard edge is off by half a step at that edge whatever the geometry does, so a
single flat per-pixel tolerance would be either vacuous or false.

The correction-off control is what keeps the rest of it honest.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: Wobble mode, 4:2:0 chroma, and pincushion

**Files:**
- Modify: `tests/test_lenscorrect_roundtrip.c`
- Modify: `tests/tests.c:~236`

**Interfaces:**
- Consumes: everything from Tasks 1-3.
- Produces: `void test_lenscorrect_roundtrip_modes(void)`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_lenscorrect_roundtrip.c`:

```c
/* Wobble removes the shake but leaves the lens in place:

     out(x) = frame_i( D_k( M_t'( U_k(x) ) ) ) = pattern( U_k(x) )

   and pattern(U_k(x)) is exactly frame 0, because lcClipTransform(0) is the
   identity.  So every corrected frame must match frame 0 of the clip -- the
   assertion the mode's whole promise rests on, that a lens-corrected shot of a
   still camera is unchanged.

   Note frame 0 of this loop is trivially exact: an identity transform in
   Wobble mode takes the memcpy fast path (transformfloat.c:331), so nothing
   is resampled.  Frames 1..5 are the real test.  The Full-mode loop in Task 3
   has no such hole -- that same line excludes Full from the fast path
   explicitly, so its i == 0 case walks the inner loop like the rest. */
static void test_lenscorrect_wobble_holds_the_lens(void){
  VSFrameInfo fi;
  VSFrame out, frames[LC_NUM_FRAMES];
  unsigned char* valid;
  VSLensDistortion ld;
  int i;

  fprintf(stderr, "--- Wobble removes the shake, keeps the lens ---\n");

  lcGenerateClip(frames, &fi, PF_RGB24, LC_K_BARREL, LC_NUM_FRAMES);
  ld = vsLensDistortionInit(&fi, LC_K_BARREL);
  vsFrameAllocate(&out, &fi);
  valid = (unsigned char*)malloc(fi.width*fi.height);
  test_bool(valid != NULL);

  for(i=0; i<LC_NUM_FRAMES; i++){
    VSTransform ti = lcInverseTransform(lcClipTransform(i));
    LCCompare c;
    lcCorrect(&out, &frames[i], &fi, VSLensCorrectWobble, LC_K_BARREL, ti);
    lcValidMask(valid, &fi, &ld, &ti, VSLensCorrectWobble);
    /* compared against frame 0 of the clip, not against the base */
    c = lcCompare(&out, &frames[0], &fi, valid);
    fprintf(stderr, "  frame %i: valid %i, flat %i, maxFlat %i, PSNR %.2f dB\n",
            i, c.n, c.nFlat, c.maxFlat, c.psnr);
    test_bool(c.n     > fi.width*fi.height/2);
    test_bool(c.nFlat > c.n/4);
    test_bool(c.maxFlat <= LC_MAX_FLAT_DELTA);
    test_bool(c.psnr    >= LC_MIN_PSNR);
  }

  free(valid);
  vsFrameFree(&out);
  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

/* Pincushion through the same Full round trip.  A map that only ever handled
   the barrel sign would pass everything above. */
static void test_lenscorrect_pincushion_roundtrip(void){
  VSFrameInfo fi;
  VSFrame base, out, frames[LC_NUM_FRAMES];
  unsigned char* valid;
  VSLensDistortion ld;
  int i;

  fprintf(stderr, "--- Full correction, pincushion k = +0.15 ---\n");

  lcGenerateClip(frames, &fi, PF_RGB24, LC_K_PIN, LC_NUM_FRAMES);
  ld = vsLensDistortionInit(&fi, LC_K_PIN);
  vsFrameAllocate(&base, &fi);
  vsFrameAllocate(&out,  &fi);
  lcRenderMapped(&base, &fi, lcIdentityMap, NULL);
  valid = (unsigned char*)malloc(fi.width*fi.height);
  test_bool(valid != NULL);

  for(i=0; i<LC_NUM_FRAMES; i++){
    VSTransform ti = lcInverseTransform(lcClipTransform(i));
    LCCompare c;
    lcCorrect(&out, &frames[i], &fi, VSLensCorrectFull, LC_K_PIN, ti);
    lcValidMask(valid, &fi, &ld, &ti, VSLensCorrectFull);
    c = lcCompare(&out, &base, &fi, valid);
    fprintf(stderr, "  frame %i: valid %i, flat %i, maxFlat %i, PSNR %.2f dB\n",
            i, c.n, c.nFlat, c.maxFlat, c.psnr);
    test_bool(c.n     > fi.width*fi.height/3);
    test_bool(c.nFlat > c.n/4);
    test_bool(c.maxFlat <= LC_MAX_FLAT_DELTA);
    test_bool(c.psnr    >= LC_MIN_PSNR);
  }

  free(valid);
  vsFrameFree(&base); vsFrameFree(&out);
  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

/* The same Full round trip on 4:2:0, where the chroma planes get their own
   half-resolution lens map.  That per-plane map is the part of lensmap.c most
   likely to be wrong, and no RGB test can reach it.

   Its own tolerances: the frames are built with setPixelRGB, which writes one
   pixel's chroma per 2x2 block, and getPixelRGB reads it back nearest
   neighbour, so chroma is quantised on both sides of the comparison in a way
   the packed path never is.  Luma is unaffected. */
static void test_lenscorrect_full_yuv420(void){
  VSFrameInfo fi;
  VSFrame base, out, frames[LC_NUM_FRAMES];
  unsigned char* valid;
  VSLensDistortion ld;
  int i;

  fprintf(stderr, "--- Full correction on PF_YUV420P ---\n");

  lcGenerateClip(frames, &fi, PF_YUV420P, LC_K_BARREL, LC_NUM_FRAMES);
  ld = vsLensDistortionInit(&fi, LC_K_BARREL);
  vsFrameAllocate(&base, &fi);
  vsFrameAllocate(&out,  &fi);
  lcRenderMapped(&base, &fi, lcIdentityMap, NULL);
  valid = (unsigned char*)malloc(fi.width*fi.height);
  test_bool(valid != NULL);

  for(i=0; i<LC_NUM_FRAMES; i++){
    VSTransform ti = lcInverseTransform(lcClipTransform(i));
    LCCompare c;
    lcCorrect(&out, &frames[i], &fi, VSLensCorrectFull, LC_K_BARREL, ti);
    lcValidMask(valid, &fi, &ld, &ti, VSLensCorrectFull);
    c = lcCompare(&out, &base, &fi, valid);
    fprintf(stderr, "  frame %i: valid %i, flat %i, maxFlat %i, PSNR %.2f dB\n",
            i, c.n, c.nFlat, c.maxFlat, c.psnr);
    test_bool(c.n     > fi.width*fi.height/2);
    test_bool(c.nFlat > c.n/4);
    test_bool(c.maxFlat <= LC_MAX_FLAT_DELTA_420);
    test_bool(c.psnr    >= LC_MIN_PSNR_420);
  }

  free(valid);
  vsFrameFree(&base); vsFrameFree(&out);
  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}

void test_lenscorrect_roundtrip_modes(void){
  test_lenscorrect_wobble_holds_the_lens();
  test_lenscorrect_pincushion_roundtrip();
  test_lenscorrect_full_yuv420();
}
```

Add to the `--testLCR` block in `tests/tests.c`:

```c
    UNIT(test_lenscorrect_roundtrip_modes());
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
cd build/tests-debug && make -j8 tests
```
Expected: FAIL to compile — `LC_MAX_FLAT_DELTA_420` and `LC_MIN_PSNR_420`
undeclared.

- [ ] **Step 3: Add the 4:2:0 tolerance placeholders**

Next to `LC_MAX_FLAT_DELTA` in the same file:

```c
/* Placeholders; Step 5 replaces them with values measured from the run. */
#define LC_MAX_FLAT_DELTA_420 2
#define LC_MIN_PSNR_420       20.0
```

- [ ] **Step 4: Run and read off the real numbers**

Run:
```bash
cd build/tests-debug && make -j8 tests && ./tests --testLCR 2>&1 | grep -A8 "PF_YUV420P"
```
Record the worst `maxFlat` and worst `PSNR` of the 4:2:0 block. Also check
whether the pincushion and Wobble blocks pass on the Task 3 constants — they
should; if pincushion does not, note the numbers and raise it in review rather
than silently loosening `LC_MIN_PSNR`, which the barrel test also uses.

- [ ] **Step 5: Set the 4:2:0 tolerances from the measurement**

```c
/* Measured worst case on 4:2:0 at k = -0.25: maxFlat = <M>, PSNR = <P> dB.
   Looser than the packed constants because setPixelRGB writes one pixel's
   chroma per 2x2 block and getPixelRGB reads it back nearest neighbour, so
   chroma is quantised on both sides of this comparison. */
#define LC_MAX_FLAT_DELTA_420 <M rounded up, plus a little>
#define LC_MIN_PSNR_420       <P rounded down, minus a little>
```

Run:
```bash
cd build/tests-debug && make -j8 tests && ./tests --testLCR
```
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add tests/test_lenscorrect_roundtrip.c tests/tests.c
git -c user.email=georg.martius@web.de commit -m "test: Wobble, pincushion and 4:2:0 through the same round trip

Wobble is checked against frame 0 of the clip rather than the base image,
because lcClipTransform(0) is the identity: pattern(U_k(x)) IS frame 0.  That
is the mode's promise stated as an assertion.

4:2:0 carries its own tolerances, since the frames are built and read back
through setPixelRGB/getPixelRGB, which quantise chroma on both sides.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: The PPM dumps and the contact sheet

**Files:**
- Modify: `tests/test_lenscorrect_roundtrip.c`
- Modify: `tests/tests.c` (a new dump block after the existing
  `--dumpSynthetic` block, which ends at `tests/tests.c:261`)

**Interfaces:**
- Consumes: everything from Tasks 1-4; `storePPMImage`, `testOut`
  (`tests/testutils.h`).
- Produces: `void lcDumpClip(void)`

- [ ] **Step 1: Write the failing test**

There is nothing to assert about a figure, so this task's verification is the
artefact itself. Add the dump entry point to
`tests/test_lenscorrect_roundtrip.c`:

```c
/* --- the figure ----------------------------------------------------------- */

/* Copies src into dest at (ox,oy).  Both must be PF_RGB24. */
static void lcBlit(VSFrame* dest, const VSFrameInfo* fid, int ox, int oy,
                   const VSFrame* src, const VSFrameInfo* fis){
  int x, y;
  for(y=0; y<fis->height; y++)
    for(x=0; x<fis->width; x++){
      uint8_t r,g,b;
      getPixelRGB(src, fis, x, y, &r,&g,&b);
      setPixelRGB(dest, fid, ox+x, oy+y, r,g,b);
    }
}

/* Writes the clip and its corrections to testout/lensclip/, plus one contact
   sheet -- base | distorted | Full-corrected for frame 0 -- which is the
   figure for docs/lens-distortion.md.  Asserts nothing; it exists to be
   looked at, and is reached only via --dumpLensClip. */
void lcDumpClip(void){
  VSFrameInfo fi, fiSheet;
  VSFrame base, full, wobble, sheet, frames[LC_NUM_FRAMES];
  char name[256];
  int i;

  lcGenerateClip(frames, &fi, PF_RGB24, LC_K_BARREL, LC_NUM_FRAMES);
  vsFrameAllocate(&base,   &fi);
  vsFrameAllocate(&full,   &fi);
  vsFrameAllocate(&wobble, &fi);
  lcRenderMapped(&base, &fi, lcIdentityMap, NULL);

  test_bool(storePPMImage(testOut("lensclip/base.ppm"), &base, &fi));

  for(i=0; i<LC_NUM_FRAMES; i++){
    VSTransform ti = lcInverseTransform(lcClipTransform(i));
    snprintf(name, sizeof(name), "lensclip/distorted_%03i.ppm", i);
    test_bool(storePPMImage(testOut(name), &frames[i], &fi));

    lcCorrect(&full, &frames[i], &fi, VSLensCorrectFull, LC_K_BARREL, ti);
    snprintf(name, sizeof(name), "lensclip/full_%03i.ppm", i);
    test_bool(storePPMImage(testOut(name), &full, &fi));

    lcCorrect(&wobble, &frames[i], &fi, VSLensCorrectWobble, LC_K_BARREL, ti);
    snprintf(name, sizeof(name), "lensclip/wobble_%03i.ppm", i);
    test_bool(storePPMImage(testOut(name), &wobble, &fi));
  }

  /* The contact sheet: three panels of frame 0 side by side, separated by an
     8px gutter left in the mid grey the frame is cleared to. */
  {
    const int gutter = 8;
    test_bool(vsFrameInfoInit(&fiSheet, 3*LC_WIDTH + 2*gutter, LC_HEIGHT,
                              PF_RGB24) != 0);
    vsFrameAllocate(&sheet, &fiSheet);
    fillFrameRGB(&sheet, &fiSheet, 128, 128, 128);
    lcCorrect(&full, &frames[0], &fi, VSLensCorrectFull, LC_K_BARREL,
              lcInverseTransform(lcClipTransform(0)));
    lcBlit(&sheet, &fiSheet, 0,                       0, &base,      &fi);
    lcBlit(&sheet, &fiSheet, LC_WIDTH + gutter,       0, &frames[0], &fi);
    lcBlit(&sheet, &fiSheet, 2*(LC_WIDTH + gutter),   0, &full,      &fi);
    test_bool(storePPMImage(testOut("lensclip/sheet.ppm"), &sheet, &fiSheet));
    vsFrameFree(&sheet);
  }

  fprintf(stderr, "dumped lens clip PPMs to %s/lensclip "
          "(base | distorted | corrected sheet.ppm, k = %.2f)\n",
          TEST_OUTPUT_DIR, LC_K_BARREL);

  vsFrameFree(&base); vsFrameFree(&full); vsFrameFree(&wobble);
  for(i=0; i<LC_NUM_FRAMES; i++) vsFrameFree(&frames[i]);
}
```

Add to `tests/tests.c`, immediately after the closing brace of the
`--dumpSynthetic` block (`tests/tests.c:261`):

```c
  if(contains(argv,argc,"--dumpLensClip",
              "dump the checkerboard lens clip and the contact sheet as PPM")){
    lcDumpClip();
  }
```

- [ ] **Step 2: Run it and verify the artefacts**

Run:
```bash
cd build/tests-debug && make -j8 tests && ./tests --dumpLensClip
ls -l testout/lensclip/
head -c 40 testout/lensclip/sheet.ppm | head -2
```
Expected: 20 files (`base.ppm`, 6 each of `distorted_`/`full_`/`wobble_`, and
`sheet.ppm`). The sheet's PPM header must read `1936 480`
(3*640 + 2*8 = 1936).

- [ ] **Step 3: Look at the contact sheet**

Convert and inspect:
```bash
pnmtopng testout/lensclip/sheet.ppm > /tmp/lensclip-sheet.png 2>/dev/null \
  || convert testout/lensclip/sheet.ppm /tmp/lensclip-sheet.png
```
Read the PNG. Confirm by eye: the left panel's checkerboard lines are straight
and the band rings evenly spaced; the middle panel's lines bow outward and the
rings crowd toward the edge; the right panel looks like the left again, with a
dark curved apron at the border where the source ran out. If the right panel
does **not** match the left, stop and report it — that is a real failure that
the numeric tests somehow missed, not a dump bug.

- [ ] **Step 4: Confirm the normal test run still writes nothing**

Run:
```bash
rm -rf testout/lensclip
cd build/tests-debug && ./tests --all 2>&1 | tail -3
ls testout/lensclip 2>&1
```
Expected: `UNIT TESTs succeeded: 72/72` (68 baseline + the 4 new units), and
`ls` reporting that `testout/lensclip` does not exist — the dumps are reached
only through `--dumpLensClip`.

- [ ] **Step 5: Commit**

```bash
git add tests/test_lenscorrect_roundtrip.c tests/tests.c
git -c user.email=georg.martius@web.de commit -m "test: dump the lens clip and a base|distorted|corrected contact sheet

Reached only through --dumpLensClip, so an ordinary test run still writes
nothing.  The sheet is the figure for docs/lens-distortion.md.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

## Verification

After Task 5, from `build/tests-debug`:

```bash
make -j8 tests && ./tests --all 2>&1 | grep -E "UNIT TESTs succeeded|Test Failed"
```

Expected: `UNIT TESTs succeeded: 72/72`, no `Test Failed` lines.
