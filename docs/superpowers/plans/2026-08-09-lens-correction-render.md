# Lens Correction in the Render Path — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Apply the estimated lens distortion `k` to the rendered output frames, with a default
"wobble" mode that removes only the motion-induced distortion variation and an opt-in "full" mode
that undistorts the picture outright.

**Architecture:** A new `src/lensmap.{c,h}` owns all render-path radial arithmetic and two `r²`
lookup tables. The four warp loops (`transformPlanar` / `transformPacked` × float / fixed-point)
gain an optional `U_k` step before the existing similarity and a `D_k` step after it. The zoom
budget learns to evaluate the real composite map instead of a closed form for similarities.

**Tech Stack:** C99, CMake, the in-tree test framework in `tests/` (test `.c` files are `#include`d
into `tests/tests.c`, not compiled separately).

**Spec:** `docs/superpowers/specs/2026-08-09-lens-correction-render-design.md`. Read it before
starting. Background maths: `docs/lens-distortion.md`.

## Global Constraints

- Language is C99 (`-std=gnu99`), two-space indent, `stroustrup` brace style, the emacs/vim footer
  block at the end of every new `.c`/`.h` file — copy it from `src/lensdistortion.c`.
- Every new source file carries the LGPL-2.1-or-later header block, copied from
  `src/lensdistortion.h`, with `Copyright (C) Georg Martius - 2026`.
- Public symbols are decorated `VS_API` (see `src/vidstab_api.h`) and prefixed `vsLens`.
- New library sources must be added to **both** `CMakeLists.txt` (line 69, the `SOURCES` list) and
  `tests/CMakeLists.txt` (**both** the `tests` and the `bench` `add_executable` lists).
- New test files are `#include`d in `tests/tests.c` alongside the others and registered with a
  `--testXXX` flag in `main`.
- Sign convention: barrel is `k < 0`. Radii are normalised by `rho` = half the **source** frame
  diagonal, so `r = 1` at a source corner.
- `gU(t) = 1/(1 + k*t)` and `gD(t) = 2/(1 + sqrt(1 - 4*k*t))`, with `t = r²`.
- The `k = 0` / lens-inactive path must stay **bit-identical** to today's output. Task 1 builds the
  guard; every later task keeps it green.
- Build and test with:
  ```
  cmake -S tests -B build/tests && cmake --build build/tests -j8
  ./build/tests/tests --all
  ```
  Run from the repository root. `--all` was 43/43 units at the start of this work.
- Commit after every task. Do not squash tasks together.

---

### Task 1: Bit-exactness baseline guard

Before anything else, pin the current output of all four warp loops so later tasks cannot change it
silently.

**Files:**
- Create: `tests/test_transform_baseline.c`
- Modify: `tests/tests.c` (add `#include "test_transform_baseline.c"` after the
  `#include "test_transform.c"` line, and a registration block in `main`)

**Interfaces:**
- Consumes: nothing.
- Produces: `void test_transform_baseline(void)` — a unit test registered as `--testBASE`.
  Later tasks re-run it unchanged.

**Note on golden values:** the checksums are captured on the machine doing the work and are a
regression guard for *this* toolchain, not a cross-platform invariant. The test prints the constants
it computed when they mismatch, so refreshing them after an intentional change is a copy-paste.

- [ ] **Step 1: Write the test file with placeholder goldens**

Create `tests/test_transform_baseline.c`:

```c
/* Pins the output of the warp loops so the lens-correction work cannot change
   the k=0 / lens-inactive path.  The golden CRCs are toolchain specific: the
   test prints what it got, so an intentional change is a copy-paste away. */

static uint32_t tbCrc32(const uint8_t* p, int n){
  static uint32_t tab[256];
  static int init = 0;
  uint32_t c = 0xFFFFFFFFu;
  int i, j;
  if(!init){
    for(i=0; i<256; i++){
      uint32_t v = i;
      for(j=0; j<8; j++) v = (v>>1) ^ (0xEDB88320u & (-(int32_t)(v & 1)));
      tab[i] = v;
    }
    init = 1;
  }
  for(i=0; i<n; i++) c = tab[(c ^ p[i]) & 0xFF] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

static uint32_t tbCrcFrame(const VSFrame* f, const VSFrameInfo* fi){
  uint32_t c = 0;
  int plane;
  for(plane=0; plane<fi->planes; plane++){
    int w = CHROMA_SIZE(fi->width,  vsGetPlaneWidthSubS(fi, plane));
    int h = CHROMA_SIZE(fi->height, vsGetPlaneHeightSubS(fi, plane));
    int y;
    for(y=0; y<h; y++)
      c ^= tbCrc32(f->data[plane] + (size_t)y*f->linesize[plane], w) + 0x9E3779B9u + (c<<6) + (c>>2);
  }
  return c;
}

/* Transforms chosen to exercise: pure integer shift, fractional shift,
   rotation, zoom, and the identity early-out. */
#define TB_NUM_T 5
static VSTransform tbTransform(int i){
  VSTransform t = null_transform();
  switch(i){
   case 0: break;                                             /* identity      */
   case 1: t.x =  7;    t.y = -4;                    break;   /* integer shift */
   case 2: t.x = -3.37; t.y =  5.91;                 break;   /* frac shift    */
   case 3: t.alpha = 0.021;                          break;   /* rotation      */
   case 4: t.x = 2.5; t.y = 1.25; t.alpha = -0.013; t.zoom = 4.0; break;
  }
  return t;
}

/* Runs one transform through the real pipeline and returns the output CRC. */
static uint32_t tbRun(const VSFrame* src, const VSFrameInfo* fi,
                      VSInterpolType ip, int useFloat, int i){
  VSTransformData td;
  VSTransformConfig cfg = vsTransformGetDefaultConfig("baseline");
  VSFrame dest;
  uint32_t crc;
  cfg.interpolType = ip;
  cfg.crop         = VSCropBorder;   /* deterministic: no dependence on history */
  cfg.optZoom      = 0;
  test_bool(vsTransformDataInit(&td, &cfg, fi, fi) == VS_OK);
  vsFrameAllocate(&dest, fi);
  test_bool(vsTransformPrepare(&td, src, &dest) == VS_OK);
  if(useFloat){
    if(fi->pFormat < PF_PACKED) test_bool(_FLT(transformPlanar)(&td, tbTransform(i)) == VS_OK);
    else                        test_bool(_FLT(transformPacked)(&td, tbTransform(i)) == VS_OK);
  }else{
    test_bool(vsDoTransform(&td, tbTransform(i)) == VS_OK);
  }
  test_bool(vsTransformFinish(&td) == VS_OK);
  crc = tbCrcFrame(&dest, fi);
  vsFrameFree(&dest);
  vsTransformDataCleanup(&td);
  return crc;
}

/* Filled in by step 3.  Order: [interpolation 0..3][transform 0..4]. */
static const uint32_t TB_GOLD_FIXED[4][TB_NUM_T] = {{0}};
static const uint32_t TB_GOLD_FLOAT[4][TB_NUM_T] = {{0}};

void test_transform_baseline(void){
  VSFrameInfo fi;
  VSFrame src;
  uint32_t seed = 12345;
  int ip, i, mismatch = 0;
  vsFrameInfoInit(&fi, 320, 240, PF_YUV420P);
  vsFrameAllocate(&src, &fi);
  ldFillTexture(&src, &fi, &seed);
  memset(src.data[1], 0x60, (size_t)src.linesize[1]*(fi.height/2));
  memset(src.data[2], 0xA0, (size_t)src.linesize[2]*(fi.height/2));

  for(ip=0; ip<4; ip++){
    for(i=0; i<TB_NUM_T; i++){
      uint32_t gf = tbRun(&src, &fi, (VSInterpolType)ip, 0, i);
      uint32_t gl = tbRun(&src, &fi, (VSInterpolType)ip, 1, i);
      if(gf != TB_GOLD_FIXED[ip][i] || gl != TB_GOLD_FLOAT[ip][i]) mismatch = 1;
      test_bool(gf == TB_GOLD_FIXED[ip][i]);
      test_bool(gl == TB_GOLD_FLOAT[ip][i]);
    }
  }
  if(mismatch){
    fprintf(stderr, "baseline CRCs changed -- if intentional, paste these in:\n");
    fprintf(stderr, "static const uint32_t TB_GOLD_FIXED[4][TB_NUM_T] = {\n");
    for(ip=0; ip<4; ip++){
      fprintf(stderr, "  {");
      for(i=0; i<TB_NUM_T; i++) fprintf(stderr, "0x%08Xu,", tbRun(&src, &fi, (VSInterpolType)ip, 0, i));
      fprintf(stderr, "},\n");
    }
    fprintf(stderr, "};\nstatic const uint32_t TB_GOLD_FLOAT[4][TB_NUM_T] = {\n");
    for(ip=0; ip<4; ip++){
      fprintf(stderr, "  {");
      for(i=0; i<TB_NUM_T; i++) fprintf(stderr, "0x%08Xu,", tbRun(&src, &fi, (VSInterpolType)ip, 1, i));
      fprintf(stderr, "},\n");
    }
    fprintf(stderr, "};\n");
  }
  vsFrameFree(&src);
}
```

`ldFillTexture` lives in `tests/test_lensdistortion.c`, which `tests.c` includes at line 54 —
**after** where this file will be included. Include `test_transform_baseline.c` *after*
`test_lensdistortion.c` in `tests.c` so the helper is already declared.

- [ ] **Step 2: Register it and build**

In `tests/tests.c`, add after line 54 (`#include "test_lensdistortion.c"`):

```c
#include "test_transform_baseline.c"
```

and in `main`, after the `--testLENS` block:

```c
  if(all || contains(argv,argc,"--testBASE", "warp-loop output baseline (k=0 guard)")){
    UNIT(test_transform_baseline());
  }
```

Run: `cmake -S tests -B build/tests && cmake --build build/tests -j8`
Expected: builds clean.

- [ ] **Step 3: Run it, capture the real goldens**

Run: `./build/tests/tests --testBASE`
Expected: FAILS, and prints two `static const uint32_t ...` blocks.

Replace the two placeholder arrays in `tests/test_transform_baseline.c` with the printed blocks.

- [ ] **Step 4: Verify it now passes, twice**

Run: `./build/tests/tests --testBASE && ./build/tests/tests --testBASE`
Expected: PASSED both times (proves the output is deterministic, not just once-captured).

Run: `./build/tests/tests --all`
Expected: all units pass, one more unit than before.

- [ ] **Step 5: Commit**

```bash
git add tests/test_transform_baseline.c tests/tests.c
git commit -m "test: pin warp-loop output before the lens correction work"
```

---

### Task 2: The `lensmap` unit

Pure geometry and lookup tables. Nothing in the render path changes yet.

**Files:**
- Create: `src/lensmap.h`, `src/lensmap.c`
- Create: `tests/test_lensmap.c`
- Modify: `CMakeLists.txt:69`, `tests/CMakeLists.txt` (both `add_executable` lists), `tests/tests.c`

**Interfaces:**
- Consumes: `VSFrameInfo` (`src/frameinfo.h`), `VSTransform` (`src/transformtype.h`),
  `vsGetPlaneWidthSubS` / `vsGetPlaneHeightSubS` / `CHROMA_SIZE`.
- Produces, and all later tasks depend on these exact names:
  ```c
  typedef enum { VSLensCorrectOff = 0, VSLensCorrectWobble, VSLensCorrectFull } VSLensCorrectMode;
  typedef struct _vslensplanemap VSLensPlaneMap;
  int  vsLensPlaneMapInit(VSLensPlaneMap* m, const VSFrameInfo* fiSrc, const VSFrameInfo* fiDest,
                          int plane, double k, VSLensCorrectMode mode);
  void vsLensPlaneMapFree(VSLensPlaneMap* m);
  int  vsLensMapBackward(const VSLensPlaneMap* m, const VSTransform* t,
                         double xd, double yd, double* xs, double* ys);
  double vsLensScaleUDirect(double k, double t);
  double vsLensScaleDDirect(double k, double t);
  ```
  plus the inline per-pixel helpers `vsLensLutF` (float) and `vsLensLutFp` (16.16) and the constants
  `VS_LENS_LUT_N`, `VS_LENS_OUTSIDE_PX`.

**Design notes fixed here, relied on by every later task:**

- Two tables per plane. `gU` covers `t ∈ [0, tMaxU]` with `tMaxU = 1.2 * (rhoDest/rho)²`; `U_k` is
  only ever applied to destination-centred coordinates, whose radius cannot exceed the destination
  half-diagonal, so this always covers the reachable range. `gD` covers `t ∈ [0, tMaxD]` with
  `tMaxD = min(4.0, 0.99 * tDomD)`.
- `t` above a table's last entry clamps to the last entry. This is provably harmless: the frame
  corner is `r = 1`, so any point with `r > 1` is outside the frame and the sample resolves to the
  border value whatever `g` says. `tMaxD = 4` means `r = 2`.
- Domain edges: `gU` is undefined for `1 + k*t <= 0` (barrel, `t >= -1/k`, unreachable at `t <= 1.2`
  for `|k| < 0.83`); `gD` for `1 - 4*k*t < 0` (pincushion, `t > 1/(4k)`). Store `tDomD`
  (`INFINITY` when `k <= 0`). A sample with `t > tDomD` is written as the sentinel coordinate
  `VS_LENS_OUTSIDE_PX = -30000`, which every interpolator resolves to `def`.
- `VS_LENS_LUT_N = 1024`. Linear interpolation. The accuracy claim holds only on the **in-frame**
  domain: a sample is inside the frame when `r_observed <= 1`, which for `gD` means
  `t <= tIn = 1/(1+k)²`. There the error is under `1e-5` in `g`, i.e. well under a hundredth of a
  pixel. Beyond `tIn` the division model's derivative diverges toward the domain edge and a
  uniform-grid linear table cannot bound the error at any `N` — but those samples map outside the
  frame and resolve to the border value, so the table only has to stay finite, positive and
  monotone there. `gU` has no nearby pole (`t = -1/k >= 3.3` for barrel, `tMaxU = 1.2`) and meets
  the tight bound across its whole table.
- Float tables are built only under `TESTING` (the `_FLT` path is `TESTING`-only); the 16.16 tables
  are always built.

- [ ] **Step 1: Write the header**

Create `src/lensmap.h` (LGPL header block copied from `src/lensdistortion.h`, then):

```c
#ifndef __LENSMAP_H
#define __LENSMAP_H

#include <math.h>
#include <stdint.h>
#include "frameinfo.h"
#include "transformtype.h"
#include "vidstab_api.h"

/** How the estimated lens distortion is applied to the rendered picture.

    With M the stabilising similarity, U_k undistort and D_k distort:

      Off     x_src = M(x_out)                 today's behaviour, bit for bit
      Wobble  x_src = D_k(M(U_k(x_out)))       same lens, camera held still
      Full    x_src = D_k(M(x_out))            ideal lens, straight lines straight

    Wobble collapses to the identity when M does, so it costs no field of view
    and leaves locked-off shots untouched.  See
    docs/superpowers/specs/2026-08-09-lens-correction-render-design.md */
typedef enum {
  VSLensCorrectOff = 0,
  VSLensCorrectWobble,
  VSLensCorrectFull
} VSLensCorrectMode;

#define VS_LENS_LUT_N       1024
/* Sentinel destination for a sample outside the model's domain.  Far enough
   outside any frame that every interpolator returns the border value, small
   enough that iToFp16() of it does not overflow. */
#define VS_LENS_OUTSIDE_PX  (-30000)

/** Everything one plane's inner loop needs to evaluate the radial maps. */
typedef struct _vslensplanemap {
  int      active;      /* 0 -> the caller takes the plain affine path         */
  VSLensCorrectMode mode;
  double   k;
  double   cdx, cdy;    /* destination plane centre, plane units               */
  double   csx, csy;    /* source plane centre, plane units                    */
  double   tMaxU, tMaxD;/* LUT domains in t = r^2                              */
  double   tDomD;       /* t beyond which D_k is undefined; INFINITY if k <= 0 */
  double   invRho2;     /* 1/rho^2 in luma-equivalent pixels^2                 */
  int      sxShift;     /* wsub: plane x units -> luma units is << sxShift     */
  int      syShift;     /* hsub                                                */
  int32_t  idxScaleU;   /* fp32: (N-1)/(tMaxU * rho^2), see vsLensLutFp        */
  int32_t  idxScaleD;
  int32_t* gU;          /* VS_LENS_LUT_N entries, 16.16                        */
  int32_t* gD;
#ifdef TESTING
  float*   gUf;
  float*   gDf;
#endif
} VSLensPlaneMap;

/** g of the division model, evaluated directly in double precision.
    Return values are only meaningful inside the model's domain; the callers
    check the domain themselves. */
static VS_INLINE double vsLensScaleUDirectI(double k, double t){ return 1.0/(1.0 + k*t); }
static VS_INLINE double vsLensScaleDDirectI(double k, double t){ return 2.0/(1.0 + sqrt(1.0 - 4.0*k*t)); }
VS_API double vsLensScaleUDirect(double k, double t);
VS_API double vsLensScaleDDirect(double k, double t);

/** Builds the map for one plane.  Returns VS_ERROR on allocation failure, and
    leaves m->active == 0 when mode is Off or k is zero. */
VS_API int vsLensPlaneMapInit(VSLensPlaneMap* m, const VSFrameInfo* fiSrc,
                              const VSFrameInfo* fiDest, int plane,
                              double k, VSLensCorrectMode mode);
VS_API void vsLensPlaneMapFree(VSLensPlaneMap* m);

/** Double-precision reference for the whole backward map of this plane, used by
    the zoom budget and the tests.  The inner loops inline the same arithmetic
    rather than calling this.  Returns VS_ERROR when the point leaves the
    model's domain, in which case *xs and *ys are set to VS_LENS_OUTSIDE_PX. */
VS_API int vsLensMapBackward(const VSLensPlaneMap* m, const VSTransform* t,
                             double xd, double yd, double* xs, double* ys);

#ifdef TESTING
/** float table lookup; t is r^2 in luma-equivalent units. */
static VS_INLINE float vsLensLutF(const float* tab, double tMax, double t){
  double u = t * ((VS_LENS_LUT_N-1) / tMax);
  int    i;
  float  f;
  if(u >= VS_LENS_LUT_N-1) return tab[VS_LENS_LUT_N-1];
  i = (int)u;
  f = (float)(u - i);
  return tab[i] + (tab[i+1] - tab[i])*f;
}
#endif

/** 16.16 table lookup.  r2fp32 is the squared radius in luma-equivalent
    pixels^2 at scale 2^32 (i.e. the sum of squares of two 16.16 values), and
    idxScale is (N-1)/(tMax*rho^2) at scale 2^32. */
static VS_INLINE int32_t vsLensLutFp(const int32_t* tab, int64_t r2fp32, int32_t idxScale){
  int64_t u = ((r2fp32 >> 16) * (int64_t)idxScale) >> 32;   /* index at 16.16 */
  int32_t i = (int32_t)(u >> 16);
  int32_t f;
  if(i >= VS_LENS_LUT_N-1) return tab[VS_LENS_LUT_N-1];
  f = (int32_t)(u & 0xFFFF);
  return tab[i] + (int32_t)((((int64_t)(tab[i+1] - tab[i])) * f) >> 16);
}

#endif
```

If `VS_INLINE` does not already exist in the tree, use `static inline` directly — check
`src/vidstabdefines.h` first and follow whatever the codebase already uses.

- [ ] **Step 2: Write the failing tests**

Create `tests/test_lensmap.c`:

```c
/* Unit tests for the render-path lens map: the scale functions, the round trip
   that the wobble mode's safety rests on, and the lookup tables. */

static const double LM_KS[] = {-0.3, -0.25, -0.1, 0.1, 0.15};
#define LM_NUM_KS ((int)(sizeof(LM_KS)/sizeof(LM_KS[0])))

/* D_k(U_k(x)) == x.  This is the identity that makes wobble mode a no-op on a
   locked-off shot, so it is checked to full double precision. */
static void test_lensmap_scales(void){
  int i, j;
  for(i=0; i<LM_NUM_KS; i++){
    double k = LM_KS[i];
    for(j=0; j<=20; j++){
      double r = j/20.0;                    /* observed radius, 0..1 */
      double gu, ru, gd;
      gu = vsLensScaleUDirect(k, r*r);
      ru = r*gu;                            /* ideal radius */
      gd = vsLensScaleDDirect(k, ru*ru);
      test_bool(fabs(ru*gd - r) < 1e-12);
    }
  }
  /* k = 0 is exactly the identity, no round-off at all */
  test_bool(vsLensScaleUDirect(0.0, 0.7) == 1.0);
  test_bool(vsLensScaleDDirect(0.0, 0.7) == 1.0);
}

/* The whole backward map collapses to the identity when the transform does. */
static void test_lensmap_identity_transform(void){
  VSFrameInfo fi;
  VSLensPlaneMap m;
  VSTransform t = null_transform();
  int i, x, y;
  vsFrameInfoInit(&fi, 640, 360, PF_YUV420P);
  for(i=0; i<LM_NUM_KS; i++){
    test_bool(vsLensPlaneMapInit(&m, &fi, &fi, 0, LM_KS[i], VSLensCorrectWobble) == VS_OK);
    test_bool(m.active == 1);
    for(y=0; y<fi.height; y+=17){
      for(x=0; x<fi.width; x+=17){
        double xs, ys;
        test_bool(vsLensMapBackward(&m, &t, x, y, &xs, &ys) == VS_OK);
        test_bool(fabs(xs - x) < 1e-9 && fabs(ys - y) < 1e-9);
      }
    }
    vsLensPlaneMapFree(&m);
  }
}

/* The tables agree with direct evaluation everywhere they are indexed. */
static void test_lensmap_lut(void){
  VSFrameInfo fi;
  VSLensPlaneMap m;
  int i, j;
  double worstG = 0, worstPx = 0;
  vsFrameInfoInit(&fi, 1920, 1080, PF_YUV420P);
  for(i=0; i<LM_NUM_KS; i++){
    double k = LM_KS[i];
    test_bool(vsLensPlaneMapInit(&m, &fi, &fi, 0, k, VSLensCorrectWobble) == VS_OK);
    for(j=0; j<4096; j++){
      double tU = m.tMaxU * j/4095.0;
      double tD = m.tMaxD * j/4095.0;
      double eU = fabs(vsLensLutF(m.gUf, m.tMaxU, tU) - vsLensScaleUDirect(k, tU));
      double eD = fabs(vsLensLutF(m.gDf, m.tMaxD, tD) - vsLensScaleDDirect(k, tD));
      double rho = 0.5*sqrt(1920.0*1920.0 + 1080.0*1080.0);
      if(eU > worstG) worstG = eU;
      if(eD > worstG) worstG = eD;
      if(eU*sqrt(tU)*rho > worstPx) worstPx = eU*sqrt(tU)*rho;
      if(eD*sqrt(tD)*rho > worstPx) worstPx = eD*sqrt(tD)*rho;
    }
    vsLensPlaneMapFree(&m);
  }
  fprintf(stderr, "  LUT: worst |dg| = %.3g, worst displacement = %.4f px\n", worstG, worstPx);
  test_bool(worstG < 1e-5);
  test_bool(worstPx < 0.01);
}

/* Pincushion: points beyond the domain report the sentinel rather than garbage. */
static void test_lensmap_domain(void){
  VSFrameInfo fi;
  VSLensPlaneMap m;
  VSTransform t = null_transform();
  double xs, ys;
  vsFrameInfoInit(&fi, 640, 360, PF_YUV420P);
  test_bool(vsLensPlaneMapInit(&m, &fi, &fi, 0, 0.15, VSLensCorrectFull) == VS_OK);
  test_bool(m.tDomD > 1.6 && m.tDomD < 1.7);        /* 1/(4*0.15) = 1.6667 */
  /* a point at r = 1.4 in ideal coordinates is outside the domain */
  t.zoom = -60;                                      /* zoom out to reach it */
  test_bool(vsLensMapBackward(&m, &t, 0, 0, &xs, &ys) == VS_ERROR);
  test_bool(xs == VS_LENS_OUTSIDE_PX && ys == VS_LENS_OUTSIDE_PX);
  vsLensPlaneMapFree(&m);
  /* barrel never leaves the domain inside the frame */
  test_bool(vsLensPlaneMapInit(&m, &fi, &fi, 0, -0.3, VSLensCorrectFull) == VS_OK);
  test_bool(m.tDomD == INFINITY);
  vsLensPlaneMapFree(&m);
}

/* Off mode and k = 0 both leave the map inactive. */
static void test_lensmap_inactive(void){
  VSFrameInfo fi;
  VSLensPlaneMap m;
  vsFrameInfoInit(&fi, 640, 360, PF_YUV420P);
  test_bool(vsLensPlaneMapInit(&m, &fi, &fi, 0, -0.25, VSLensCorrectOff) == VS_OK);
  test_bool(m.active == 0);
  vsLensPlaneMapFree(&m);
  test_bool(vsLensPlaneMapInit(&m, &fi, &fi, 0, 0.0, VSLensCorrectWobble) == VS_OK);
  test_bool(m.active == 0);
  vsLensPlaneMapFree(&m);
}

/* Chroma planes: the map must be the luma map expressed in plane units. */
static void test_lensmap_chroma_consistency(void){
  VSFrameInfo fi422, fi420;
  VSLensPlaneMap luma, c422, c420;
  VSTransform t = null_transform();
  int x, y;
  /* Translation only, deliberately.  The legacy affine step applies one shared
     sin/cos in plane coordinates, which is inconsistent under anisotropic
     subsampling; spec 2.3 declines to fix or replicate that, so a rotation here
     would assert something this work does not deliver.  Translation is
     subsampled per axis correctly and still exercises the radial map off
     centre, which is what this test exists to check. */
  t.x = 9.0; t.y = -5.0;
  vsFrameInfoInit(&fi422, 640, 360, PF_YUV422P);
  vsFrameInfoInit(&fi420, 640, 360, PF_YUV420P);
  test_bool(vsLensPlaneMapInit(&luma, &fi422, &fi422, 0, -0.25, VSLensCorrectWobble) == VS_OK);
  test_bool(vsLensPlaneMapInit(&c422, &fi422, &fi422, 1, -0.25, VSLensCorrectWobble) == VS_OK);
  test_bool(vsLensPlaneMapInit(&c420, &fi420, &fi420, 1, -0.25, VSLensCorrectWobble) == VS_OK);
  for(y=0; y<180; y+=13){
    for(x=0; x<320; x+=13){
      double lx, ly, cx, cy;
      /* 4:2:2 chroma pixel (x,y) is luma (2x, y) */
      test_bool(vsLensMapBackward(&luma, &t, 2.0*x, 1.0*y, &lx, &ly) == VS_OK);
      test_bool(vsLensMapBackward(&c422, &t, 1.0*x, 1.0*y, &cx, &cy) == VS_OK);
      test_bool(fabs(cx*2.0 - lx) < 1e-6);
      test_bool(fabs(cy*1.0 - ly) < 1e-6);
      /* 4:2:0 chroma pixel (x,y) is luma (2x, 2y) -- isotropic, easier */
      if(y < 90){
        test_bool(vsLensMapBackward(&luma, &t, 2.0*x, 2.0*y, &lx, &ly) == VS_OK);
        test_bool(vsLensMapBackward(&c420, &t, 1.0*x, 1.0*y, &cx, &cy) == VS_OK);
        test_bool(fabs(cx*2.0 - lx) < 1e-6);
        test_bool(fabs(cy*2.0 - ly) < 1e-6);
      }
    }
  }
  vsLensPlaneMapFree(&luma); vsLensPlaneMapFree(&c422); vsLensPlaneMapFree(&c420);
}
```

`vsLensMapBackward` takes coordinates in **plane units** and the translation in `VSTransform` in
**luma pixels** (it divides by the plane subsampling itself), matching how the warp loops already
compute `tx = t.x / (1 << wsub)`.

- [ ] **Step 3: Register the tests and confirm they fail to build**

In `tests/tests.c` add `#include "test_lensmap.c"` after `#include "test_lensdistortion.c"`, and in
`main`:

```c
  if(all || contains(argv,argc,"--testLMAP", "render-path lens map and LUTs")){
    UNIT(test_lensmap_scales());
    UNIT(test_lensmap_identity_transform());
    UNIT(test_lensmap_lut());
    UNIT(test_lensmap_domain());
    UNIT(test_lensmap_inactive());
    UNIT(test_lensmap_chroma_consistency());
  }
```

Run: `cmake --build build/tests -j8`
Expected: FAIL — `lensmap.h: No such file or directory`.

- [ ] **Step 4: Implement `src/lensmap.c`**

```c
#include "lensmap.h"
#include "transform.h"                /* CHROMA_SIZE, VS_OK, VS_ERROR */
#include "vidstabdefines.h"
#include <stdlib.h>

double vsLensScaleUDirect(double k, double t){ return vsLensScaleUDirectI(k, t); }
double vsLensScaleDDirect(double k, double t){ return vsLensScaleDDirectI(k, t); }

static int32_t toFp16d(double v){
  if(v >  32000.0) v =  32000.0;
  if(v < -32000.0) v = -32000.0;
  return (int32_t)(v * 65536.0 + (v >= 0 ? 0.5 : -0.5));
}

int vsLensPlaneMapInit(VSLensPlaneMap* m, const VSFrameInfo* fiSrc,
                       const VSFrameInfo* fiDest, int plane,
                       double k, VSLensCorrectMode mode){
  double rho, rhoDest, rho2;
  int i;
  memset(m, 0, sizeof(*m));
  m->tDomD = INFINITY;
  if(mode == VSLensCorrectOff || k == 0.0) return VS_OK;

  m->k       = k;
  m->sxShift = vsGetPlaneWidthSubS(fiSrc, plane);
  m->syShift = vsGetPlaneHeightSubS(fiSrc, plane);

  /* rho is the SOURCE half-diagonal in luma pixels: the same lens, whatever
     the destination geometry.  See the spec, section 2.2. */
  rho  = 0.5*sqrt((double)fiSrc->width*fiSrc->width +
                  (double)fiSrc->height*fiSrc->height);
  rhoDest = 0.5*sqrt((double)fiDest->width*fiDest->width +
                     (double)fiDest->height*fiDest->height);
  rho2 = rho*rho;
  m->invRho2 = 1.0/rho2;

  /* U_k only ever sees destination-centred coordinates, so the destination
     half-diagonal plus 20% covers it.  D_k sees ideal coordinates after the
     similarity, so it needs room; r = 2 is twice the corner radius and every
     point beyond r = 1 is off-frame anyway. */
  m->tMaxU = 1.2 * (rhoDest/rho)*(rhoDest/rho);
  m->tMaxD = 4.0;
  if(k > 0){
    m->tDomD = 1.0/(4.0*k);
    if(m->tMaxD > 0.99*m->tDomD) m->tMaxD = 0.99*m->tDomD;
  }
  /* U_k's own domain edge, 1 + k*t <= 0, sits at t = -1/k for barrel; that is
     t >= 3.3 for k = -0.3, far beyond tMaxU. Assert rather than handle. */

  m->gU = (int32_t*)vs_malloc(sizeof(int32_t)*VS_LENS_LUT_N);
  m->gD = (int32_t*)vs_malloc(sizeof(int32_t)*VS_LENS_LUT_N);
  if(!m->gU || !m->gD){ vsLensPlaneMapFree(m); return VS_ERROR; }
#ifdef TESTING
  m->gUf = (float*)vs_malloc(sizeof(float)*VS_LENS_LUT_N);
  m->gDf = (float*)vs_malloc(sizeof(float)*VS_LENS_LUT_N);
  if(!m->gUf || !m->gDf){ vsLensPlaneMapFree(m); return VS_ERROR; }
#endif
  for(i=0; i<VS_LENS_LUT_N; i++){
    double tU = m->tMaxU * i/(double)(VS_LENS_LUT_N-1);
    double tD = m->tMaxD * i/(double)(VS_LENS_LUT_N-1);
    double gu = vsLensScaleUDirectI(k, tU);
    double gd = vsLensScaleDDirectI(k, tD);
    m->gU[i] = toFp16d(gu);
    m->gD[i] = toFp16d(gd);
#ifdef TESTING
    m->gUf[i] = (float)gu;
    m->gDf[i] = (float)gd;
#endif
  }
  /* idxScale = (N-1)/(tMax*rho^2) at scale 2^32, consumed by vsLensLutFp. */
  m->idxScaleU = (int32_t)((VS_LENS_LUT_N-1)/(m->tMaxU*rho2) * 4294967296.0);
  m->idxScaleD = (int32_t)((VS_LENS_LUT_N-1)/(m->tMaxD*rho2) * 4294967296.0);
  m->active = 1;
  return VS_OK;
}

void vsLensPlaneMapFree(VSLensPlaneMap* m){
  if(m->gU)  { vs_free(m->gU);  m->gU  = 0; }
  if(m->gD)  { vs_free(m->gD);  m->gD  = 0; }
#ifdef TESTING
  if(m->gUf) { vs_free(m->gUf); m->gUf = 0; }
  if(m->gDf) { vs_free(m->gDf); m->gDf = 0; }
#endif
  m->active = 0;
}
```

`Init` must also fill the plane centres and the mode, which `vsLensMapBackward` needs. Set them
from exactly the expressions the warp loops use for `c_d_x` / `c_s_x`, so the two agree by
construction:

```c
  m->mode = mode;
  m->cdx  = (fiDest->width  >> m->sxShift)/2.0;
  m->cdy  = (fiDest->height >> m->syShift)/2.0;
  m->csx  = (fiSrc->width   >> m->sxShift)/2.0;
  m->csy  = (fiSrc->height  >> m->syShift)/2.0;
```

Beware: `_FLT(transformPlanar)` uses `(width >> wsub)/2.0` while the fixed-point `transformPlanar`
uses `CHROMA_SIZE(width, wsub)/2`, an integer division. For even dimensions these agree; keep the
float form here and let the fixed-point loop keep using its own `c_s_x`, which is what the `k = 0`
bit-exactness guard requires. Then:

```c
/* t = r^2 in luma-equivalent units for a plane-unit offset. */
static double tOf(const VSLensPlaneMap* m, double dx, double dy){
  double lx = dx * (1 << m->sxShift);
  double ly = dy * (1 << m->syShift);
  return (lx*lx + ly*ly) * m->invRho2;
}

int vsLensMapBackward(const VSLensPlaneMap* m, const VSTransform* t,
                      double xd, double yd, double* xs, double* ys){
  double dx = xd - m->cdx, dy = yd - m->cdy;
  double z, ca, sa, tx, ty, xi, yi, ex, ey, tt;
  if(m->mode == VSLensCorrectWobble){
    double g = vsLensScaleUDirectI(m->k, tOf(m, dx, dy));
    dx *= g; dy *= g;
  }
  z  = 1.0 - t->zoom/100.0;
  ca = z*cos(-t->alpha); sa = z*sin(-t->alpha);
  tx = t->x / (double)(1 << m->sxShift);
  ty = t->y / (double)(1 << m->syShift);
  xi =  ca*dx + sa*dy + m->csx - tx;
  yi = -sa*dx + ca*dy + m->csy - ty;
  if(m->mode != VSLensCorrectOff){
    ex = xi - m->csx; ey = yi - m->csy;
    tt = tOf(m, ex, ey);
    if(tt > m->tDomD){ *xs = *ys = VS_LENS_OUTSIDE_PX; return VS_ERROR; }
    { double g = vsLensScaleDDirectI(m->k, tt);
      xi = m->csx + ex*g; yi = m->csy + ey*g; }
  }
  *xs = xi; *ys = yi;
  return VS_OK;
}
```

When `m->active == 0`, `mode` is `Off` in the struct, so `vsLensMapBackward` degenerates to the
plain similarity — which is what the zoom budget in Task 7 wants.

- [ ] **Step 5: Wire the build**

`CMakeLists.txt` line 69: append `src/lensmap.c` to `SOURCES`.
`tests/CMakeLists.txt`: add `../src/lensmap.c` to **both** `add_executable(tests ...)` and
`add_executable(bench ...)`, next to `../src/lensdistortion.c`. Note `bench` does not currently list
`lensdistortion.c`; add `lensmap.c` there anyway since `transform.c` will reference it from Task 3
onward, and add `../src/lensdistortion.c` to `bench` too if the link then fails.

- [ ] **Step 6: Run the tests**

Run: `cmake -S tests -B build/tests && cmake --build build/tests -j8 && ./build/tests/tests --testLMAP`
Expected: PASS, with the LUT test printing its worst error line.

Run: `./build/tests/tests --all`
Expected: everything passes, `--testBASE` included.

- [ ] **Step 7: Commit**

```bash
git add src/lensmap.c src/lensmap.h tests/test_lensmap.c tests/tests.c CMakeLists.txt tests/CMakeLists.txt
git commit -m "lensmap: radial map and lookup tables for the render path"
```

---

### Task 3: Config surface, plumbing, and the float planar luma path

The first task that changes a picture.

**Files:**
- Modify: `src/transform.h` (`VSTransformConfig`, `VSTransformData`), `src/transform.c`
  (`vsTransformGetDefaultConfig`, `vsTransformDataInit`, `vsTransformDataCleanup`, new
  `vsTransformSetLensK`, new internal `lensEnsureMaps`)
- Modify: `src/transform_internal.h` (declare `lensEnsureMaps`)
- Modify: `src/transformfloat.c` (`_FLT(transformPlanar)`, plane 0 only for now)
- Modify: `tests/test_lensmap.c`, `tests/tests.c`

**Interfaces:**
- Consumes: everything Task 2 produced.
- Produces:
  ```c
  /* VSTransformConfig */
  VSLensCorrectMode lensCorrection;   /* default VSLensCorrectWobble */
  double            lensK;            /* 0 -> use the estimate */
  /* VSTransformData */
  VSLensCorrectMode lensMode;
  int               lensActive;
  double            lensK;
  double            lensMapK;         /* effective k the maps were built for; -1 = none */
  VSLensPlaneMap    lensMaps[3];
  /* API */
  VS_API void vsTransformSetLensK(VSTransformData* td, double k);
  /* internal, src/transform_internal.h */
  void lensEnsureMaps(VSTransformData* td);
  ```

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_lensmap.c`:

```c
/* --- render-path tests ---------------------------------------------------- */

/* Builds a td for a luma-only frame with a known k. */
static void lmInitTd(VSTransformData* td, const VSFrameInfo* fi,
                     VSLensCorrectMode mode, double k, VSInterpolType ip){
  VSTransformConfig cfg = vsTransformGetDefaultConfig("lensmap-test");
  cfg.interpolType    = ip;
  cfg.crop            = VSCropBorder;
  cfg.optZoom         = 0;
  cfg.lensCorrection  = mode;
  test_bool(vsTransformDataInit(td, &cfg, fi, fi) == VS_OK);
  vsTransformSetLensK(td, k);
}

/* Wobble mode with an identity transform must not touch a single pixel. */
static void test_lensmap_wobble_identity_is_exact(void){
  VSFrameInfo fi;
  VSFrame src, dest;
  uint32_t seed = 999;
  int i, ip;
  vsFrameInfoInit(&fi, 320, 240, PF_GRAY8);
  vsFrameAllocate(&src, &fi);
  ldFillTexture(&src, &fi, &seed);
  for(ip=0; ip<4; ip++){
    for(i=0; i<LM_NUM_KS; i++){
      VSTransformData td;
      lmInitTd(&td, &fi, VSLensCorrectWobble, LM_KS[i], (VSInterpolType)ip);
      vsFrameAllocate(&dest, &fi);
      test_bool(vsTransformPrepare(&td, &src, &dest) == VS_OK);
      test_bool(_FLT(transformPlanar)(&td, null_transform()) == VS_OK);
      test_bool(vsTransformFinish(&td) == VS_OK);
      test_bool(memcmp(src.data[0], dest.data[0], (size_t)fi.width*fi.height) == 0);
      vsFrameFree(&dest);
      vsTransformDataCleanup(&td);
    }
  }
  vsFrameFree(&src);
}

/* The claim.  Synthesise a still scene filmed by a moving camera through a
   barrel lens, correct with the exact inverse transforms, and check that the
   result stops wobbling.  Compare frames to each other, not to a golden. */
static void test_lensmap_removes_wobble(void){
  const double k = -0.25;
  const int NF = 6;
  VSFrameInfo fi;
  VSFrame base, obs[6], out[6];
  VSLensDistortion ld;
  uint32_t seed = 4242;
  double worstOff = 0, worstWob = 0;
  int i, mode;
  vsFrameInfoInit(&fi, 480, 320, PF_GRAY8);
  vsFrameAllocate(&base, &fi);
  ldFillTexture(&base, &fi, &seed);
  ld = vsLensDistortionInit(&fi, k);

  for(mode=0; mode<2; mode++){
    VSLensCorrectMode cm = mode ? VSLensCorrectWobble : VSLensCorrectOff;
    double worst = 0;
    for(i=0; i<NF; i++){
      VSTransformData td;
      VSTransform cum = null_transform(), inv;
      cum.x = 6.0*i - 15.0; cum.y = -4.0*i + 10.0;
      vsFrameAllocate(&obs[i], &fi);
      vsFrameAllocate(&out[i], &fi);
      ldRenderWarped(&base, &obs[i], &fi, &ld, &cum);
      inv = cum;                       /* the render loop applies -t, so pass +cum */
      lmInitTd(&td, &fi, cm, k, VS_BiLinear);
      test_bool(vsTransformPrepare(&td, &obs[i], &out[i]) == VS_OK);
      test_bool(_FLT(transformPlanar)(&td, inv) == VS_OK);
      test_bool(vsTransformFinish(&td) == VS_OK);
      vsTransformDataCleanup(&td);
    }
    /* residual instability: mean |out[i] - out[0]| over the region that is
       inside the source for every frame (a 40 px inset is ample here) */
    for(i=1; i<NF; i++){
      double sum = 0; int n = 0, x, y;
      for(y=40; y<fi.height-40; y++)
        for(x=40; x<fi.width-40; x++){
          sum += fabs((double)out[i].data[0][y*out[i].linesize[0]+x] -
                      (double)out[0].data[0][y*out[0].linesize[0]+x]);
          n++;
        }
      if(sum/n > worst) worst = sum/n;
    }
    for(i=0; i<NF; i++){ vsFrameFree(&obs[i]); vsFrameFree(&out[i]); }
    if(mode) worstWob = worst; else worstOff = worst;
  }
  fprintf(stderr, "  wobble: mean|dI| uncorrected = %.2f, corrected = %.2f\n",
          worstOff, worstWob);
  test_bool(worstWob < worstOff * 0.5);
  vsFrameFree(&base);
}
```

`ldRenderWarped` samples `D_k(S^{-1}(U_k(y)))`, so the frame it produces is the scene seen after
camera motion `cum`. The correcting transform that undoes it is `cum` itself, because the warp loop
subtracts `t.x`/`t.y`. If the first run shows the corrected residual *larger* than uncorrected,
the sign is flipped — try `mult_transform(&cum, -1)` and fix the comment rather than the threshold.

- [ ] **Step 2: Register and run, confirm failure**

Add to the `--testLMAP` block in `tests/tests.c`:

```c
    UNIT(test_lensmap_wobble_identity_is_exact());
    UNIT(test_lensmap_removes_wobble());
```

Run: `cmake --build build/tests -j8`
Expected: FAIL — `lensCorrection` is not a member of `VSTransformConfig`.

- [ ] **Step 3: Add the config and data fields**

`src/transform.h`: add `#include "lensmap.h"` near the other includes. In `VSTransformConfig`, after
`estimateLensDistortion`:

```c
    /* Applying the estimated distortion to the picture, as opposed to merely
       using it to interpret the motions.  Wobble is the default: it collapses
       to the identity whenever the stabilising transform does, so it costs no
       field of view and leaves a locked-off shot untouched, while removing the
       frame-to-frame distortion variation the stabiliser cannot.  Full
       undistorts the picture outright and stays opt-in.  See
       docs/superpowers/specs/2026-08-09-lens-correction-render-design.md */
    VSLensCorrectMode lensCorrection;
    /* Manual override for k; 0 means "use whatever was estimated".  Needed for
       the transforms-file path, which carries no k, and for users who know
       their lens.  Not NaN: -ffast-math makes a NaN sentinel unreliable, and
       nothing is lost, since "no correction" is lensCorrection = Off. */
    double            lensK;
```

In `VSTransformData`, after `conf`:

```c
    /* Lens correction state.  k only becomes known after vsTransformDataInit
       runs, so the per-plane maps are built lazily by lensEnsureMaps(). */
    VSLensCorrectMode lensMode;
    int               lensActive;
    double            lensK;      /* 0 until set */
    double            lensMapK;   /* effective k the maps were built for; -1 = none,
                                     a value no effective k can ever take */
    VSLensPlaneMap    lensMaps[3];
```

- [ ] **Step 4: Implement the plumbing in `src/transform.c`**

In `vsTransformGetDefaultConfig`, after `conf.estimateLensDistortion = 1;`:

```c
  /* Wobble is safe to default on: identity in, identity out. */
  conf.lensCorrection = VSLensCorrectWobble;
  conf.lensK          = 0.0;
```

In `vsTransformDataInit`, before `return VS_OK`:

```c
  td->lensMode   = td->conf.lensCorrection;
  td->lensActive = 0;
  td->lensK      = td->conf.lensK;      /* 0 unless the caller overrode it */
  td->lensMapK   = -1.0;                /* no effective k can be -1, so the
                                           first lensEnsureMaps always builds */
  memset(td->lensMaps, 0, sizeof(td->lensMaps));
```

In `vsTransformDataCleanup`, first thing:

```c
  for(int p=0; p<3; p++) vsLensPlaneMapFree(&td->lensMaps[p]);
```

New functions:

```c
void vsTransformSetLensK(VSTransformData* td, double k){
  td->lensK = k;
}

/* Builds (or rebuilds) the per-plane maps.  k arrives after
   vsTransformDataInit, so this cannot live there.  Cheap when nothing
   changed: one double compare. */
void lensEnsureMaps(VSTransformData* td){
  double k = td->lensK;
  int planes, p;
  /* |k| below this moves a corner pixel by well under a pixel; acting on it
     would only add noise.  Same guard the estimator uses. */
  int want = td->lensMode != VSLensCorrectOff && fabs(k) > 0.01;
  if(!want) k = 0.0;
  if(td->lensMapK == k) return;
  planes = td->fiSrc.pFormat < PF_PACKED ? td->fiSrc.planes : 1;
  for(p=0; p<3; p++) vsLensPlaneMapFree(&td->lensMaps[p]);
  td->lensActive = 0;
  for(p=0; p<planes; p++){
    if(vsLensPlaneMapInit(&td->lensMaps[p], &td->fiSrc, &td->fiDest, p, k,
                          want ? td->lensMode : VSLensCorrectOff) != VS_OK){
      vs_log_error(td->conf.modName, "lens map allocation failed, correction off\n");
      for(int q=0; q<3; q++) vsLensPlaneMapFree(&td->lensMaps[q]);
      td->lensActive = 0;
      td->lensMapK = 0.0;
      return;
    }
  }
  td->lensActive = td->lensMaps[0].active;
  td->lensMapK   = k;
}
```

Simplify the guard at the top to just `if(td->lensMapK == k) return;` after computing the effective
`k` — the extra clause above is redundant, drop it. Declare `lensEnsureMaps` in
`src/transform_internal.h` and `vsTransformSetLensK` in `src/transform.h` next to
`vsTransformGetConfig`, with a doc comment saying it is for consumers that do not share one
`VSTransformData` between the two passes.

- [ ] **Step 5: Insert the map into `_FLT(transformPlanar)`**

`src/transformfloat.c`. At the top of the function, before the early-out:

```c
  lensEnsureMaps(td);
```

Change the early-out (line 297) to:

```c
  /* Wobble mode maps identity to identity (D_k . U_k = id), so the fast path
     stays correct and stays reachable.  Full mode must still undistort. */
  if (t.alpha==0 && t.x==0 && t.y==0 && t.zoom == 0 &&
      !(td->lensActive && td->lensMode == VSLensCorrectFull)){
```

Inside the plane loop, after the existing `zsin_a`/`tx`/`ty` setup:

```c
    const VSLensPlaneMap* lm = &td->lensMaps[plane];
    int lensOn = lm->active;
    int wobble = lensOn && td->lensMode == VSLensCorrectWobble;
    float sxf = (float)(1 << wsub), syf = (float)(1 << hsub);
    float irho2 = (float)lm->invRho2;
```

and replace the inner loop body with:

```c
      for (y = 0; y < h; y++) {
        float x_d1 = (x - c_d_x);
        float y_d1 = (y - c_d_y);
        float x_s, y_s;
        if (wobble) {
          float lx = x_d1*sxf, ly = y_d1*syf;
          float g  = vsLensLutF(lm->gUf, lm->tMaxU, (lx*lx + ly*ly)*irho2);
          x_d1 *= g; y_d1 *= g;
        }
        x_s  =  zcos_a * x_d1 + zsin_a * y_d1 + c_s_x - tx;
        y_s  = -zsin_a * x_d1 + zcos_a * y_d1 + c_s_y - ty;
        if (lensOn) {
          float ex = x_s - c_s_x, ey = y_s - c_s_y;
          float lx = ex*sxf, ly = ey*syf;
          float tt = (lx*lx + ly*ly)*irho2;
          if (tt > lm->tDomD) { x_s = y_s = VS_LENS_OUTSIDE_PX; }
          else {
            float g = vsLensLutF(lm->gDf, lm->tMaxD, tt);
            x_s = c_s_x + ex*g; y_s = c_s_y + ey*g;
          }
        }
        uint8_t *dest = &dat_2[x + y * td->destbuf.linesize[plane]];
        td->_FLT(interpolate)(dest, x_s, y_s, dat_1, td->src.linesize[plane],
                              sw, sh, crop ? black : *dest);
      }
```

`lm->tDomD` is `INFINITY` for barrel, so the comparison never fires there. Note `lensOn` is false
for every plane when the maps are inactive, which restores the original instruction sequence.

For this task only planes beyond 0 may be left going through the affine path — but since
`vsLensPlaneMapInit` is already called for every plane in `lensEnsureMaps`, the code above simply
works for all planes. Let it. Task 4 is then only about *testing* chroma, which is fine: fold
Task 4's test into this task if it is already passing.

- [ ] **Step 6: Run the tests**

Run: `cmake --build build/tests -j8 && ./build/tests/tests --testLMAP`
Expected: PASS. The wobble test prints its two residuals; the corrected one should be several times
smaller, not marginally.

Run: `./build/tests/tests --testBASE`
Expected: PASS — this proves the default-on wobble mode did not disturb `k = 0` output, since
`lensK` is `0` there and the maps stay inactive.

Run: `./build/tests/tests --all`
Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add src/transform.h src/transform.c src/transform_internal.h src/transformfloat.c tests/test_lensmap.c tests/tests.c
git commit -m "transform: apply the lens map in the float planar warp"
```

---

### Task 4: Chroma planes end to end

Task 3 already runs every plane through its own map. This task proves it, on 4:2:0 **and** 4:2:2.

**Files:**
- Modify: `tests/test_lensmap.c`, `tests/tests.c`

**Interfaces:**
- Consumes: `lmInitTd`, `_FLT(transformPlanar)`, the maps from Task 3.
- Produces: `void test_lensmap_chroma_render(void)`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_lensmap.c`:

```c
/* Colour must not drift away from luma toward the frame edge.  Paint a frame
   where chroma is a step function aligned with a luma step, warp it with the
   lens active, and check the two edges still coincide after correction.
   4:2:2 is the case that catches an anisotropic-radius bug. */
static void lmCheckChromaAlignment(VSPixelFormat pf, const char* name){
  const double k = -0.25;
  VSFrameInfo fi;
  VSFrame src, dest;
  VSTransformData td;
  VSTransform t = null_transform();
  int wsub, hsub, x, y, worst = 0;
  vsFrameInfoInit(&fi, 480, 320, pf);
  vsFrameAllocate(&src, &fi);
  wsub = vsGetPlaneWidthSubS(&fi, 1);
  hsub = vsGetPlaneHeightSubS(&fi, 1);
  /* vertical step at luma x = 300, replicated into chroma at the same place */
  for(y=0; y<fi.height; y++)
    for(x=0; x<fi.width; x++)
      src.data[0][y*src.linesize[0]+x] = x < 300 ? 40 : 200;
  for(y=0; y<CHROMA_SIZE(fi.height,hsub); y++)
    for(x=0; x<CHROMA_SIZE(fi.width,wsub); x++){
      uint8_t v = (x << wsub) < 300 ? 40 : 200;
      src.data[1][y*src.linesize[1]+x] = v;
      src.data[2][y*src.linesize[2]+x] = v;
    }

  t.x = 12.0; t.y = -8.0;
  lmInitTd(&td, &fi, VSLensCorrectWobble, k, VS_BiLinear);
  vsFrameAllocate(&dest, &fi);
  test_bool(vsTransformPrepare(&td, &src, &dest) == VS_OK);
  test_bool(_FLT(transformPlanar)(&td, t) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);

  /* For each chroma row, find the step in luma and in chroma; they must land
     within one chroma pixel of each other. */
  for(y = 8; y < CHROMA_SIZE(fi.height,hsub) - 8; y++){
    int ly = y << hsub, lx = -1, cx = -1;
    for(x=1; x<fi.width; x++)
      if(dest.data[0][ly*dest.linesize[0]+x] > 120 &&
         dest.data[0][ly*dest.linesize[0]+x-1] <= 120){ lx = x; break; }
    for(x=1; x<CHROMA_SIZE(fi.width,wsub); x++)
      if(dest.data[1][y*dest.linesize[1]+x] > 120 &&
         dest.data[1][y*dest.linesize[1]+x-1] <= 120){ cx = x; break; }
    if(lx < 0 || cx < 0) continue;
    { int d = abs((cx << wsub) - lx);
      if(d > worst) worst = d; }
  }
  fprintf(stderr, "  %s: worst luma/chroma step offset = %i luma px\n", name, worst);
  test_bool(worst <= (1 << wsub));
  vsFrameFree(&dest); vsFrameFree(&src);
  vsTransformDataCleanup(&td);
}

static void test_lensmap_chroma_render(void){
  lmCheckChromaAlignment(PF_YUV420P, "YUV420P");
  lmCheckChromaAlignment(PF_YUV422P, "YUV422P");
  lmCheckChromaAlignment(PF_YUV444P, "YUV444P");
}
```

Check `PF_YUV444P` exists in `src/frameinfo.h`; drop that line if it does not.

- [ ] **Step 2: Register and run**

Add `UNIT(test_lensmap_chroma_render());` to the `--testLMAP` block.

Run: `cmake --build build/tests -j8 && ./build/tests/tests --testLMAP`
Expected: PASS if Task 3's per-plane `invRho2`/`sxShift` handling is right. If 4:2:2 fails while
4:2:0 passes, the radius is being computed in plane units somewhere — that is the bug §2.3 of the
spec predicts, and the fix is in `vsLensPlaneMapInit`/the inner loop, not in the test.

- [ ] **Step 3: Commit**

```bash
git add tests/test_lensmap.c tests/tests.c
git commit -m "test: luma/chroma alignment under lens correction, 4:2:0 and 4:2:2"
```

---

### Task 5: The fixed-point planar path

`transformfixedpoint.c` is the production path. Everything above only exercised `TESTING` code.

**Files:**
- Modify: `src/transformfixedpoint.c` (`transformPlanar`)
- Modify: `tests/test_lensmap.c`, `tests/tests.c`

**Interfaces:**
- Consumes: `vsLensLutFp`, `VSLensPlaneMap.idxScaleU/idxScaleD/gU/gD/tDomD`, `lensEnsureMaps`.
- Produces: `void test_lensmap_fixed_float_equivalence(void)`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_lensmap.c`:

```c
/* The fixed-point path is what ships.  It must match the float path within a
   couple of LSB across modes, k, and interpolation types. */
static void test_lensmap_fixed_float_equivalence(void){
  const double ks[] = {-0.3, -0.25, -0.1, 0.12};
  VSFrameInfo fi;
  VSFrame src, dfix, dflt;
  uint32_t seed = 7;
  int ik, ip, im, x, y;
  double worst = 0;
  vsFrameInfoInit(&fi, 320, 240, PF_YUV420P);
  vsFrameAllocate(&src, &fi);
  ldFillTexture(&src, &fi, &seed);
  memset(src.data[1], 0x55, (size_t)src.linesize[1]*(fi.height/2));
  memset(src.data[2], 0xAA, (size_t)src.linesize[2]*(fi.height/2));

  for(im=1; im<=2; im++){
    VSLensCorrectMode cm = im == 1 ? VSLensCorrectWobble : VSLensCorrectFull;
    for(ik=0; ik<4; ik++){
      for(ip=0; ip<4; ip++){
        VSTransformData a, b;
        VSTransform t = null_transform();
        double sum = 0; int n = 0;
        t.x = 6.25; t.y = -3.5; t.alpha = 0.008;
        lmInitTd(&a, &fi, cm, ks[ik], (VSInterpolType)ip);
        lmInitTd(&b, &fi, cm, ks[ik], (VSInterpolType)ip);
        vsFrameAllocate(&dfix, &fi); vsFrameAllocate(&dflt, &fi);
        test_bool(vsTransformPrepare(&a, &src, &dfix) == VS_OK);
        test_bool(transformPlanar(&a, t) == VS_OK);
        test_bool(vsTransformFinish(&a) == VS_OK);
        test_bool(vsTransformPrepare(&b, &src, &dflt) == VS_OK);
        test_bool(_FLT(transformPlanar)(&b, t) == VS_OK);
        test_bool(vsTransformFinish(&b) == VS_OK);
        for(y=0; y<fi.height; y++)
          for(x=0; x<fi.width; x++){
            double d = fabs((double)dfix.data[0][y*dfix.linesize[0]+x] -
                            (double)dflt.data[0][y*dflt.linesize[0]+x]);
            sum += d; n++;
            if(d > worst) worst = d;
          }
        test_bool(sum/n < 1.0);
        vsFrameFree(&dfix); vsFrameFree(&dflt);
        vsTransformDataCleanup(&a); vsTransformDataCleanup(&b);
      }
    }
  }
  fprintf(stderr, "  fixed vs float: worst pixel difference = %.0f\n", worst);
  /* The two paths differ slightly even without a lens (fp16 rounding); a few
     LSB at high-contrast edges is expected, a systematic offset is not. */
  test_bool(worst <= 8);
  vsFrameFree(&src);
}
```

- [ ] **Step 2: Register and run, confirm failure**

Add `UNIT(test_lensmap_fixed_float_equivalence());`.

Run: `cmake --build build/tests -j8 && ./build/tests/tests --testLMAP`
Expected: FAIL — the fixed-point path ignores the lens, so the two outputs diverge badly at the
frame periphery.

- [ ] **Step 3: Implement in `src/transformfixedpoint.c`**

Add `lensEnsureMaps(td);` at the top of `transformPlanar`, extend the early-out with the same
`!(td->lensActive && td->lensMode == VSLensCorrectFull)` clause as the float path, and inside the
plane loop after `c_ty`:

```c
    const VSLensPlaneMap* lm = &td->lensMaps[plane];
    int lensOn = lm->active;
    int wobble = lensOn && td->lensMode == VSLensCorrectWobble;
    /* luma-equivalent shifts: a plane-unit offset is << sub to become luma */
    int lsx = lm->sxShift, lsy = lm->syShift;
    /* tDomD as a squared-radius threshold in luma px^2 at scale 2^32, so the
       per-pixel domain test is an integer compare.  INFINITY for barrel. */
    int64_t domR2 = lm->tDomD == INFINITY ? INT64_MAX
                    : (int64_t)(lm->tDomD / lm->invRho2 * 4294967296.0);
```

`tDomD / invRho2` is `tDomD * rho²` in luma px², times `2^32` to match the `r2fp32` scale. For
`k = 0.15` and a 1920×1080 frame that is `1.667 * 1.22e6 * 4.29e9 ≈ 8.7e15`, comfortably inside
`int64`. Guard the conversion: if it would exceed `INT64_MAX/2`, use `INT64_MAX`.

Inner loop:

```c
      for (x = 0; x < dw; x++) {
        int32_t x_d1 = (x - c_d_x);
        fp16 dx = iToFp16(x_d1), dy = iToFp16(y_d1);
        fp16 x_s, y_s;
        if (wobble) {
          int64_t lx = (int64_t)dx << lsx, ly = (int64_t)dy << lsy;
          int32_t g  = vsLensLutFp(lm->gU, lx*lx + ly*ly, lm->idxScaleU);
          dx = (fp16)(((int64_t)dx * g) >> 16);
          dy = (fp16)(((int64_t)dy * g) >> 16);
        }
        /* zcos_a and zsin_a are 16.16; multiplying by a 16.16 offset needs the
           extra shift the integer form did not.  One expression serves both
           paths: with the lens off, dx is exactly x_d1<<16, so
           (zcos_a*(x_d1<<16))>>16 == zcos_a*x_d1 with no rounding at all --
           the int64 intermediate only removes an overflow risk that the
           wobble scaling introduces. */
        x_s = (fp16)((((int64_t)zcos_a*dx + (int64_t)zsin_a*dy) >> 16)) + c_tx;
        y_s = (fp16)(((-(int64_t)zsin_a*dx + (int64_t)zcos_a*dy) >> 16)) + c_ty;
        if (lensOn) {
          {
            int64_t ex = x_s - c_s_x, ey = y_s - c_s_y;
            int64_t lx = ex << lsx, ly = ey << lsy;
            int64_t r2 = lx*lx + ly*ly;
            if (r2 > domR2) { x_s = iToFp16(VS_LENS_OUTSIDE_PX); y_s = iToFp16(VS_LENS_OUTSIDE_PX); }
            else {
              int32_t g = vsLensLutFp(lm->gD, r2, lm->idxScaleD);
              x_s = (fp16)(c_s_x + ((ex * g) >> 16));
              y_s = (fp16)(c_s_y + ((ey * g) >> 16));
            }
          }
        }
        uint8_t *dest = &dat_2[x + y * td->destbuf.linesize[plane]];
        td->interpolate(dest, x_s, y_s, dat_1,
                        td->src.linesize[plane], sw, sh,
                        td->conf.crop ? black : *dest);
      }
```

There is deliberately **no** duplicated affine `else` arm: the unified expression is bit-identical
to the original when the lens is off, per the comment above. `--testBASE` must therefore still pass
**unchanged** after this task. If it does not, something really did change — report it rather than
regenerating the golden CRCs.

- [ ] **Step 4: Run**

Run: `cmake --build build/tests -j8 && ./build/tests/tests --testLMAP --testBASE`
Expected: both PASS. If the equivalence test shows a large systematic difference, the likely cause
is the 16.16-by-16.16 multiply shift in the `lensOn` branch — verify by setting `k` small
(e.g. 0.011) where the two paths should nearly coincide with the affine result.

Run: `./build/tests/tests --all`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add src/transformfixedpoint.c tests/test_lensmap.c tests/tests.c
git commit -m "transform: apply the lens map in the fixed-point planar warp"
```

---

### Task 6: The packed paths

**Files:**
- Modify: `src/transformfixedpoint.c` (`transformPacked`), `src/transformfloat.c`
  (`_FLT(transformPacked)`)
- Modify: `tests/test_lensmap.c`, `tests/tests.c`

**Interfaces:**
- Consumes: everything above. Packed formats have no subsampling, so `sxShift == syShift == 0` and
  only `lensMaps[0]` is used.
- Produces: `void test_lensmap_packed(void)`.

**Trap:** `_FLT(transformPacked)` (`src/transformfloat.c:239`) has a translation-only fast path for
`|alpha| <= 0.1°` that does integer pixel copies and **ignores zoom entirely** — a pre-existing
quirk. With the lens active that branch must not be taken. Do not try to fix the zoom bug here.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_lensmap.c`:

```c
/* Packed formats go through a different loop with no subsampling. */
static void test_lensmap_packed(void){
  const double k = -0.25;
  VSFrameInfo fi;
  VSFrame src, dest;
  VSTransformData td;
  VSTransform id = null_transform();
  int x, y, diff = 0;
  vsFrameInfoInit(&fi, 320, 240, PF_RGB24);
  vsFrameAllocate(&src, &fi);
  for(y=0; y<fi.height; y++)
    for(x=0; x<fi.width; x++)
      setPixelRGB(&src, &fi, x, y, (uint8_t)(x*7), (uint8_t)(y*5), (uint8_t)(x^y));

  /* wobble + identity must be a bit-exact copy, in both paths */
  lmInitTd(&td, &fi, VSLensCorrectWobble, k, VS_BiLinear);
  vsFrameAllocate(&dest, &fi);
  test_bool(vsTransformPrepare(&td, &src, &dest) == VS_OK);
  test_bool(vsDoTransform(&td, id) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);
  for(y=0; y<fi.height; y++)
    if(memcmp(src.data[0] + (size_t)y*src.linesize[0],
              dest.data[0] + (size_t)y*dest.linesize[0],
              (size_t)fi.width*fi.bytesPerPixel) != 0) diff++;
  test_bool(diff == 0);
  vsFrameFree(&dest);
  vsTransformDataCleanup(&td);

  /* full mode with identity must NOT be a copy -- it undistorts */
  lmInitTd(&td, &fi, VSLensCorrectFull, k, VS_BiLinear);
  vsFrameAllocate(&dest, &fi);
  test_bool(vsTransformPrepare(&td, &src, &dest) == VS_OK);
  test_bool(vsDoTransform(&td, id) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);
  diff = 0;
  for(y=0; y<fi.height; y++)
    if(memcmp(src.data[0] + (size_t)y*src.linesize[0],
              dest.data[0] + (size_t)y*dest.linesize[0],
              (size_t)fi.width*fi.bytesPerPixel) != 0) diff++;
  test_bool(diff > fi.height/2);
  vsFrameFree(&dest);
  vsTransformDataCleanup(&td);
  vsFrameFree(&src);
}
```

Check the exact packed format name in `src/frameinfo.h` (`PF_RGB24` or similar) and use whichever
exists; `tests/test_packed.c` shows which formats the suite already exercises.

- [ ] **Step 2: Register and run, confirm failure**

Add `UNIT(test_lensmap_packed());`.

Run: `cmake --build build/tests -j8 && ./build/tests/tests --testLMAP`
Expected: FAIL on the full-mode half — the packed loop still ignores the lens, so identity produces
an exact copy.

- [ ] **Step 3: Implement in both packed loops**

Same shape as the planar loops. In `transformPacked` (fixed point) add `lensEnsureMaps(td);`, extend
the early-out, take `lm = &td->lensMaps[0]` with `lsx = lsy = 0`, and wrap the `x_s`/`y_s`
computation exactly as in Task 5 — the per-channel inner loop over `k` is unchanged, it just uses the
corrected `x_s`/`y_s`.

In `_FLT(transformPacked)`, add `lensEnsureMaps(td);` and change the branch at line 239 to:

```c
  /* The translation-only shortcut copies whole pixels and cannot express a
     radial map, so the lens forces the general path. */
  if (fabs(t.alpha) > 0.1*M_PI/180.0 || td->lensActive) {
```

then insert the `U_k` / `D_k` steps in that branch's loop the same way as in
`_FLT(transformPlanar)`, with `sxf = syf = 1.0f`.

- [ ] **Step 4: Run**

Run: `cmake --build build/tests -j8 && ./build/tests/tests --testLMAP --testBASE --testPK && ./build/tests/tests --all`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add src/transformfixedpoint.c src/transformfloat.c tests/test_lensmap.c tests/tests.c
git commit -m "transform: apply the lens map in the packed warps"
```

---

### Task 7: The zoom budget

**Files:**
- Modify: `src/transform.c` (new `vsTransformRequiredZoom`, call site at line 484)
- Modify: `src/transform.h` (declaration)
- Modify: `tests/test_lensmap.c`, `tests/tests.c`

**Interfaces:**
- Consumes: `vsLensMapBackward`, `lensEnsureMaps`.
- Produces: `VS_API double vsTransformRequiredZoom(VSTransformData* td, const VSTransform* t);`
  — returns the same percentage units as `transform_get_required_zoom`, and delegates to it
  verbatim when the lens is inactive.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_lensmap.c`:

```c
/* With the returned zoom applied, no destination pixel may sample outside the
   source.  Checked on the frame boundary, densely, not just at the corners. */
static void test_lensmap_required_zoom(void){
  const double ks[] = {-0.3, -0.25, -0.1, 0.12};
  VSFrameInfo fi;
  int ik, im, it, i;
  vsFrameInfoInit(&fi, 640, 360, PF_GRAY8);
  for(im=1; im<=2; im++){
    VSLensCorrectMode cm = im == 1 ? VSLensCorrectWobble : VSLensCorrectFull;
    for(ik=0; ik<4; ik++){
      for(it=0; it<3; it++){
        VSTransformData td;
        VSTransform t = null_transform();
        double z;
        switch(it){
         case 0: t.x = 20; t.y = -14;               break;
         case 1: t.alpha = 0.05;                    break;
         case 2: t.x = -25; t.y = 18; t.alpha = 0.03; break;
        }
        lmInitTd(&td, &fi, cm, ks[ik], VS_BiLinear);
        lensEnsureMaps(&td);
        z = vsTransformRequiredZoom(&td, &t);
        test_bool(z >= 0 && z < 60);
        t.zoom = z + 1e-6;
        /* walk the destination boundary */
        for(i=0; i<2000; i++){
          double u = i/500.0, xd, yd, xs, ys;
          if(u < 1)      { xd = u*(fi.width-1);   yd = 0; }
          else if(u < 2) { xd = fi.width-1;       yd = (u-1)*(fi.height-1); }
          else if(u < 3) { xd = (3-u)*(fi.width-1); yd = fi.height-1; }
          else           { xd = 0;                yd = (4-u)*(fi.height-1); }
          test_bool(vsLensMapBackward(&td.lensMaps[0], &t, xd, yd, &xs, &ys) == VS_OK);
          test_bool(xs >= -0.51 && xs <= fi.width-0.49 &&
                    ys >= -0.51 && ys <= fi.height-0.49);
        }
        vsTransformDataCleanup(&td);
      }
    }
  }
}

/* Inactive lens: identical to the old closed form, to the last bit. */
static void test_lensmap_required_zoom_off(void){
  VSFrameInfo fi;
  VSTransformData td;
  int i;
  vsFrameInfoInit(&fi, 640, 360, PF_GRAY8);
  lmInitTd(&td, &fi, VSLensCorrectOff, -0.25, VS_BiLinear);
  lensEnsureMaps(&td);
  for(i=0; i<20; i++){
    VSTransform t = null_transform();
    t.x = i - 10.0; t.y = 0.5*i - 4.0; t.alpha = 0.002*i;
    test_bool(vsTransformRequiredZoom(&td, &t) ==
              transform_get_required_zoom(&t, fi.width, fi.height));
  }
  vsTransformDataCleanup(&td);
}
```

- [ ] **Step 2: Register and run, confirm failure**

Add both `UNIT(...)` lines.

Run: `cmake --build build/tests -j8`
Expected: FAIL — `vsTransformRequiredZoom` undeclared.

- [ ] **Step 3: Implement**

In `src/transform.c`:

```c
/* Does every destination boundary sample land inside the source at this zoom? */
static int lensFitsAtZoom(const VSLensPlaneMap* lm, const VSTransform* t0,
                          double zoom, int w, int h){
  VSTransform t = *t0;
  int i;
  t.zoom = zoom;
  /* Walk the boundary densely.  Corners and midpoints are NOT sufficient:
     containment is per axis against a rectangle, not against a radius, so
     composed with a rotation the radial expansion can push a mid-edge point
     outside while both adjacent corners stay inside.  Measured counterexample:
     Full, k=-0.10, t=(-25,18), alpha=0.03 overshoots by 2.47 px.  This runs
     once per transform, not per pixel, so the cost is irrelevant. */
  for(i=0; i<nBoundary; i++){
    double xs, ys;
    if(vsLensMapBackward(lm, &t, us[i][0]*(w-1), us[i][1]*(h-1), &xs, &ys) != VS_OK)
      return 0;
    if(xs < -0.5 || xs > w-0.5 || ys < -0.5 || ys > h-0.5) return 0;
  }
  return 1;
}

double vsTransformRequiredZoom(VSTransformData* td, const VSTransform* t){
  int w = td->fiDest.width, h = td->fiDest.height;
  double lo, hi;
  int i;
  lensEnsureMaps(td);
  if(!td->lensActive)
    return transform_get_required_zoom(t, td->fiSrc.width, td->fiSrc.height);
  /* Zoom sits inside M, so the required zoom is a fixed point rather than a
     formula.  Overshoot is monotone in zoom, so bisect. */
  lo = 0.0; hi = 60.0;
  if(lensFitsAtZoom(&td->lensMaps[0], t, lo, w, h)) return 0.0;
  if(!lensFitsAtZoom(&td->lensMaps[0], t, hi, w, h)) return hi;
  for(i=0; i<15; i++){
    double mid = 0.5*(lo+hi);
    if(lensFitsAtZoom(&td->lensMaps[0], t, mid, w, h)) hi = mid; else lo = mid;
  }
  return hi;
}
```

Note the boundary test uses the **destination** dimensions for the sample grid and the **source**
dimensions for the containment check; here they are equal, but keep them distinct so a future
resizing transform does not silently break.

Declare it in `src/transform.h`. Change the `optZoom == 2` loop at `src/transform.c:484`:

```c
      zooms[i] = vsTransformRequiredZoom(td, &ts[i]);
```

and drop the now-unused `w`/`h` locals if the compiler warns. Leave `optZoom == 1` alone: it uses
its own translation-only estimate and is documented as the cheap algorithm.

- [ ] **Step 4: Run**

Run: `cmake --build build/tests -j8 && ./build/tests/tests --testLMAP && ./build/tests/tests --all`
Expected: all PASS. `--testBASE` still passes because it sets `optZoom = 0`.

- [ ] **Step 5: Commit**

```bash
git add src/transform.c src/transform.h tests/test_lensmap.c tests/tests.c
git commit -m "transform: budget zoom from the real backward map when the lens is active"
```

---

### Task 8: Full mode evidence

Full mode already works mechanically. This task proves it does the thing it claims and that
pincushion borders behave.

**Files:**
- Modify: `tests/test_lensmap.c`, `tests/tests.c`

**Interfaces:**
- Consumes: everything above.
- Produces: `void test_lensmap_straightness(void)`, `void test_lensmap_pincushion_border(void)`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_lensmap.c`:

```c
/* Fits a line to the detected edge positions of one painted grid column and
   returns the maximum deviation from it, in pixels. */
static double lmColumnBend(const VSFrame* f, const VSFrameInfo* fi, int approxX){
  double sy = 0, sx = 0, syy = 0, sxy = 0, a, b, worst = 0;
  int n = 0, y, x;
  int xs[1024], ys[1024];
  for(y=20; y<fi->height-20; y+=4){
    int best = -1, bestv = 200;
    for(x=approxX-40; x<=approxX+40; x++){
      int v;
      if(x < 0 || x >= fi->width) continue;
      v = f->data[0][y*f->linesize[0]+x];
      if(v < bestv){ bestv = v; best = x; }
    }
    if(best < 0 || bestv > 120) continue;
    if(n >= 1024) break;
    xs[n] = best; ys[n] = y; n++;
  }
  if(n < 10) return 1e9;
  for(y=0; y<n; y++){ sy += ys[y]; sx += xs[y]; syy += (double)ys[y]*ys[y]; sxy += (double)ys[y]*xs[y]; }
  b = (n*sxy - sy*sx) / (n*syy - sy*sy);
  a = (sx - b*sy)/n;
  for(y=0; y<n; y++){
    double d = fabs(xs[y] - (a + b*ys[y]));
    if(d > worst) worst = d;
  }
  return worst;
}

/* Full mode makes straight lines straight; wobble mode leaves them curved. */
static void test_lensmap_straightness(void){
  const double k = -0.25;
  VSFrameInfo fi;
  VSFrame grid, obs, outFull, outWob;
  VSLensDistortion ld;
  VSTransform id = null_transform();
  VSTransformData td;
  int x, y;
  double bendObs, bendFull, bendWob;
  vsFrameInfoInit(&fi, 640, 480, PF_GRAY8);
  vsFrameAllocate(&grid, &fi);
  vsFrameAllocate(&obs, &fi);
  memset(grid.data[0], 220, (size_t)grid.linesize[0]*fi.height);
  for(y=0; y<fi.height; y++)
    for(x=0; x<fi.width; x++)
      if(x % 80 == 0 || y % 80 == 0)
        grid.data[0][y*grid.linesize[0]+x] = 20;
  ld = vsLensDistortionInit(&fi, k);
  ldRenderWarped(&grid, &obs, &fi, &ld, &id);
  bendObs = lmColumnBend(&obs, &fi, 560);

  vsFrameAllocate(&outFull, &fi);
  lmInitTd(&td, &fi, VSLensCorrectFull, k, VS_BiCubic);
  test_bool(vsTransformPrepare(&td, &obs, &outFull) == VS_OK);
  test_bool(vsDoTransform(&td, id) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);
  vsTransformDataCleanup(&td);
  bendFull = lmColumnBend(&outFull, &fi, 560);

  vsFrameAllocate(&outWob, &fi);
  lmInitTd(&td, &fi, VSLensCorrectWobble, k, VS_BiCubic);
  test_bool(vsTransformPrepare(&td, &obs, &outWob) == VS_OK);
  test_bool(vsDoTransform(&td, id) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);
  vsTransformDataCleanup(&td);
  bendWob = lmColumnBend(&outWob, &fi, 560);

  fprintf(stderr, "  grid bend px: distorted %.2f, full %.2f, wobble %.2f\n",
          bendObs, bendFull, bendWob);
  test_bool(bendObs > 3.0);            /* the input really is curved */
  test_bool(bendFull < 1.5);           /* full mode straightens it */
  test_bool(fabs(bendWob - bendObs) < 1.0);  /* wobble leaves it exactly as-is */
  vsFrameFree(&grid); vsFrameFree(&obs); vsFrameFree(&outFull); vsFrameFree(&outWob);
}

/* Pincushion under full mode reads outside the source at the corners: the
   result must be the border colour, not garbage. */
static void test_lensmap_pincushion_border(void){
  VSFrameInfo fi;
  VSFrame src, dest;
  VSTransformData td;
  VSTransform id = null_transform();
  int x, y;
  vsFrameInfoInit(&fi, 320, 240, PF_GRAY8);
  vsFrameAllocate(&src, &fi);
  memset(src.data[0], 200, (size_t)src.linesize[0]*fi.height);
  lmInitTd(&td, &fi, VSLensCorrectFull, 0.15, VS_BiLinear);
  vsFrameAllocate(&dest, &fi);
  test_bool(vsTransformPrepare(&td, &src, &dest) == VS_OK);
  test_bool(vsDoTransform(&td, id) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);
  /* the very corner reads well outside the source */
  test_bool(dest.data[0][0] < 60);
  test_bool(dest.data[0][fi.width-1] < 60);
  /* the centre is untouched */
  test_bool(dest.data[0][(fi.height/2)*dest.linesize[0] + fi.width/2] > 190);
  /* nothing anywhere is a wild value */
  for(y=0; y<fi.height; y++)
    for(x=0; x<fi.width; x++){
      uint8_t v = dest.data[0][y*dest.linesize[0]+x];
      test_bool(v <= 200);
    }
  vsFrameFree(&src); vsFrameFree(&dest);
  vsTransformDataCleanup(&td);
}
```

- [ ] **Step 2: Register and run**

Add both `UNIT(...)` lines.

Run: `cmake --build build/tests -j8 && ./build/tests/tests --testLMAP`
Expected: PASS. The straightness test prints its three bend numbers — record them in the commit
message, they are the headline evidence that full mode works.

If `bendFull` is not clearly below `bendObs`, the `D_k` direction is inverted somewhere: full mode
would then be *adding* distortion. Check `vsLensMapBackward` against `ldRenderWarped`, which
documents the same composition.

- [ ] **Step 3: Commit**

```bash
git add tests/test_lensmap.c tests/tests.c
git commit -m "test: full-mode straightening and pincushion borders"
```

---

### Task 9: Estimate plumbing, serialisation behaviour, and docs

Wires the estimator's `k` through to the renderer and finishes the documentation.

**Files:**
- Modify: `src/localmotion2transform.c` (`vsLocalmotions2Transforms`, ~line 60)
- Modify: `docs/lens-distortion.md`
- Delete: `docs/lens-correction-handover.md`
- Modify: `tests/test_lensmap.c`, `tests/tests.c`

**Interfaces:**
- Consumes: `vsTransformSetLensK`, `VSTransformConfig.lensK`.
- Produces: no new symbols; the behaviour that after `vsLocalmotions2Transforms` the same
  `VSTransformData` renders with the estimated `k`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_lensmap.c`:

```c
/* An explicit conf.lensK must win over anything estimated, and must survive
   into the render path without any further call. */
static void test_lensmap_config_override(void){
  VSFrameInfo fi;
  VSTransformData td;
  VSTransformConfig cfg = vsTransformGetDefaultConfig("override");
  vsFrameInfoInit(&fi, 320, 240, PF_GRAY8);
  cfg.lensK = -0.25;
  test_bool(vsTransformDataInit(&td, &cfg, &fi, &fi) == VS_OK);
  lensEnsureMaps(&td);
  test_bool(td.lensActive == 1);
  test_bool(td.lensMaps[0].k == -0.25);
  vsTransformDataCleanup(&td);

  /* the default is wobble, but with no k there is nothing to do */
  cfg = vsTransformGetDefaultConfig("override");
  test_bool(cfg.lensCorrection == VSLensCorrectWobble);
  test_bool(cfg.lensK == 0.0);
  test_bool(vsTransformDataInit(&td, &cfg, &fi, &fi) == VS_OK);
  lensEnsureMaps(&td);
  test_bool(td.lensActive == 0);
  vsTransformDataCleanup(&td);

  /* an explicit Off beats an explicit k */
  cfg = vsTransformGetDefaultConfig("override");
  cfg.lensK = -0.25;
  cfg.lensCorrection = VSLensCorrectOff;
  test_bool(vsTransformDataInit(&td, &cfg, &fi, &fi) == VS_OK);
  lensEnsureMaps(&td);
  test_bool(td.lensActive == 0);
  vsTransformDataCleanup(&td);
}

/* The estimate reaches the renderer through the shared VSTransformData:
   after vsLocalmotions2Transforms, rendering through the same td must apply
   the k it just fitted, with no further call from the consumer. */
static void test_lensmap_estimate_reaches_render(void){
  const double trueK = -0.25;
  VSTransformData td;
  VSTransformConfig cfg = vsTransformGetDefaultConfig("e2e-render");
  VSTransformations trans;
  VSManyLocalMotions mlms;
  VSFrameInfo fi;
  /* IMPLEMENTER: build fi, the distorted clip, and mlms exactly as
     test_lensdistortion_endtoend() does -- lift that setup, do not reinvent
     it -- then run the assertions below.  This test must genuinely assert;
     an empty body is not an acceptable deliverable. */
  cfg.estimateLensDistortion = 1;
  cfg.lensCorrection         = VSLensCorrectWobble;
  test_bool(vsTransformDataInit(&td, &cfg, &fi, &fi) == VS_OK);
  vsTransformationsInit(&trans);
  test_bool(vsLocalmotions2Transforms(&td, &mlms, &trans) == VS_OK);
  test_bool(fabs(td.lensK - trueK) < 0.05);
  lensEnsureMaps(&td);
  test_bool(td.lensActive == 1);
  test_bool(td.lensMaps[0].active == 1);
  vsTransformationsCleanup(&trans);
  vsTransformDataCleanup(&td);
}
```

For the second test, open `tests/test_lensdistortion.c:1099` (`test_lensdistortion_endtoend`) and
lift its clip setup verbatim — it already renders distorted frames at a known `k`, runs
`vsMotionDetection`, and collects `VSManyLocalMotions`. Only the clip construction is missing above;
every assertion is already written. The committed test must run and assert — a body that is still
all comments fails this task.

- [ ] **Step 2: Register and run, confirm failure**

Add both `UNIT(...)` lines.

Run: `cmake --build build/tests -j8 && ./build/tests/tests --testLMAP`
Expected: `test_lensmap_estimate_reaches_render` FAILS — `td.lensK` is still `0` after
`vsLocalmotions2Transforms`.

- [ ] **Step 3: Stash the estimate**

In `src/localmotion2transform.c`, inside the `if(td->conf.estimateLensDistortion && ...)` block,
after `useLens` is computed and `lens` is built (around line 66):

```c
    /* Hand the estimate to the render path.  An explicit conf.lensK wins: the
       user knows their lens better than a fit over one clip does. */
    if(fabs(td->conf.lensK) <= 0.01 && useLens)
      vsTransformSetLensK(td, le.k);
```

Add `#include <math.h>` if it is not already there (it is, for `fabs`).

- [ ] **Step 4: Run**

Run: `cmake --build build/tests -j8 && ./build/tests/tests --all`
Expected: all PASS.

- [ ] **Step 5: Documentation**

In `docs/lens-distortion.md`, add a section "Applying the correction to the output" covering:

- the three modes and their backward maps, from spec §1;
- why wobble is the default (identity in, identity out), from spec §1.1;
- the config surface: `lensCorrection`, `lensK`, and their defaults;
- that the transforms-file path (`src/serialize.c`) carries no `k`, so correction is off there
  unless `lensK` is set — the file format was deliberately not extended;
- that `full` mode softens the periphery (`dD/dr ≈ 0.586` at `r = 1`, `k = -0.25`) and bicubic helps,
  but the interpolation type is not changed automatically;
- that `k` is one value per clip, so zoom lenses are out of scope;
- the measured numbers from Tasks 3, 5 and 8 (wobble residual reduction, fixed/float agreement, grid
  bend before and after) — quote the actual figures the tests printed, not estimates.

Then delete `docs/lens-correction-handover.md`; the spec and this section supersede it. Grep for
references to it first (`grep -rn lens-correction-handover .`) and update any that exist.

- [ ] **Step 6: Full verification**

Run:
```
rm -rf build/tests && cmake -S tests -B build/tests && cmake --build build/tests -j8 && ./build/tests/tests --all
```
Expected: a clean build with no new warnings, and every unit passing.

Also build the library proper, which the test CMake does not cover:
```
cmake -S . -B build/lib && cmake --build build/lib -j8
```
Expected: builds clean — this catches a missing `src/lensmap.c` in the top-level `SOURCES`.

- [ ] **Step 7: Commit**

```bash
git add src/localmotion2transform.c docs/lens-distortion.md tests/test_lensmap.c tests/tests.c
git rm docs/lens-correction-handover.md
git commit -m "lens: route the estimate into the render path and document the modes"
```

---

## Self-review notes for the executor

Two places where this plan knowingly leaves a judgement call to the implementer, both flagged
inline:

1. **Task 3, the sign of the correcting transform** in `test_lensmap_removes_wobble`. The warp loop
   subtracts `t.x`, `ldRenderWarped` adds `cum.x`, so passing `cum` should be right — but verify
   from the measured residual rather than assuming, and fix the comment if it turns out inverted.
2. **Task 9, the second test body** is deliberately a pointer to
   `test_lensdistortion_endtoend()` rather than duplicated clip-construction code, because that
   setup is ~60 lines and duplicating it would rot. Lift it, do not rewrite it.

Anything else that does not behave as described here is a bug in the plan — say so rather than
adjusting a threshold to make a test pass.
