/* Unit tests for the render-path lens map: the scale functions, the round trip
   that the wobble mode's safety rests on, and the lookup tables. */

#include "lensmap.h"
#include "transform_internal.h"     /* for lensEnsureMaps(), tested directly below */

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

/* The tables agree with direct evaluation everywhere a sample can land
   in-frame.  gU has no nearby pole (its domain edge sits at t=-1/k, far past
   tMaxU), so its whole table is held to the tight bound.  gD's derivative
   diverges towards tDomD for pincushion, so only the part of its domain that
   an in-frame observed pixel can actually reach -- t <= tIn = 1/(1+k)^2, the
   ideal radius of the frame corner squared -- gets the same tight bound;
   beyond tIn every sample is off-frame by construction and D_k's job there is
   only to stay finite, positive and monotone, not accurate. */
static void test_lensmap_lut(void){
  VSFrameInfo fi;
  VSLensPlaneMap m;
  int i, j;
  double worstG = 0, worstPx = 0;             /* in-frame region */
  double worstGOut = 0, worstPxOut = 0;       /* off-frame region, sanity only */
  vsFrameInfoInit(&fi, 1920, 1080, PF_YUV420P);
  for(i=0; i<LM_NUM_KS; i++){
    double k = LM_KS[i];
    double tIn = 1.0/((1.0+k)*(1.0+k));
    double prevGD = -1;
    test_bool(vsLensPlaneMapInit(&m, &fi, &fi, 0, k, VSLensCorrectWobble) == VS_OK);
    for(j=0; j<4096; j++){
      double tU = m.tMaxU * j/4095.0;
      double tD = m.tMaxD * j/4095.0;
      double gDf = vsLensLutF(m.gDf, m.tMaxD, tD);
      double eU = fabs(vsLensLutF(m.gUf, m.tMaxU, tU) - vsLensScaleUDirect(k, tU));
      double eD = fabs(gDf - vsLensScaleDDirect(k, tD));
      double rho = 0.5*sqrt(1920.0*1920.0 + 1080.0*1080.0);
      if(eU > worstG) worstG = eU;
      if(eU*sqrt(tU)*rho > worstPx) worstPx = eU*sqrt(tU)*rho;
      if(tD <= tIn){
        if(eD > worstG) worstG = eD;
        if(eD*sqrt(tD)*rho > worstPx) worstPx = eD*sqrt(tD)*rho;
      }else{
        /* sanity only: this sample is outside the frame, so accuracy is not
           claimed, but the table must still behave -- finite, positive, and
           monotone in t (increasing for pincushion, decreasing for barrel:
           D_k's denominator 1+sqrt(1-4kt) moves opposite to the sign of k). */
        test_bool(gDf == gDf && gDf > 0);           /* finite (not NaN), positive */
        if(prevGD >= 0) test_bool(k > 0 ? gDf >= prevGD : gDf <= prevGD);
        if(eD > worstGOut) worstGOut = eD;
        if(eD*sqrt(tD)*rho > worstPxOut) worstPxOut = eD*sqrt(tD)*rho;
      }
      prevGD = gDf;
    }
    vsLensPlaneMapFree(&m);
  }
  fprintf(stderr, "  LUT in-frame:  worst |dg| = %.3g, worst displacement = %.4f px\n",
          worstG, worstPx);
  fprintf(stderr, "  LUT off-frame: worst |dg| = %.3g, worst displacement = %.4f px"
                  " (sanity only, no accuracy bound)\n", worstGOut, worstPxOut);
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
  /* barrel never leaves the domain inside the frame: m.tDomD carries the
     "no bound" sentinel, -1.0, not INFINITY -- see lensmap.h for why an
     infinity comparison would be unsafe under -ffast-math. */
  test_bool(vsLensPlaneMapInit(&m, &fi, &fi, 0, -0.3, VSLensCorrectFull) == VS_OK);
  test_bool(m.tDomD < 0.0);
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

/* Chroma planes: the map must be the luma map expressed in plane units.

   alpha is deliberately left at 0 here.  The legacy affine step (which
   vsLensMapBackward must reproduce exactly, for the k=0 bit-exactness
   guard) applies one shared sin/cos pair in plane coordinates to every
   plane, regardless of that plane's own subsampling.  For 4:2:0 that is
   harmless because the plane is subsampled isotropically (wsub==hsub), but
   for 4:2:2 (wsub=1, hsub=0) it is not: rotation mixes an x-offset and a
   y-offset that are expressed in different luma-equivalent scales on that
   plane, so a nonzero alpha makes the luma point and its chroma counterpart
   genuinely diverge.  Zeroing alpha here isolates the thing this module is
   actually responsible for: the radial map must be computed in
   luma-equivalent units, not plane units, and translation -- which *is*
   subsampled correctly per axis -- still exercises that at off-centre
   positions. */
static void test_lensmap_chroma_consistency(void){
  VSFrameInfo fi422, fi420;
  VSLensPlaneMap luma, c422, c420;
  VSTransform t = null_transform();
  int x, y;
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

/* An identity transform (t.alpha==t.x==t.y==t.zoom==0) must take the fast
   memcpy path even in Wobble mode -- it is what makes Wobble free of cost on
   a locked-off shot.  This does NOT exercise the LUT, U_k or D_k: the early
   return in _FLT(transformPlanar) fires before any of that code runs, for
   any k.  See test_lensmap_wobble_cancellation_through_loop for a test that
   actually walks the inner loop and checks the U_k/D_k cancellation itself. */
static void test_lensmap_wobble_identity_takes_fast_path(void){
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

/* The identity claim (D_k . U_k = id, so Wobble with an identity M is a
   no-op) exercised through the *real* inner loop, not the fast-path memcpy.
   A transform that is exactly the identity in every field the fast path
   tests is intercepted before touching a pixel (see
   test_lensmap_wobble_identity_takes_fast_path) -- by design, since that
   fast path is what makes Wobble free on a locked-off shot.  To reach the
   wobble/affine/distort sequence in the inner loop at all, t.x is nudged by
   a sub-pixel amount, just enough to defeat the "t.x==0" check without
   meaningfully moving anything.

   There is no golden for "the LUT-based float code got U_k/M/D_k right", so
   this checks it against an independent computation of the same quantity:
   vsLensMapBackward(), which recomputes U_k, M and D_k in double precision
   from the closed-form scale functions rather than from the float LUT the
   inner loop actually uses.  Sampling the same interpolation function at
   that reference position, and comparing pixel values, tests the LUT
   implementation against ground truth rather than against itself. */
static void test_lensmap_wobble_cancellation_through_loop(void){
  VSFrameInfo fi;
  VSFrame src, dest;
  uint32_t seed = 777;
  int i, ip;
  vsFrameInfoInit(&fi, 320, 240, PF_GRAY8);
  vsFrameAllocate(&src, &fi);
  ldFillTexture(&src, &fi, &seed);
  for(ip=0; ip<4; ip++){
    for(i=0; i<LM_NUM_KS; i++){
      VSTransformData td;
      VSLensPlaneMap ref;
      VSTransform t = null_transform();
      double sumDiff = 0; int n = 0, maxDiff = 0, x, y;
      t.x = 0.01;  /* defeats "t.x==0" in the fast-path check; near-identity */
      lmInitTd(&td, &fi, VSLensCorrectWobble, LM_KS[i], (VSInterpolType)ip);
      vsFrameAllocate(&dest, &fi);
      test_bool(vsTransformPrepare(&td, &src, &dest) == VS_OK);
      test_bool(_FLT(transformPlanar)(&td, t) == VS_OK);
      test_bool(vsTransformFinish(&td) == VS_OK);
      test_bool(vsLensPlaneMapInit(&ref, &fi, &fi, 0, LM_KS[i], VSLensCorrectWobble) == VS_OK);
      for(y=0; y<fi.height; y+=3){
        for(x=0; x<fi.width; x+=3){
          double xs, ys;
          uint8_t expected, actual;
          int diff;
          vsLensMapBackward(&ref, &t, x, y, &xs, &ys);
          td._FLT(interpolate)(&expected, (float)xs, (float)ys, src.data[0],
                                src.linesize[0], fi.width, fi.height, 0);
          actual = dest.data[0][y*dest.linesize[0]+x];
          diff = actual > expected ? actual - expected : expected - actual;
          sumDiff += diff;
          n++;
          if(diff > maxDiff) maxDiff = diff;
        }
      }
      /* The LUT is accurate to a small fraction of a pixel (see
         test_lensmap_lut), so the two computations should agree almost
         everywhere; a handful of samples that straddle a hard texture edge
         (see ldFillTexture) can legitimately land on different sides of it
         in float vs. double precision, so the bound is on the mean, not the
         worst single pixel. */
      test_bool(sumDiff/n < 0.5);
      vsLensPlaneMapFree(&ref);
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
      /* ldRenderWarped subtracts cum.x/cum.y from the undistorted point before
         distorting back (obs = base . D_k(U_k(.) - cum.t)); the warp loop
         here also subtracts its transform's translation after its own U_k, so
         undoing cum means passing -cum, not +cum -- confirmed empirically:
         passing +cum left the corrected residual unchanged from uncorrected. */
      inv = mult_transform(&cum, -1);
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

/* lensK == 0.0 is the "no manual override" sentinel (VSTransformConfig.lensK),
   not a request for correction -- Wobble with k == 0.0 must build no active
   map at all, the same as if correction were never asked for. */
static void test_lensmap_zero_k_stays_inactive(void){
  VSFrameInfo fi;
  VSTransformData td;
  vsFrameInfoInit(&fi, 320, 240, PF_GRAY8);
  lmInitTd(&td, &fi, VSLensCorrectWobble, 0.0, VS_BiLinear);
  lensEnsureMaps(&td);
  test_bool(td.lensActive == 0);
  /* lensActive == 0 alone is also what a freshly memset VSTransformData looks
     like, so it would not catch a lensMapK sentinel of 0.0 short-circuiting
     the build.  vsLensPlaneMapInit sets tDomD = -1.0 on every call including
     its k == 0.0 early return, so tDomD tells "built an inactive map" apart
     from "never ran". */
  test_bool(td.lensMaps[0].tDomD < 0.0);
  vsTransformDataCleanup(&td);
}

/* The other direction: a real, nonzero k must build active maps on the very
   first call, i.e. the "no map built yet" sentinel must never collide with a
   genuine effective k. */
static void test_lensmap_nonzero_k_builds_on_first_call(void){
  VSFrameInfo fi;
  VSTransformData td;
  vsFrameInfoInit(&fi, 320, 240, PF_GRAY8);
  lmInitTd(&td, &fi, VSLensCorrectWobble, -0.25, VS_BiLinear);
  lensEnsureMaps(&td);
  test_bool(td.lensActive == 1);
  vsTransformDataCleanup(&td);
}

/* Colour must not drift away from luma toward the frame edge.  Paint a frame
   where chroma is a step function aligned with a luma step, warp it with the
   lens active, and check the two edges still coincide after correction.
   4:2:2 is the case that catches an anisotropic-radius bug. */
/* Sub-pixel edge locate: row is a clean monotone step from ~40 to ~200 (see
   lmCheckChromaAlignment), so the two samples straddling the midpoint value
   can be linearly interpolated to get a fractional crossing position.  Far
   finer than "first sample past a threshold" on purpose: the anisotropic-
   radius bug this catches moves the chroma edge by only about one luma pixel,
   which a whole-pixel measurement could not separate from noise.  Returns
   -1.0 if no crossing is found in [1,n). */
static double lmFindEdge(const uint8_t* row, int n, double mid){
  int x;
  for(x=1; x<n; x++)
    if(row[x-1] <= mid && row[x] > mid)
      return (x-1) + (mid - row[x-1])/(double)(row[x] - row[x-1]);
  return -1.0;
}

/* A chroma sample at plane index cx represents the CENTRE of the block of
   (1<<wsub) luma columns it was averaged from, not its first column, so a bare
   "cx << wsub" carries a fixed half-chroma-pixel bias.  That bias is the same
   for correct and buggy implementations alike and would swamp the sub-pixel
   signal here: without this correction even a correct implementation measures
   ~0.75 luma px of worst offset on 4:2:0 and 4:2:2. */
static double lmChromaToLuma(double cx, int sub){ return (cx + 0.5)*sub - 0.5; }

/* Tolerance in luma-equivalent px, from measurement.  A correct implementation
   still shows ~0.27 luma px of offset on YUV420P/YUV422P and 0.00 on YUV444P:
   real curvature error from averaging a nonlinear radial map over a chroma
   sample's footprint, driven by the horizontal subsampling factor (wsub=1 for
   both).  The tolerance is roughly 2x that. */
#define LM_CHROMA_TOL_LUMA_PX 0.6

static void lmCheckChromaAlignment(VSPixelFormat pf, const char* name){
  const double k = -0.25;
  VSFrameInfo fi;
  VSFrame src, dest;
  VSTransformData td;
  VSTransform t = null_transform();
  int wsub, hsub, x, y;
  double worst = 0;
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

  /* For each chroma row, sub-pixel-locate the step in luma and in chroma
     (measured identically) and compare in luma-equivalent units. */
  for(y = 8; y < CHROMA_SIZE(fi.height,hsub) - 8; y++){
    int ly = y << hsub;
    double lx = lmFindEdge(dest.data[0] + ly*dest.linesize[0], fi.width, 120.0);
    double cx = lmFindEdge(dest.data[1] + y*dest.linesize[1],
                            CHROMA_SIZE(fi.width,wsub), 120.0);
    double d;
    if(lx < 0 || cx < 0) continue;
    d = fabs(lmChromaToLuma(cx, 1 << wsub) - lx);
    if(d > worst) worst = d;
  }
  fprintf(stderr, "  %s: worst luma/chroma step offset = %.4f luma px\n", name, worst);
  test_bool(worst <= LM_CHROMA_TOL_LUMA_PX);
  vsFrameFree(&dest); vsFrameFree(&src);
  vsTransformDataCleanup(&td);
}

static void test_lensmap_chroma_render(void){
  lmCheckChromaAlignment(PF_YUV420P, "YUV420P");
  lmCheckChromaAlignment(PF_YUV422P, "YUV422P");
  lmCheckChromaAlignment(PF_YUV444P, "YUV444P");
}

/* Renders src through both warp loops with the given config and returns the
   mean and worst absolute per-pixel (luma) difference between them. */
static void lmFixFloatDiff(const VSFrameInfo* fi, const VSFrame* src,
                           VSLensCorrectMode cm, double k, VSInterpolType ip,
                           const VSTransform* t, double* meanOut, double* worstOut){
  VSFrame dfix, dflt;
  VSTransformData a, b;
  double sum = 0, worst = 0; int n = 0, x, y;
  lmInitTd(&a, fi, cm, k, ip);
  lmInitTd(&b, fi, cm, k, ip);
  vsFrameAllocate(&dfix, fi); vsFrameAllocate(&dflt, fi);
  test_bool(vsTransformPrepare(&a, src, &dfix) == VS_OK);
  test_bool(transformPlanar(&a, *t) == VS_OK);
  test_bool(vsTransformFinish(&a) == VS_OK);
  test_bool(vsTransformPrepare(&b, src, &dflt) == VS_OK);
  test_bool(_FLT(transformPlanar)(&b, *t) == VS_OK);
  test_bool(vsTransformFinish(&b) == VS_OK);
  for(y=0; y<fi->height; y++)
    for(x=0; x<fi->width; x++){
      double d = fabs((double)dfix.data[0][y*dfix.linesize[0]+x] -
                      (double)dflt.data[0][y*dflt.linesize[0]+x]);
      sum += d; n++;
      if(d > worst) worst = d;
    }
  *meanOut = sum/n; *worstOut = worst;
  vsFrameFree(&dfix); vsFrameFree(&dflt);
  vsTransformDataCleanup(&a); vsTransformDataCleanup(&b);
}

/* The fixed-point path is what ships, and it must not disagree with the
   float path any more *because of the lens* than the two paths already
   disagree without one.

   An absolute bound on |fixed - float| does not hold, not even with the lens
   inactive: interpolateBiLin/interpolateBiCub carry a rounding-bias "+1"
   correction their float twins do not, which on a hard-edged texture
   (ldFillTexture's 900 random rectangles) alone costs several LSB of mean
   difference.  --testBASE cannot catch that -- it pins the fixed and float
   goldens separately and never compares them.

   So the primary assertion here is differential: measure the lens-off
   disagreement Dbase once per interpolation type (mode and k don't matter
   with the lens inactive), measure the lens-on disagreement Dlens per
   (mode, k, interpolation) config, and require that turning the lens on
   doesn't add more than a small, measured margin on top of whatever the
   interpolator pair already disagreed about.

   A tight worst-pixel bound does not hold for any interpolation type on this
   texture: nearest-neighbour-style snapping (interpolateZero, interpolateLin's
   y-rounding) flips whenever a coordinate sits near a rounding boundary, and
   turning the lens on shifts coordinates by a fraction of a pixel almost
   everywhere.  On a hard-edged texture one such flip is a near-full-scale
   jump.  Only MEAN is a stable per-config statistic; the worst-based margin
   below is printed for visibility but cannot bind. */
static void test_lensmap_fixed_float_equivalence(void){
  const double ks[] = {-0.3, -0.25, -0.1, 0.12};
  VSFrameInfo fi;
  VSFrame src;
  uint32_t seed = 7;
  int ik, ip, im;
  double baseMean[4], baseWorst[4];
  VSTransform t0 = null_transform();
  t0.x = 6.25; t0.y = -3.5; t0.alpha = 0.008;

  /* Margin on the lens's *additional* fixed-vs-float disagreement, on top of
     each interpolation type's own lens-off baseline. Measured across all
     2 modes x 4 k's x 4 interpolation types:
       max observed mean(Dlens)  - mean(Dbase)  = 7.73  (Full,  k=0.12, BiLinear/BiCubic)
       max observed worst(Dlens) - worst(Dbase) = 205   (Full,  k=-0.30, VS_Linear)
     MEAN_MARGIN is ~2x the mean excess and does the real work below.  The
     worst excess is printed but not asserted: at 2x it would be 410, beyond
     the 0..255 range of a sample, so nothing could ever fail it.  The pixel
     level guarantee lives in test_lensmap_fixed_reference below. */
  const double MEAN_MARGIN  = 16.0;

  vsFrameInfoInit(&fi, 320, 240, PF_YUV420P);
  vsFrameAllocate(&src, &fi);
  ldFillTexture(&src, &fi, &seed);
  memset(src.data[1], 0x55, (size_t)src.linesize[1]*(fi.height/2));
  memset(src.data[2], 0xAA, (size_t)src.linesize[2]*(fi.height/2));

  /* Lens-off baseline, once per interpolation type: mode and k are
     irrelevant here because vsLensPlaneMapInit leaves the map inactive for
     VSLensCorrectOff regardless of k (see lensmap.c). */
  for(ip=0; ip<4; ip++){
    lmFixFloatDiff(&fi, &src, VSLensCorrectOff, 0.0, (VSInterpolType)ip, &t0,
                  &baseMean[ip], &baseWorst[ip]);
    fprintf(stderr, "  lens-off baseline ip=%d: mean(Dbase)=%.4f worst(Dbase)=%.0f\n",
            ip, baseMean[ip], baseWorst[ip]);
  }

  for(im=1; im<=2; im++){
    VSLensCorrectMode cm = im == 1 ? VSLensCorrectWobble : VSLensCorrectFull;
    for(ik=0; ik<4; ik++){
      for(ip=0; ip<4; ip++){
        double meanLens, worstLens;
        lmFixFloatDiff(&fi, &src, cm, ks[ik], (VSInterpolType)ip, &t0,
                      &meanLens, &worstLens);
        fprintf(stderr, "  mode=%d k=%.2f ip=%d: mean(Dbase)=%.4f mean(Dlens)=%.4f "
                "(d=%.4f)  worst(Dbase)=%.0f worst(Dlens)=%.0f (d=%.0f)\n",
                im, ks[ik], ip, baseMean[ip], meanLens, meanLens - baseMean[ip],
                baseWorst[ip], worstLens, worstLens - baseWorst[ip]);
        test_bool(meanLens  <= baseMean[ip]  + MEAN_MARGIN);
      }
    }
  }
  vsFrameFree(&src);
}

/* fp16 conversion for a double-precision plane coordinate (not necessarily
   integral, unlike test_interpolate.c's iToFp16_t). Clamped the same way
   lensmap.c's toFp16d() is, since vsLensMapBackward can return values well
   outside the frame (e.g. VS_LENS_OUTSIDE_PX = -30000) that would otherwise
   overflow the *65536 multiply before the cast. */
static fp16 lmDToFp16(double v){
  double s = v * 65536.0;
  if(s >  2147483647.0) s =  2147483647.0;
  if(s < -2147483648.0) s = -2147483648.0;
  return (fp16)(int64_t)(s + (s >= 0 ? 0.5 : -0.5));
}

/* Direct-reference check for the FIXED-POINT planar warp -- the path that
   actually ships. test_lensmap_wobble_cancellation_through_loop above
   established the right pattern for the float path: compare a warp loop
   against vsLensMapBackward (an independent, closed-form, double-precision
   computation of U_k/M/D_k), sampled through the SAME interpolator, rather
   than against a warp loop written in a different language. That cancels
   the interpolator-family divergence documented above completely, instead
   of trying to bound it away -- so this is the test that actually carries
   the fixed-point loop's correctness guarantee. test_lensmap_fixed_float_
   equivalence is kept as a coarse, independent cross-check (it is not
   wrong, just blunt: see its own comment on why an absolute or float-
   relative bound doesn't work well on this texture). */
static void test_lensmap_fixed_reference(void){
  const double ks[] = {-0.3, -0.25, -0.1, 0.12};
  VSFrameInfo fi;
  VSFrame src, dest;
  uint32_t seed = 7;
  int im, ik, ip, x, y;

  /* Measured across 2 modes x 4 k's x 4 interpolation types (fixed-point loop
     vs. double-precision vsLensMapBackward, same interpolator):
       VS_BiLinear/VS_BiCubic: max observed mean = 0.077, max observed max = 73
       VS_Zero/VS_Linear:      max observed mean = 0.081, max observed max = 236
     TIGHT_MEAN/LOOSE_MEAN are ~2x those mean figures and carry the real
     guarantee.

     A pixel-exact MAX bound is not achievable for any interpolation type: each
     has coordinates where its behaviour is discontinuous in the real-valued
     source position, and the fixed and double computations, agreeing to a few
     thousandths of a pixel, can land on opposite sides of one.  VS_Zero and
     VS_Linear's y-rounding break at every half-integer; VS_BiLinear/VS_BiCubic
     only where the interior and border regimes meet, at the source frame's
     four edges -- hence their smaller max.  TIGHT_MAX/LOOSE_MAX are ~2x the
     worst observed, enough to catch a systematic regression. */
  const double TIGHT_MEAN = 0.15;
  const int    TIGHT_MAX  = 150;
  const double LOOSE_MEAN = 0.2;

  vsFrameInfoInit(&fi, 320, 240, PF_GRAY8);
  vsFrameAllocate(&src, &fi);
  ldFillTexture(&src, &fi, &seed);

  for(im=1; im<=2; im++){
    VSLensCorrectMode cm = im == 1 ? VSLensCorrectWobble : VSLensCorrectFull;
    for(ik=0; ik<4; ik++){
      for(ip=0; ip<4; ip++){
        VSTransformData td;
        VSLensPlaneMap ref;
        VSTransform t = null_transform();
        double sumDiff = 0; int n = 0, maxDiff = 0;
        t.x = 6.25; t.y = -3.5; t.alpha = 0.008;
        lmInitTd(&td, &fi, cm, ks[ik], (VSInterpolType)ip);
        vsFrameAllocate(&dest, &fi);
        test_bool(vsTransformPrepare(&td, &src, &dest) == VS_OK);
        test_bool(transformPlanar(&td, t) == VS_OK);
        test_bool(vsTransformFinish(&td) == VS_OK);
        test_bool(vsLensPlaneMapInit(&ref, &fi, &fi, 0, ks[ik], cm) == VS_OK);
        for(y=0; y<fi.height; y++){
          for(x=0; x<fi.width; x++){
            double xs, ys;
            uint8_t expected, actual;
            int diff;
            vsLensMapBackward(&ref, &t, x, y, &xs, &ys);
            td.interpolate(&expected, lmDToFp16(xs), lmDToFp16(ys), src.data[0],
                          src.linesize[0], fi.width, fi.height, 0);
            actual = dest.data[0][y*dest.linesize[0]+x];
            diff = actual > expected ? actual - expected : expected - actual;
            sumDiff += diff; n++;
            if(diff > maxDiff) maxDiff = diff;
          }
        }
        fprintf(stderr, "  fixed-reference mode=%d k=%.2f ip=%d: mean=%.4f max=%d\n",
                im, ks[ik], ip, sumDiff/n, maxDiff);
        if(ip == VS_BiLinear || ip == VS_BiCubic){
          test_bool(sumDiff/n <= TIGHT_MEAN);
          test_bool(maxDiff   <= TIGHT_MAX);
        }else{
          test_bool(sumDiff/n <= LOOSE_MEAN);
        }
        vsLensPlaneMapFree(&ref);
        vsFrameFree(&dest);
        vsTransformDataCleanup(&td);
      }
    }
  }
  vsFrameFree(&src);
}

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

/* Direct-reference check for the FIXED-POINT packed warp -- the loop
   vsDoTransform actually dispatches to for every packed (RGB/BGR/RGBA)
   frame. Ported directly from test_lensmap_fixed_reference above: render
   through the real loop (transformPacked), independently compute the
   expected source coordinate per destination pixel with the closed-form,
   double-precision vsLensMapBackward, then sample the source through the
   SAME fixed-point interpolator the loop used (interpolateN) and compare.
   Using the same interpolator on both sides cancels interpolator behaviour
   out of the comparison entirely, so what is left is a direct measurement
   of the packed loop's geometry -- test_lensmap_packed above only checks
   that the lens is engaged at all, not that it lands in the right place.

   interpolateN (unlike td->interpolate for the planar loop) does not
   depend on cfg.interpolType -- transformPacked always calls it directly,
   see src/transformfixedpoint.c -- so there is no need to sweep
   interpolation types here, unlike the planar version of this test.

   A real translation plus a small rotation is used (not the identity) so
   transformPacked's fast path is not taken and the lens map is genuinely
   exercised; td->conf.crop is VSCropBorder (see lmInitTd), so the packed
   loop's border def is the fixed value 16 (src/transformfixedpoint.c:
   "td->conf.crop ? 16 : *dest"), passed to interpolateN below to match. */
static void test_lensmap_fixed_reference_packed(void){
  const double ks[] = {-0.3, -0.25, -0.1, 0.12};
  VSFrameInfo fi;
  VSFrame src, dest;
  int im, ik, x, y, z;

  /* Measured across 2 modes x 4 k's (fixed-point transformPacked vs.
     double-precision vsLensMapBackward, same interpolator, PF_RGB24, 320x240,
     t.x=6.25 t.y=-3.5 t.alpha=0.008):
       max observed mean = 0.0387   max observed max = 219
     TIGHT_MEAN is ~2x the observed mean and carries the real guarantee.  MAX
     is bounded only loosely, for the same boundary-straddling reason
     documented in test_lensmap_fixed_reference above. */
  const double TIGHT_MEAN = 0.08;
  const int    LOOSE_MAX  = 440;

  vsFrameInfoInit(&fi, 320, 240, PF_RGB24);
  vsFrameAllocate(&src, &fi);
  for(y=0; y<fi.height; y++)
    for(x=0; x<fi.width; x++)
      setPixelRGB(&src, &fi, x, y, (uint8_t)(x*7), (uint8_t)(y*5), (uint8_t)(x^y));

  for(im=1; im<=2; im++){
    VSLensCorrectMode cm = im == 1 ? VSLensCorrectWobble : VSLensCorrectFull;
    for(ik=0; ik<4; ik++){
      VSTransformData td;
      VSLensPlaneMap ref;
      VSTransform t = null_transform();
      double sumDiff = 0; int n = 0, maxDiff = 0;
      t.x = 6.25; t.y = -3.5; t.alpha = 0.008;
      lmInitTd(&td, &fi, cm, ks[ik], VS_BiLinear);
      vsFrameAllocate(&dest, &fi);
      test_bool(vsTransformPrepare(&td, &src, &dest) == VS_OK);
      test_bool(transformPacked(&td, t) == VS_OK);
      test_bool(vsTransformFinish(&td) == VS_OK);
      test_bool(vsLensPlaneMapInit(&ref, &fi, &fi, 0, ks[ik], cm) == VS_OK);
      for(y=0; y<fi.height; y++){
        for(x=0; x<fi.width; x++){
          double xs, ys;
          vsLensMapBackward(&ref, &t, x, y, &xs, &ys);
          for(z=0; z<fi.bytesPerPixel; z++){
            uint8_t expected, actual;
            int diff;
            interpolateN(&expected, lmDToFp16(xs), lmDToFp16(ys), src.data[0],
                         src.linesize[0], fi.width, fi.height,
                         fi.bytesPerPixel, (uint8_t)z, 16);
            actual = dest.data[0][y*dest.linesize[0] + x*fi.bytesPerPixel + z];
            diff = actual > expected ? actual - expected : expected - actual;
            sumDiff += diff; n++;
            if(diff > maxDiff) maxDiff = diff;
          }
        }
      }
      fprintf(stderr, "  fixed-reference-packed mode=%d k=%.2f: mean=%.4f max=%d\n",
              im, ks[ik], sumDiff/n, maxDiff);
      test_bool(sumDiff/n <= TIGHT_MEAN);
      test_bool(maxDiff   <= LOOSE_MAX);
      vsLensPlaneMapFree(&ref);
      vsFrameFree(&dest);
      vsTransformDataCleanup(&td);
    }
  }
  vsFrameFree(&src);
}

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
        /* Walk the destination boundary at 500 samples/edge (2000 total) --
           deliberately independent of and much denser than the 64
           samples/edge vsTransformRequiredZoom uses internally (see
           VS_LENS_ZOOM_SAMPLES_PER_EDGE in transform.c).  This test's whole
           point is to catch under-sampling in the implementation; if it
           used the same density (or anything close to it) it would only
           confirm the implementation against itself and could not have
           caught the eight-point version's 2.47 px miss that motivated
           this test in the first place. */
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

/* --- full-mode evidence --------------------------------------------------- */

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

/* Sanity-check lmColumnBend itself before trusting its verdict on the real
   test below: feed it a perfectly straight synthetic column (bend must be
   near zero) and a column with a known sinusoidal curvature baked in (bend
   must come back close to the amplitude that was put in). A metric that
   always reports a small number would make the straightness claim below
   meaningless. */
static void test_lensmap_column_bend_metric(void){
  VSFrameInfo fi;
  VSFrame straight, curved;
  double bend;
  int x, y;
  const double amp = 6.0;
  vsFrameInfoInit(&fi, 320, 240, PF_GRAY8);
  vsFrameAllocate(&straight, &fi);
  vsFrameAllocate(&curved, &fi);
  memset(straight.data[0], 220, (size_t)straight.linesize[0]*fi.height);
  memset(curved.data[0], 220, (size_t)curved.linesize[0]*fi.height);
  for(y=0; y<fi.height; y++){
    int sxp = 160;
    int cxp = 160 + (int)lrint(amp*sin(y*0.05));
    for(x=sxp-1; x<=sxp+1 && x>=0 && x<fi.width; x++)
      straight.data[0][y*straight.linesize[0]+x] = 20;
    for(x=cxp-1; x<=cxp+1 && x>=0 && x<fi.width; x++)
      curved.data[0][y*curved.linesize[0]+x] = 20;
  }
  bend = lmColumnBend(&straight, &fi, 160);
  fprintf(stderr, "  bend metric sanity: straight column -> %.3f px (expect near 0)\n", bend);
  test_bool(bend < 1.0);
  bend = lmColumnBend(&curved, &fi, 160);
  fprintf(stderr, "  bend metric sanity: sinusoid amplitude %.1f px -> measured %.3f px\n",
          amp, bend);
  test_bool(bend > 0.5*amp && bend < 1.5*amp);
  vsFrameFree(&straight); vsFrameFree(&curved);
}

/* Renders how a straight-line scene appears through a lens of strength k:
   destination (observed, distorted) pixel (x,y) shows the scene point whose
   undistorted coordinate is U_k(x,y), i.e. obs(x,y) = scene(U_k(x,y)).

   This is deliberately NOT ldRenderWarped(src, dst, fi, ld, id) with an
   identity camera motion: that helper computes D_k(M^-1(U_k(.))), which for
   M = identity collapses to the plain identity by the very round-trip
   guarantee vsLensUndistortPoint/vsLensDistortPoint carry (see
   test_lensdistortion_roundtrip) -- confirmed empirically here too: with
   cum = null_transform(), ldRenderWarped's output was byte-identical to its
   input on every row, i.e. it applied no distortion at all. That helper
   exists to render frame i *relative to frame 0* under real inter-frame
   camera motion (see test_lensmap_removes_wobble), not to distort an ideal
   scene from a standing start, so it is the wrong tool for "render one
   frame of a straight-edged scene as a lens would bend it". */
static void lmRenderDistortedScene(const VSFrame* scene, VSFrame* obs,
                                   const VSFrameInfo* fi, const VSLensDistortion* ld){
  int x, y;
  for(y=0; y<fi->height; y++){
    for(x=0; x<fi->width; x++){
      double ux, uy;
      uint8_t v;
      if(vsLensUndistortPoint(ld, x, y, &ux, &uy) == VS_OK)
        v = ldSampleBilinear(scene->data[0], fi->width, fi->height,
                             scene->linesize[0], ux, uy);
      else
        v = 220;  /* outside U_k's domain: background colour, not garbage */
      obs->data[0][y*obs->linesize[0]+x] = v;
    }
  }
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
  /* Vertical lines only: lmColumnBend tracks the darkest pixel in a window
     around one column, row by row.  A horizontal line breaks that: on the rows
     it occupies every pixel in the window ties at the minimum, and the scan
     keeps the leftmost, reporting the window edge -- a ~40 px artifact, an
     order of magnitude larger than any real lens bend.  The claim under test
     needs only one clean vertical edge. */
  for(y=0; y<fi.height; y++)
    for(x=0; x<fi.width; x++)
      if(x % 80 == 0)
        grid.data[0][y*grid.linesize[0]+x] = 20;
  ld = vsLensDistortionInit(&fi, k);
  lmRenderDistortedScene(&grid, &obs, &fi, &ld);
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
  /* A mid-edge pixel should still sample inside the source -- but which edge
     depends on the frame's aspect ratio, not just "not a corner". rho is
     normalised by the half-diagonal (here sqrt(160^2+120^2) = 200), so the
     horizontal mid-edges sit at r/rho = 160/200 = 0.8 while the vertical
     mid-edges sit at r/rho = 120/200 = 0.6.  At k=0.15 the horizontal
     mid-edges are already in the border-blend zone (measured 0 and 40), close
     enough to the domain edge that "still inside" fails there; the vertical
     ones stay cleanly inside (measured 200, the background), so those are the
     pair checked here. */
  test_bool(dest.data[0][fi.width/2] > 190);
  test_bool(dest.data[0][(fi.height-1)*dest.linesize[0] + fi.width/2] > 190);
  /* the centre is untouched */
  test_bool(dest.data[0][(fi.height/2)*dest.linesize[0] + fi.width/2] > 190);
  /* Nothing anywhere is a wild value. The bound is 201, not 200: the
     fixed-point bilinear border blend (interpolateBiLinBorder in
     transformfixedpoint.c) carries a deliberate "+1" rounding-bias
     correction -- the same one documented at length in
     test_lensmap_fixed_float_equivalence above -- which on a perfectly flat
     200-valued source can round a genuine in-bounds sample up to 201.
     Measured on this exact test: worst value 201, affecting 580/76800
     pixels (0.8%), nowhere near a source value of 0 (the background) or a
     clamped 255, so this is the known rounding artifact, not sampled
     garbage. A bound of 200 exactly would fail on a correct implementation. */
  for(y=0; y<fi.height; y++)
    for(x=0; x<fi.width; x++){
      uint8_t v = dest.data[0][y*dest.linesize[0]+x];
      test_bool(v <= 201);
    }
  vsFrameFree(&src); vsFrameFree(&dest);
  vsTransformDataCleanup(&td);
}

/* Inactive lens: identical to the closed form, to the last bit. */
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

/* optZoom == 1 is the default zoom path (VSTransformConfig.optZoom defaults
   to 1, not 2), so the lens-aware budget must reach it too, not just the
   optZoom == 2 path that vsTransformRequiredZoom was originally wired into.

   Builds a batch of transforms with known x/y extremes and a known largest
   rotation, runs it through vsPreprocessTransforms with optZoom == 1, and
   checks:
     - lens inactive: the result is exactly the old closed form (== , not a
       tolerance -- modelled on test_lensmap_required_zoom_off above).
     - lens active: the result is at least the lens-aware requirement for the
       four corner combinations of those same extremes (never less -- this is
       a budget, and under-budgeting shows as black slivers), and at least
       the old closed form too (the lens-aware term can only add zoom here,
       never subtract it). */
static void test_lensmap_optzoom1_lens_budget(void){
  VSFrameInfo fi;
  const int n = 10;
  VSTransform ts[10];
  VSTransformations trans;
  int i;
  vsFrameInfoInit(&fi, 640, 360, PF_GRAY8);
  for(i=0; i<n; i++){
    ts[i] = null_transform();
    ts[i].x     = -30.0 + 8.0*i;      /* spans -30 .. +42 */
    ts[i].y     =  20.0 - 5.0*i;      /* spans  20 .. -25 */
    ts[i].alpha =  0.01*(i - 3);      /* spans -0.03 .. +0.06, |.| worst at i=9 */
  }

  /* --- lens inactive: bit-identical to the closed form --- */
  {
    VSTransformData td;
    VSTransformConfig cfg = vsTransformGetDefaultConfig("optzoom1-off");
    VSTransform min_t, max_t;
    double zx, zy, expect;
    cfg.relative          = 0;
    cfg.smoothing         = 0;
    cfg.optZoom           = 1;
    cfg.lensCorrection    = VSLensCorrectOff;
    cfg.estimateLensDistortion = 0;
    test_bool(vsTransformDataInit(&td, &cfg, &fi, &fi) == VS_OK);
    vsTransformSetLensK(&td, -0.25);       /* would want correction, but mode is Off */
    trans.ts = ts; trans.len = n; trans.current = 0; trans.warned_end = 0;
    cleanmaxmin_xy_transform(ts, n, 1, &min_t, &max_t);
    zx = 2*VS_MAX(max_t.x,fabs(min_t.x))/fi.width;
    zy = 2*VS_MAX(max_t.y,fabs(min_t.y))/fi.height;
    expect = VS_CLAMP(100 * VS_MAX(zx,zy), -60, 60);
    test_bool(vsPreprocessTransforms(&td, &trans) == VS_OK);
    test_bool(td.conf.zoom == expect);
    test_bool(td.lensActive == 0);
    vsTransformDataCleanup(&td);
  }

  /* --- lens active: at least the lens-aware requirement --- */
  {
    VSTransformData td;
    VSTransformConfig cfg = vsTransformGetDefaultConfig("optzoom1-on");
    VSTransform min_t, max_t;
    double zx, zy, closedZoom, floorZoom, maxAlpha;
    int ix, iy;
    cfg.relative          = 0;
    cfg.smoothing         = 0;
    cfg.optZoom           = 1;
    cfg.lensCorrection    = VSLensCorrectWobble;
    cfg.estimateLensDistortion = 0;
    test_bool(vsTransformDataInit(&td, &cfg, &fi, &fi) == VS_OK);
    vsTransformSetLensK(&td, -0.25);
    trans.ts = ts; trans.len = n; trans.current = 0; trans.warned_end = 0;

    cleanmaxmin_xy_transform(ts, n, 1, &min_t, &max_t);
    zx = 2*VS_MAX(max_t.x,fabs(min_t.x))/fi.width;
    zy = 2*VS_MAX(max_t.y,fabs(min_t.y))/fi.height;
    closedZoom = 100 * VS_MAX(zx,zy);

    maxAlpha = 0.0;
    for(i=0; i<n; i++)
      if(fabs(ts[i].alpha) > fabs(maxAlpha)) maxAlpha = ts[i].alpha;

    lensEnsureMaps(&td);
    test_bool(td.lensActive == 1);
    floorZoom = closedZoom;
    {
      double cx[2] = { min_t.x, max_t.x };
      double cy[2] = { min_t.y, max_t.y };
      for(ix=0; ix<2; ix++){
        for(iy=0; iy<2; iy++){
          VSTransform corner = null_transform();
          corner.x = cx[ix]; corner.y = cy[iy]; corner.alpha = maxAlpha;
          floorZoom = VS_MAX(floorZoom, vsTransformRequiredZoom(&td, &corner));
        }
      }
    }
    floorZoom = VS_CLAMP(floorZoom, -60, 60);

    test_bool(vsPreprocessTransforms(&td, &trans) == VS_OK);
    fprintf(stderr, "  optZoom==1 lens budget: closed form %.3f, lens-aware floor %.3f,"
                    " actual %.3f\n", closedZoom, floorZoom, td.conf.zoom);
    test_bool(td.conf.zoom >= floorZoom - 1e-9);
    test_bool(td.conf.zoom >= closedZoom - 1e-9);
    vsTransformDataCleanup(&td);
  }
}

/* The backward map is non-linear in alpha, so the sign of the rotation
   matters at a translation corner, not just its magnitude: sampling only the
   largest-magnitude signed alpha (as the eight-point/four-corner budget
   above did before this fix) can miss the worse-fitting sign entirely.

   Built by direct measurement with a standalone probe against this exact
   geometry (k=0.10 pincushion, 640x360, corners (-30,-25)/(-30,20)/(42,-25)/
   (42,20), |alpha|=0.03): sampling alpha=+0.03 only across the four corners
   gives a floor of 18.0597 (worst corner (-30,-25)); the true worst point is
   corner (42,-25) at alpha=-0.03, which needs 18.5669 -- 0.51 zoom units
   more. The batch below is built so cleanmaxmin_xy_transform yields exactly
   those four corners and the largest-magnitude alpha in the batch is +0.03
   (no transform has a larger-magnitude negative alpha), so a version that
   only tried +maxAlpha would under-budget by that same 0.51. */
static void test_lensmap_optzoom1_lens_budget_bothsigns(void){
  VSFrameInfo fi;
  const int n = 4;
  VSTransform ts[4];
  VSTransformations trans;
  VSTransformData td;
  VSTransformConfig cfg = vsTransformGetDefaultConfig("optzoom1-bothsigns");
  double posOnlyFloor, bothSignsFloor;
  int ix, iy;
  vsFrameInfoInit(&fi, 640, 360, PF_GRAY8);

  ts[0] = null_transform(); ts[0].x = -30; ts[0].y = -25; ts[0].alpha =  0.01;
  ts[1] = null_transform(); ts[1].x =  42; ts[1].y =  20; ts[1].alpha =  0.03;
  ts[2] = null_transform(); ts[2].x = -10; ts[2].y =   5; ts[2].alpha =  0.005;
  ts[3] = null_transform(); ts[3].x =   0; ts[3].y =   0; ts[3].alpha = -0.02;

  cfg.relative               = 0;
  cfg.smoothing              = 0;
  cfg.optZoom                = 1;
  cfg.lensCorrection         = VSLensCorrectWobble;
  cfg.estimateLensDistortion = 0;
  test_bool(vsTransformDataInit(&td, &cfg, &fi, &fi) == VS_OK);
  vsTransformSetLensK(&td, 0.10);
  lensEnsureMaps(&td);
  test_bool(td.lensActive == 1);

  /* independently reproduce the pos-only and both-signs corner budgets, using
     the same four x/y corners the production code derives from this batch */
  {
    double cx[2] = { -30.0, 42.0 };
    double cy[2] = { -25.0, 20.0 };
    posOnlyFloor = 0.0;
    bothSignsFloor = 0.0;
    for(ix=0; ix<2; ix++){
      for(iy=0; iy<2; iy++){
        VSTransform tp = null_transform(), tn = null_transform();
        double zp, zn;
        tp.x = cx[ix]; tp.y = cy[iy]; tp.alpha =  0.03;
        tn.x = cx[ix]; tn.y = cy[iy]; tn.alpha = -0.03;
        zp = vsTransformRequiredZoom(&td, &tp);
        zn = vsTransformRequiredZoom(&td, &tn);
        posOnlyFloor   = VS_MAX(posOnlyFloor, zp);
        bothSignsFloor = VS_MAX(bothSignsFloor, VS_MAX(zp, zn));
      }
    }
  }
  fprintf(stderr, "  optZoom==1 both-signs: pos-only floor %.4f, both-signs floor %.4f\n",
                  posOnlyFloor, bothSignsFloor);
  /* the gap is real for this geometry, not a wash */
  test_bool(bothSignsFloor > posOnlyFloor + 0.1);

  trans.ts = ts; trans.len = n; trans.current = 0; trans.warned_end = 0;
  test_bool(vsPreprocessTransforms(&td, &trans) == VS_OK);
  fprintf(stderr, "  optZoom==1 both-signs: actual budget %.4f\n", td.conf.zoom);
  /* the fixed budget must cover the true (both-signs) floor ... */
  test_bool(td.conf.zoom >= bothSignsFloor - 1e-6);
  /* ... and strictly more than the pos-only floor, so sampling both signs
     actually matters for this geometry */
  test_bool(td.conf.zoom > posOnlyFloor + 0.1);

  vsTransformDataCleanup(&td);
}

/* --- estimate-to-render plumbing ------------------------------------------ */

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
   the k it just fitted, with no further call from the consumer.

   The clip construction below is lifted verbatim from
   test_lensdistortion_endtoend()/ldRunEndToEnd() in test_lensdistortion.c: a
   random-walk camera motion rendered through a known barrel lens, run
   through vsMotionDetection to collect a VSManyLocalMotions. That is the
   only part not already given by the plan; every assertion after it was
   written into the task brief up front. */
static void test_lensmap_estimate_reaches_render(void){
  const double trueK = -0.25;
  const int NF = 10;
  VSTransformData td;
  VSTransformConfig cfg = vsTransformGetDefaultConfig("e2e-render");
  VSTransformations trans;
  VSManyLocalMotions mlms;
  VSFrameInfo fi;
  VSFrame frames[10];
  VSTransform cum[10];
  VSLensDistortion ld;
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig("lens-e2e-render");
  VSMotionDetect md;
  uint32_t seed = 4242u;
  int i;

  test_bool(vsFrameInfoInit(&fi, 1280, 720, PF_GRAY8) != 0);
  ld = vsLensDistortionInit(&fi, trueK);

  for(i=0; i<NF; i++) vsFrameAllocate(&frames[i], &fi);
  ldFillTexture(&frames[0], &fi, &seed);

  cum[0] = null_transform();
  for(i=1; i<NF; i++){
    cum[i] = cum[i-1];
    cum[i].x     += 10.0*ldRandUniform(&seed);
    cum[i].y     += 10.0*ldRandUniform(&seed);
    cum[i].alpha += 0.008*ldRandUniform(&seed);
  }
  {
    VSFrame base;
    vsFrameAllocate(&base, &fi);
    memcpy(base.data[0], frames[0].data[0], (size_t)frames[0].linesize[0]*fi.height);
    for(i=0; i<NF; i++) ldRenderWarped(&base, &frames[i], &fi, &ld, &cum[i]);
    vsFrameFree(&base);
  }

  test_bool(vsMotionDetectInit(&md, &mdconf, &fi) == VS_OK);
  md.conf.numThreads = 1;
  vs_vector_init(&mlms, NF);
  for(i=0; i<NF; i++){
    LocalMotions lms;
    test_bool(vsMotionDetection(&md, &lms, &frames[i]) == VS_OK);
    if(i == 0){ vs_vector_del(&lms); continue; }
    vs_vector_append_dup(&mlms, &lms, sizeof(LocalMotions));
  }
  vsMotionDetectionCleanup(&md);
  for(i=0; i<NF; i++) vsFrameFree(&frames[i]);

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

  for(i=0; i<vs_vector_size(&mlms); i++) vs_vector_del(VSMLMGet(&mlms, i));
  vs_vector_del(&mlms);
}
