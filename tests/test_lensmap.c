/* Unit tests for the render-path lens map: the scale functions, the round trip
   that the wobble mode's safety rests on, and the lookup tables. */

#include "lensmap.h"

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

/* Chroma planes: the map must be the luma map expressed in plane units.

   alpha is deliberately left at 0 here.  The legacy affine step (which
   vsLensMapBackward must reproduce exactly, for the k=0 bit-exactness
   guard) applies one shared sin/cos pair in plane coordinates to every
   plane, regardless of that plane's own subsampling.  For 4:2:0 that is
   harmless because the plane is subsampled isotropically (wsub==hsub), but
   for 4:2:2 (wsub=1, hsub=0) it is not: rotation mixes an x-offset and a
   y-offset that are expressed in different luma-equivalent scales on that
   plane, so a nonzero alpha makes the luma point and its chroma counterpart
   genuinely diverge -- by design, see docs/superpowers/specs/
   2026-08-09-lens-correction-render-design.md section 2.3, which states
   this work neither fixes nor replicates a per-axis-consistent rotation for
   anisotropic chroma. Zeroing alpha here isolates the thing this module is
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
