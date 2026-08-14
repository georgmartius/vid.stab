/* Pins the output of the warp loops on the k=0 / lens-inactive path.  The
   golden CRCs are toolchain specific: the test prints what it got, so an
   intentional change is a copy-paste away. */

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

/* Runs one transform through the real pipeline into the caller's frame and
   returns the output CRC. */
static uint32_t tbRunInto(VSFrame* dest, const VSFrame* src, const VSFrameInfo* fi,
                          VSInterpolType ip, int useFloat, int i){
  VSTransformData td;
  VSTransformConfig cfg = vsTransformGetDefaultConfig("baseline");
  uint32_t crc;
  cfg.interpolType = ip;
  cfg.crop         = VSCropBorder;   /* deterministic: no dependence on history */
  cfg.optZoom      = 0;
  test_bool(vsTransformDataInit(&td, &cfg, fi, fi) == VS_OK);
  test_bool(vsTransformPrepare(&td, src, dest) == VS_OK);
  if(useFloat){
    if(fi->pFormat < PF_PACKED) test_bool(_FLT(transformPlanar)(&td, tbTransform(i)) == VS_OK);
    else                        test_bool(_FLT(transformPacked)(&td, tbTransform(i)) == VS_OK);
  }else{
    test_bool(vsDoTransform(&td, tbTransform(i)) == VS_OK);
  }
  test_bool(vsTransformFinish(&td) == VS_OK);
  crc = tbCrcFrame(dest, fi);
  vsTransformDataCleanup(&td);
  return crc;
}

/* Same, for callers that only want the CRC. */
static uint32_t tbRun(const VSFrame* src, const VSFrameInfo* fi,
                      VSInterpolType ip, int useFloat, int i){
  VSFrame dest;
  uint32_t crc;
  vsFrameAllocate(&dest, fi);
  crc = tbRunInto(&dest, src, fi, ip, useFloat, i);
  vsFrameFree(&dest);
  return crc;
}

typedef struct { int max; double mean; int over8; int n; } TBDiff;

/* Per-byte difference statistics between two frames, over every plane. */
static TBDiff tbDiff(const VSFrame* a, const VSFrame* b, const VSFrameInfo* fi){
  TBDiff r;
  double sum = 0;
  int plane, x, y;
  r.max = 0; r.over8 = 0; r.n = 0; r.mean = 0;
  for(plane=0; plane<fi->planes; plane++){
    int w = CHROMA_SIZE(fi->width,  vsGetPlaneWidthSubS(fi, plane));
    int h = CHROMA_SIZE(fi->height, vsGetPlaneHeightSubS(fi, plane));
    for(y=0; y<h; y++){
      const uint8_t* pa = a->data[plane] + (size_t)y*a->linesize[plane];
      const uint8_t* pb = b->data[plane] + (size_t)y*b->linesize[plane];
      for(x=0; x<w; x++){
        int d = (int)pa[x] - (int)pb[x];
        if(d < 0) d = -d;
        if(d > r.max) r.max = d;
        if(d > 8) r.over8++;
        sum += d;
        r.n++;
      }
    }
  }
  if(r.n) r.mean = sum / r.n;
  return r;
}

/* Mean |fixed - float| for one case, for the regeneration printer below. */
static double tbMeanFF(const VSFrame* src, const VSFrameInfo* fi,
                       VSInterpolType ip, int i){
  VSFrame a, b;
  double m;
  vsFrameAllocate(&a, fi);
  vsFrameAllocate(&b, fi);
  tbRunInto(&a, src, fi, ip, 0, i);
  tbRunInto(&b, src, fi, ip, 1, i);
  m = tbDiff(&a, &b, fi).mean;
  vsFrameFree(&a);
  vsFrameFree(&b);
  return m;
}

/* Deterministic analytic pattern for the packed source, so the packed warp
   loops (transformPacked / _FLT(transformPacked)) are exercised too -- a
   PF_YUV420P frame never reaches them since fi->pFormat < PF_PACKED always
   picks the planar branch in tbRun(). No rand() involved, just an
   x/y-dependent pattern with enough structure to be sensitive to warping. */
static void tbFillPackedTexture(VSFrame* f, const VSFrameInfo* fi){
  int x, y;
  for(y=0; y<fi->height; y++)
    for(x=0; x<fi->width; x++)
      setPixelRGB(f, fi, x, y, (uint8_t)(x*7), (uint8_t)(y*5), (uint8_t)(x^y));
}

/* Filled in by step 3.  Order: [interpolation 0..3][transform 0..4]. */
static const uint32_t TB_GOLD_FIXED[4][TB_NUM_T] = {
  {0xEE415B68u,0xB1106B86u,0x3652C8D0u,0x6CF2793Au,0x5FB27C2Au,},
  {0xEE415B68u,0x45DA3139u,0x5A91A23Au,0x8806F2F5u,0x31EDC942u,},
  {0xEE415B68u,0x6D974C04u,0x72B0F49Cu,0x252E6A3Bu,0xF1999AC2u,},
  {0xEE415B68u,0x0FC226D7u,0xB60B7C0Eu,0xF98197D5u,0xF02D480Cu,},
};
/* Same shape, but for a PF_RGB24 (packed) source, so transformPacked is
   pinned too. */
static const uint32_t TB_GOLD_PACKED_FIXED[4][TB_NUM_T] = {
  {0x95C3E009u,0x5510ACC8u,0xE6DDC561u,0x14936393u,0x4FBCDF2Au,},
  {0x95C3E009u,0x5510ACC8u,0xE6DDC561u,0x14936393u,0x4FBCDF2Au,},
  {0x95C3E009u,0x5510ACC8u,0xE6DDC561u,0x14936393u,0x4FBCDF2Au,},
  {0x95C3E009u,0x5510ACC8u,0xE6DDC561u,0x14936393u,0x4FBCDF2Au,},
};

/* The float loops are NOT pinned by CRC: a CRC of float output is not stable
   across optimisation levels (measured at -O0 vs -O1 on one compiler, with
   -ffast-math and -ffp-contract ruled out), so it pins a quantity the language
   does not promise.  They are held against the fixed-point loops instead --
   same source, transform and interpolation, bounded per-byte difference.  The
   fixed-point CRCs above are exact, integer arithmetic being reproducible, so
   the pair cannot both drift unnoticed.

   The metric is the MEAN absolute per-byte difference, not the worst one.
   That is forced by the data: on this deliberately high-frequency source a
   sub-pixel difference in where a sample lands can flip a pixel to an
   unrelated value, so the worst difference runs to 200+
   even when the two implementations agree everywhere that matters. Measured
   across all 20 cases, worst |fixed-float| per case:

     planar  max 0..245, mean 0.0000..3.39
     packed  max 0..249, mean 0.0000..26.91

   The means are what carry information. They are also stable where the CRC
   was not: between -O0 and -O3 every max and every over-8 count is identical
   and the means move only in the fourth decimal (3.1573 -> 3.1572), because
   the optimiser moves a handful of pixels slightly rather than changing the
   picture. test_lensmap_fixed_float_equivalence reached the same conclusion
   independently for the lens-active path, and records there why gating on
   the worst difference would be a dead check.

   So the float loops are pinned the same way as the fixed ones -- a golden
   table, regenerable the same way -- but of the mean rather than a CRC, and
   with an explicit drift allowance rather than none. That is the whole fix:
   the old check demanded bit-exactness from a quantity the language does not
   promise; this one demands stability from a quantity that measurably has it.

   A single global bound was tried first and rejected: the per-case baselines
   span 0.0000 to 26.91, so one bound loose enough for the largest is far too
   loose for the rest. Injecting a half-pixel error into the float planar loop
   moved every non-identity case by 2.2 to 4.6, but only ONE case crossed a
   global bound of 6.0. Against the table below the same injection fails 16 of
   the 20 planar checks -- the four it does not are the identity transform,
   which takes an early-out and is genuinely unaffected.

   Measured sensitivity, by adding a constant offset to x_s in the float
   planar loop and counting failed checks:

     +0.05 px   0 failures   (passes -- below the guard's resolution)
     +0.15 px   1 failure
     +0.25 px  15 failures
     +0.50 px  16 failures

   So this catches a systematic geometry error somewhere between 0.15 and
   0.25 px. TB_MEAN_DRIFT = 1.0 is enormous next to the 0.0001 the optimiser
   actually moves these means, and could be tightened a long way; it is left
   loose deliberately, because the failure this test exists to prevent is a
   changed picture, and a guard that cries wolf on a compiler upgrade is one
   that gets deleted. */
#define TB_MEAN_DRIFT 1.0

/* Mean |fixed - float| per case, same [interpolation][transform] order as the
   CRC tables. Note packed transform 4: _FLT(transformPacked) has a
   translation-only fast path for |alpha| <= 0.1 degrees that ignores zoom
   entirely, so there the two loops legitimately render different pictures and
   the entry is 26.9 rather than the ~2 of its neighbours. It is pinned like
   any other value; it records current behaviour, not correct behaviour. */
static const double TB_MEAN_FF[4][TB_NUM_T] = {
  {0.0000, 0.0656, 0.0000, 0.0104, 0.0140, },
  {0.0000, 0.0338, 0.0055, 0.0126, 0.0181, },
  {0.0000, 3.3918, 3.1573, 1.0524, 0.0617, },
  {0.0000, 3.3794, 3.2570, 1.0974, 0.1572, },
};
/* Identical across interpolation types: transformPacked and its float twin
   carry their own sampling and ignore cfg.interpolType, which the packed CRC
   table above shows the same way. */
static const double TB_MEAN_FF_PACKED[4][TB_NUM_T] = {
  {0.0000, 0.4228, 2.1724, 0.6096, 26.9078, },
  {0.0000, 0.4228, 2.1724, 0.6096, 26.9078, },
  {0.0000, 0.4228, 2.1724, 0.6096, 26.9078, },
  {0.0000, 0.4228, 2.1724, 0.6096, 26.9078, },
};

void test_transform_baseline(void){
  VSFrameInfo fi, fiPacked;
  VSFrame src, srcPacked;
  uint32_t seed = 12345;
  int ip, i, mismatch = 0;
  vsFrameInfoInit(&fi, 320, 240, PF_YUV420P);
  vsFrameAllocate(&src, &fi);
  ldFillTexture(&src, &fi, &seed);
  memset(src.data[1], 0x60, (size_t)src.linesize[1]*(fi.height/2));
  memset(src.data[2], 0xA0, (size_t)src.linesize[2]*(fi.height/2));

  vsFrameInfoInit(&fiPacked, 320, 240, PF_RGB24);
  vsFrameAllocate(&srcPacked, &fiPacked);
  tbFillPackedTexture(&srcPacked, &fiPacked);

  for(ip=0; ip<4; ip++){
    for(i=0; i<TB_NUM_T; i++){
      VSFrame planarFixed, planarFloat, packedFixed, packedFloat;
      uint32_t gf, pf;
      TBDiff dPlanar, dPacked;

      vsFrameAllocate(&planarFixed, &fi);
      vsFrameAllocate(&planarFloat, &fi);
      vsFrameAllocate(&packedFixed, &fiPacked);
      vsFrameAllocate(&packedFloat, &fiPacked);

      gf = tbRunInto(&planarFixed, &src, &fi, (VSInterpolType)ip, 0, i);
                tbRunInto(&planarFloat, &src, &fi, (VSInterpolType)ip, 1, i);
      pf = tbRunInto(&packedFixed, &srcPacked, &fiPacked, (VSInterpolType)ip, 0, i);
                tbRunInto(&packedFloat, &srcPacked, &fiPacked, (VSInterpolType)ip, 1, i);

      dPlanar = tbDiff(&planarFixed, &planarFloat, &fi);
      dPacked = tbDiff(&packedFixed, &packedFloat, &fiPacked);

      if(gf != TB_GOLD_FIXED[ip][i] || pf != TB_GOLD_PACKED_FIXED[ip][i]
         || fabs(dPlanar.mean - TB_MEAN_FF[ip][i])        > TB_MEAN_DRIFT
         || fabs(dPacked.mean - TB_MEAN_FF_PACKED[ip][i]) > TB_MEAN_DRIFT) mismatch = 1;
      test_bool(gf == TB_GOLD_FIXED[ip][i]);
      test_bool(pf == TB_GOLD_PACKED_FIXED[ip][i]);
      /* Symmetric: the float loops moving CLOSER to the fixed ones is just as
         much a change in behaviour as moving away, and worth a look either
         way. */
      test_bool(fabs(dPlanar.mean - TB_MEAN_FF[ip][i])        <= TB_MEAN_DRIFT);
      test_bool(fabs(dPacked.mean - TB_MEAN_FF_PACKED[ip][i]) <= TB_MEAN_DRIFT);

      vsFrameFree(&planarFixed); vsFrameFree(&planarFloat);
      vsFrameFree(&packedFixed); vsFrameFree(&packedFloat);
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
    fprintf(stderr, "};\nstatic const uint32_t TB_GOLD_PACKED_FIXED[4][TB_NUM_T] = {\n");
    for(ip=0; ip<4; ip++){
      fprintf(stderr, "  {");
      for(i=0; i<TB_NUM_T; i++) fprintf(stderr, "0x%08Xu,", tbRun(&srcPacked, &fiPacked, (VSInterpolType)ip, 0, i));
      fprintf(stderr, "},\n");
    }
    fprintf(stderr, "};\nstatic const double TB_MEAN_FF[4][TB_NUM_T] = {\n");
    for(ip=0; ip<4; ip++){
      fprintf(stderr, "  {");
      for(i=0; i<TB_NUM_T; i++) fprintf(stderr, "%.4f, ", tbMeanFF(&src, &fi, (VSInterpolType)ip, i));
      fprintf(stderr, "},\n");
    }
    fprintf(stderr, "};\nstatic const double TB_MEAN_FF_PACKED[4][TB_NUM_T] = {\n");
    for(ip=0; ip<4; ip++){
      fprintf(stderr, "  {");
      for(i=0; i<TB_NUM_T; i++) fprintf(stderr, "%.4f, ", tbMeanFF(&srcPacked, &fiPacked, (VSInterpolType)ip, i));
      fprintf(stderr, "},\n");
    }
    fprintf(stderr, "};\n");
  }
  vsFrameFree(&srcPacked);
  vsFrameFree(&src);
}
