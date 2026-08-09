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
static const uint32_t TB_GOLD_FLOAT[4][TB_NUM_T] = {
  {0xEE415B68u,0x2745FC6Bu,0x3652C8D0u,0x6ACDCC5Au,0x552B3A53u,},
  {0xEE415B68u,0x9CF29AC6u,0xD9E501C7u,0x55D282A1u,0x7920DDE0u,},
  {0xEE415B68u,0x9CF29AC6u,0xCB1C1F42u,0xA8F3FE88u,0x77911B01u,},
  {0xEE415B68u,0x9CF29AC6u,0x318756E1u,0x1219068Fu,0x1EDBA420u,},
};

/* Same shape, but for a PF_RGB24 (packed) source, so transformPacked and
   _FLT(transformPacked) are pinned too.  Note: _FLT(transformPacked) has a
   pre-existing translation-only fast path for |alpha| <= 0.1 degrees that
   ignores zoom entirely; transform 4 (small alpha + zoom) goes down that
   path in the float case, and its golden below encodes that quirk. That is
   intentional -- this guard pins current behaviour, not "correct" behaviour. */
static const uint32_t TB_GOLD_PACKED_FIXED[4][TB_NUM_T] = {
  {0x95C3E009u,0x5510ACC8u,0xE6DDC561u,0x14936393u,0x4FBCDF2Au,},
  {0x95C3E009u,0x5510ACC8u,0xE6DDC561u,0x14936393u,0x4FBCDF2Au,},
  {0x95C3E009u,0x5510ACC8u,0xE6DDC561u,0x14936393u,0x4FBCDF2Au,},
  {0x95C3E009u,0x5510ACC8u,0xE6DDC561u,0x14936393u,0x4FBCDF2Au,},
};
static const uint32_t TB_GOLD_PACKED_FLOAT[4][TB_NUM_T] = {
  {0x95C3E009u,0xE6C77673u,0x38B15C66u,0xA187C187u,0xB46D462Fu,},
  {0x95C3E009u,0xE6C77673u,0x38B15C66u,0xA187C187u,0xB46D462Fu,},
  {0x95C3E009u,0xE6C77673u,0x38B15C66u,0xA187C187u,0xB46D462Fu,},
  {0x95C3E009u,0xE6C77673u,0x38B15C66u,0xA187C187u,0xB46D462Fu,},
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
      uint32_t gf = tbRun(&src, &fi, (VSInterpolType)ip, 0, i);
      uint32_t gl = tbRun(&src, &fi, (VSInterpolType)ip, 1, i);
      uint32_t pf = tbRun(&srcPacked, &fiPacked, (VSInterpolType)ip, 0, i);
      uint32_t pl = tbRun(&srcPacked, &fiPacked, (VSInterpolType)ip, 1, i);
      if(gf != TB_GOLD_FIXED[ip][i] || gl != TB_GOLD_FLOAT[ip][i]
         || pf != TB_GOLD_PACKED_FIXED[ip][i] || pl != TB_GOLD_PACKED_FLOAT[ip][i]) mismatch = 1;
      test_bool(gf == TB_GOLD_FIXED[ip][i]);
      test_bool(gl == TB_GOLD_FLOAT[ip][i]);
      test_bool(pf == TB_GOLD_PACKED_FIXED[ip][i]);
      test_bool(pl == TB_GOLD_PACKED_FLOAT[ip][i]);
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
    fprintf(stderr, "};\nstatic const uint32_t TB_GOLD_PACKED_FIXED[4][TB_NUM_T] = {\n");
    for(ip=0; ip<4; ip++){
      fprintf(stderr, "  {");
      for(i=0; i<TB_NUM_T; i++) fprintf(stderr, "0x%08Xu,", tbRun(&srcPacked, &fiPacked, (VSInterpolType)ip, 0, i));
      fprintf(stderr, "},\n");
    }
    fprintf(stderr, "};\nstatic const uint32_t TB_GOLD_PACKED_FLOAT[4][TB_NUM_T] = {\n");
    for(ip=0; ip<4; ip++){
      fprintf(stderr, "  {");
      for(i=0; i<TB_NUM_T; i++) fprintf(stderr, "0x%08Xu,", tbRun(&srcPacked, &fiPacked, (VSInterpolType)ip, 1, i));
      fprintf(stderr, "},\n");
    }
    fprintf(stderr, "};\n");
  }
  vsFrameFree(&srcPacked);
  vsFrameFree(&src);
}
