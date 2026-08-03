/* ---------------------------------------------------------------------------
   Unit tests for the packed pixel formats PF_RGB24 / PF_BGR24 / PF_RGBA.

   These formats are advertised by FFmpeg's vidstab filters
   (ff_vidstab_pix_fmts contains AV_PIX_FMT_RGB24, AV_PIX_FMT_BGR24 and
   AV_PIX_FMT_RGBA) so they really do reach this library.

   The convention that is verified here (see vidstabdefines.h):
     VSFrame::linesize is *always* a byte offset between two rows. Only the
     column index is scaled by bytesPerPixel, never the row offset.
   --------------------------------------------------------------------------- */

static uint8_t* packedPixel(const VSFrame* f, const VSFrameInfo* fi, int x, int y){
  return f->data[0] + y * f->linesize[0] + x * fi->bytesPerPixel;
}

/** blocky test image, every channel carries a clearly different pattern so that
    a stride/channel mixup shows up as a large difference and not as noise */
static void fillPackedBlocks(VSFrame* f, const VSFrameInfo* fi){
  int x,y,c;
  for(y=0; y<fi->height; y++){
    for(x=0; x<fi->width; x++){
      uint8_t* p = packedPixel(f,fi,x,y);
      int bx = x/16, by = y/16;
      p[0] = (uint8_t)( 30 + 37*((bx*5+by*3)%6));
      p[1] = (uint8_t)(200 - 23*((bx*2+by*7)%7));
      p[2] = (uint8_t)( 80 + 11*((bx+by)%13));
      for(c=3; c<fi->bytesPerPixel; c++)
        p[c] = (uint8_t)(255 - 4*((bx+by)%9));
    }
  }
}

/** randomly coloured tiles: high contrast and a unique fingerprint per field,
    but with a correlation length of BLOCK pixels so that the coarse grid search
    of the motion detection actually has a gradient to follow */
#define PACKED_BLOCK 8
static void fillPackedNoise(VSFrame* f, const VSFrameInfo* fi, unsigned int seed){
  int x,y,c;
  int bw = fi->width/PACKED_BLOCK + 1;
  int bh = fi->height/PACKED_BLOCK + 1;
  uint8_t* blocks = vs_zalloc(bw*bh*4);
  srand(seed);
  for(y=0; y<bw*bh*4; y++) blocks[y] = (uint8_t)(rand()%256);
  for(y=0; y<fi->height; y++){
    for(x=0; x<fi->width; x++){
      uint8_t* p = packedPixel(f,fi,x,y);
      uint8_t* b = blocks + ((x/PACKED_BLOCK) + (y/PACKED_BLOCK)*bw)*4;
      for(c=0; c<fi->bytesPerPixel; c++)
        p[c] = b[c];
    }
  }
  vs_free(blocks);
}

/** copies src shifted by (dx,dy): dest(x,y) = src(x-dx,y-dy), black outside */
static void shiftPacked(VSFrame* dest, const VSFrame* src, const VSFrameInfo* fi,
                        int dx, int dy){
  int x,y,c;
  for(y=0; y<fi->height; y++){
    for(x=0; x<fi->width; x++){
      uint8_t* d = packedPixel(dest,fi,x,y);
      int sx = x-dx, sy = y-dy;
      if(sx<0 || sy<0 || sx>=fi->width || sy>=fi->height){
        for(c=0; c<fi->bytesPerPixel; c++) d[c]=0;
      }else{
        const uint8_t* s = packedPixel(src,fi,sx,sy);
        for(c=0; c<fi->bytesPerPixel; c++) d[c]=s[c];
      }
    }
  }
}

/** counts channels that differ by more than tol, reports the first few */
static int packedCountDiff(const VSFrame* a, const VSFrame* b,
                           const VSFrameInfo* fi, int tol, const char* what){
  int x,y,c,bad=0;
  for(y=0; y<fi->height; y++){
    for(x=0; x<fi->width; x++){
      const uint8_t* pa = packedPixel(a,fi,x,y);
      const uint8_t* pb = packedPixel(b,fi,x,y);
      for(c=0; c<fi->bytesPerPixel; c++){
        int diff = (int)pa[c] - (int)pb[c];
        if(abs(diff)>tol){
          if(bad<8)
            fprintf(stderr,"  %s: mismatch at (%i,%i) channel %i: %i != %i\n",
                    what, x, y, c, (int)pa[c], (int)pb[c]);
          bad++;
        }
      }
    }
  }
  fprintf(stderr,"  %s: %i deviating channel values (tolerance %i)\n", what, bad, tol);
  return bad;
}

static const char* packedFormatName(VSPixelFormat pf){
  switch(pf){
   case PF_RGB24: return "PF_RGB24";
   case PF_BGR24: return "PF_BGR24";
   case PF_RGBA:  return "PF_RGBA";
   default:       return "?";
  }
}

/* --- frame info / allocation / copy ------------------------------------- */

static void test_packed_frameinfo(void){
  VSPixelFormat fmts[3] = {PF_RGB24, PF_BGR24, PF_RGBA};
  int bpps[3]           = {3,3,4};
  int i,k;
  fprintf(stderr,"--- packed VSFrameInfo / allocation ---\n");
  for(i=0; i<3; i++){
    VSFrameInfo fi;
    VSFrame f, g;
    int bytes;
    long sum = 0;
    test_bool(vsFrameInfoInit(&fi, 64, 32, fmts[i]) == 1);
    fprintf(stderr,"%s: planes=%i bytesPerPixel=%i\n",
            packedFormatName(fmts[i]), fi.planes, fi.bytesPerPixel);
    /* everything is in plane 0 for packed data, so there is exactly one plane.
       With planes==0 nothing is allocated/copied at all. */
    test_bool(fi.planes == 1);
    test_bool(fi.bytesPerPixel == bpps[i]);

    vsFrameAllocate(&f, &fi);
    test_bool(!vsFrameIsNull(&f));
    test_bool(f.linesize[0] == 64*bpps[i]);
    bytes = f.linesize[0]*fi.height;
    /* touch every byte: catches an undersized allocation under ASan */
    for(k=0; k<bytes; k++) sum += f.data[0][k];
    test_bool(sum == 0); /* vs_zalloc */

    fillPackedBlocks(&f,&fi);
    vsFrameAllocate(&g, &fi);
    test_bool(!vsFrameIsNull(&g));
    vsFrameCopy(&g, &f, &fi);
    test_bool(memcmp(g.data[0], f.data[0], bytes) == 0);
    vsFrameFree(&f);
    vsFrameFree(&g);

    { /* vsFrameFillFromBuffer must describe one packed plane */
      uint8_t* buf = vs_zalloc(bytes);
      VSFrame h;
      vsFrameFillFromBuffer(&h, buf, &fi);
      test_bool(h.data[0] == buf);
      test_bool(h.linesize[0] == 64*bpps[i]);
      test_bool(h.data[1] == 0);
      vs_free(buf);
    }
  }
}

/* --- identity transform must reproduce the input bit for bit ------------- */

static void test_packed_identity(VSPixelFormat pf){
  VSFrameInfo fi;
  VSFrame src, dest;
  VSTransformConfig conf = vsTransformGetDefaultConfig("test_packed_identity");
  VSTransformData td;
  fprintf(stderr,"--- identity transform, %s ---\n", packedFormatName(pf));
  test_bool(vsFrameInfoInit(&fi, 128, 64, pf) == 1);
  vsFrameAllocate(&src,&fi);
  vsFrameAllocate(&dest,&fi);
  test_bool(!vsFrameIsNull(&src) && !vsFrameIsNull(&dest));
  fillPackedBlocks(&src,&fi);
  vsFrameCopy(&dest,&src,&fi);

  test_bool(vsTransformDataInit(&td, &conf, &fi, &fi) == VS_OK);
  test_bool(vsTransformPrepare(&td,&dest,&dest) == VS_OK);
  test_bool(vsDoTransform(&td, null_transform()) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);
  test_bool(packedCountDiff(&dest,&src,&fi,0,"identity") == 0);

  vsTransformDataCleanup(&td);
  vsFrameFree(&src);
  vsFrameFree(&dest);
}

/* --- pure integer translation, no channel smearing ---------------------- */

static void test_packed_translate(VSPixelFormat pf){
  VSFrameInfo fi;
  VSFrame src, dest, ref, fltres;
  VSTransformConfig conf = vsTransformGetDefaultConfig("test_packed_translate");
  VSTransformData td;
  VSTransform t = null_transform();
  const int dx = 8, dy = -4;
  int x,y,c,bad=0;

  fprintf(stderr,"--- integer translation (%i,%i), %s ---\n", dx, dy,
          packedFormatName(pf));
  test_bool(vsFrameInfoInit(&fi, 128, 64, pf) == 1);
  vsFrameAllocate(&src,&fi);
  vsFrameAllocate(&dest,&fi);
  vsFrameAllocate(&ref,&fi);
  vsFrameAllocate(&fltres,&fi);
  fillPackedBlocks(&src,&fi);
  t.x = dx;
  t.y = dy;

  /* the float implementation takes the exact integer copy path for alpha==0,
     so it is an exact oracle: dest(x,y) = src(x-dx,y-dy), border keeps src */
  vsFrameCopy(&dest,&src,&fi);
  test_bool(vsTransformDataInit(&td, &conf, &fi, &fi) == VS_OK);
  test_bool(vsTransformPrepare(&td,&dest,&dest) == VS_OK);
  test_bool(transformPacked_float(&td, t) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);
  vsFrameCopy(&fltres,&dest,&fi);
  vsTransformDataCleanup(&td);

  /* build the expected image */
  for(y=0; y<fi.height; y++){
    for(x=0; x<fi.width; x++){
      uint8_t* d = packedPixel(&ref,&fi,x,y);
      int sx = x-dx, sy = y-dy;
      const uint8_t* s = (sx<0||sy<0||sx>=fi.width||sy>=fi.height)
        ? packedPixel(&src,&fi,x,y)   /* VSKeepBorder: previous frame content */
        : packedPixel(&src,&fi,sx,sy);
      for(c=0; c<fi.bytesPerPixel; c++) d[c]=s[c];
    }
  }
  test_bool(packedCountDiff(&fltres,&ref,&fi,0,"translate/float") == 0);

  /* the fixed point implementation interpolates, allow the usual +-2 */
  vsFrameCopy(&dest,&src,&fi);
  test_bool(vsTransformDataInit(&td, &conf, &fi, &fi) == VS_OK);
  test_bool(vsTransformPrepare(&td,&dest,&dest) == VS_OK);
  test_bool(transformPacked(&td, t) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);
  test_bool(packedCountDiff(&dest,&ref,&fi,2,"translate/fixedpoint") == 0);
  vsTransformDataCleanup(&td);

  /* channels must not leak into each other: a stride bug mixes them up, which
     always produces differences far bigger than the interpolation rounding */
  for(y=0; y<fi.height; y++){
    for(x=0; x<fi.width; x++){
      const uint8_t* pd = packedPixel(&dest,&fi,x,y);
      const uint8_t* pr = packedPixel(&ref,&fi,x,y);
      for(c=0; c<fi.bytesPerPixel; c++)
        if(abs((int)pd[c]-(int)pr[c]) > 16) bad++;
    }
  }
  fprintf(stderr,"  gross (channel swap) errors: %i\n", bad);
  test_bool(bad == 0);

  vsFrameFree(&src);
  vsFrameFree(&dest);
  vsFrameFree(&ref);
  vsFrameFree(&fltres);
}

/* --- motion detection on packed frames ---------------------------------- */

static void test_packed_motiondetect(VSPixelFormat pf){
  VSFrameInfo fi;
  VSFrame f1, f2;
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig("test_packed_md");
  VSMotionDetect md;
  LocalMotions lms;
  const int dx = 8, dy = -6;
  VSTransform t;

  fprintf(stderr,"--- motion detection, %s, shift (%i,%i) ---\n",
          packedFormatName(pf), dx, dy);
  test_bool(vsFrameInfoInit(&fi, 320, 240, pf) == 1);
  vsFrameAllocate(&f1,&fi);
  vsFrameAllocate(&f2,&fi);
  test_bool(!vsFrameIsNull(&f1) && !vsFrameIsNull(&f2));
  fillPackedNoise(&f1,&fi,42);
  shiftPacked(&f2,&f1,&fi,dx,dy);

  test_bool(vsMotionDetectInit(&md, &mdconf, &fi) == VS_OK);
  md.conf.numThreads = 1;
  test_bool(vsMotionDetection(&md, &lms, &f1) == VS_OK);
  vs_vector_del(&lms);
  test_bool(vsMotionDetection(&md, &lms, &f2) == VS_OK);
  fprintf(stderr,"  found %i local motions\n", vs_vector_size(&lms));
  test_bool(vs_vector_size(&lms) > 4);
  t = vsSimpleMotionsToTransform(fi, "test_packed_md", &lms);
  fprintf(stderr,"  recovered transform: ");
  storeVSTransform(stderr,&t);
  /* the content moved by (dx,dy), so the compensating shift is (-dx,-dy) */
  test_bool(fabs(t.x + dx) < 2);
  test_bool(fabs(t.y + dy) < 2);
  test_bool(fabs(t.alpha) < 0.01);
  vs_vector_del(&lms);
  vsMotionDetectionCleanup(&md);
  vsFrameFree(&f1);
  vsFrameFree(&f2);
}

/* --- the 'show' overlay on packed frames --------------------------------- */

/* With show enabled the detector draws the fields and motion vectors into the
   frame. This used to be skipped entirely for packed formats. */
static void test_packed_show(VSPixelFormat pf){
  VSFrameInfo fi;
  VSFrame f1, f2, before;
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig("test_packed_show");
  VSMotionDetect md;
  LocalMotions lms;
  const int dx = 8, dy = -6;
  int x, y, c, changed = 0, grey = 1, alphaOk = 1;

  fprintf(stderr,"--- show overlay, %s ---\n", packedFormatName(pf));
  test_bool(vsFrameInfoInit(&fi, 320, 240, pf) == 1);
  vsFrameAllocate(&f1,&fi);
  vsFrameAllocate(&f2,&fi);
  vsFrameAllocate(&before,&fi);
  fillPackedNoise(&f1,&fi,7);
  shiftPacked(&f2,&f1,&fi,dx,dy);
  vsFrameCopy(&before,&f2,&fi);

  mdconf.show = 2;   /* 2 also draws the field scan areas */
  test_bool(vsMotionDetectInit(&md, &mdconf, &fi) == VS_OK);
  md.conf.numThreads = 1;
  test_bool(vsMotionDetection(&md, &lms, &f1) == VS_OK);
  vs_vector_del(&lms);
  test_bool(vsMotionDetection(&md, &lms, &f2) == VS_OK);
  vs_vector_del(&lms);

  for(y=0; y<fi.height; y++){
    for(x=0; x<fi.width; x++){
      const uint8_t* p = packedPixel(&f2,&fi,x,y);
      const uint8_t* q = packedPixel(&before,&fi,x,y);
      int diff = 0;
      for(c=0; c<3; c++) if(p[c]!=q[c]) diff = 1;
      if(diff){
        changed++;
        /* every overlay colour is a grey level */
        if(!(p[0]==p[1] && p[1]==p[2])) grey = 0;
      }
      if(fi.bytesPerPixel==4 && p[3]!=q[3]) alphaOk = 0;
    }
  }
  fprintf(stderr,"  overlay pixels drawn: %i (grey=%i alphaUntouched=%i)\n",
          changed, grey, alphaOk);
  test_bool(changed > 100);   /* something was actually drawn */
  test_bool(grey);
  test_bool(alphaOk);

  vsMotionDetectionCleanup(&md);
  vsFrameFree(&f1);
  vsFrameFree(&f2);
  vsFrameFree(&before);
}

void test_packed(void){
  test_packed_frameinfo();
  test_packed_identity(PF_RGB24);
  test_packed_identity(PF_RGBA);
  test_packed_translate(PF_RGB24);
  test_packed_translate(PF_BGR24);
  test_packed_translate(PF_RGBA);
  test_packed_motiondetect(PF_RGB24);
  test_packed_motiondetect(PF_RGBA);
  test_packed_show(PF_RGB24);
  test_packed_show(PF_BGR24);
  test_packed_show(PF_RGBA);
}

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 *   c-basic-offset: 2 t
 * End:
 *
 * vim: expandtab shiftwidth=2:
 */
