/* Chroma/luma geometric consistency under rotation.

   Included as a translation unit by tests.c, so it needs no includes of its
   own. Regression test for issue #79.

   A chroma sample spans (1<<wsub) luma columns and (1<<hsub) luma rows. When
   those two differ -- 4:2:2 (1,0), 4:4:0 (0,1), 4:1:1 (2,0) -- the plane's
   axes are not to the same scale, so a rotation applied directly in plane
   coordinates rotates in an anisotropic space and shears chroma against luma.
   It is correct by accident whenever wsub==hsub, which is why 4:2:0 and 4:4:4
   never showed the bug and the reporter's workaround (re-encoding to 4:2:0)
   appeared to fix it.

   The invariant asserted here is the one that was violated: paint the same
   shape into every plane, rotate, and the shape's centroid must land in the
   same place in frame coordinates whichever plane you measure it in. */

#define CG_W        160   /* multiples of 4 in both axes, so every format below */
#define CG_H        128   /*   has integral chroma dimensions */
#define CG_ALPHA    0.3   /* rad, ~17 deg: large enough for the shear to show */
#define CG_FG       255
#define CG_THRESH   200   /* excludes the background (0) and the crop fill (0x80) */
/* The shear this catches displaces the chroma centroid by 5-8 luma px for the
   asymmetric formats at this angle and offset; a correct transform keeps luma
   and chroma within well under a pixel of each other. */
#define CG_TOL      2.0

/* Luma-space coordinate of the centre of plane sample j along an axis
   subsampled by 2^sub. Sample j covers luma [j<<sub, ((j+1)<<sub)-1]. */
static double cgToLuma(int j, int sub){
  return (j + 0.5) * (double)(1 << sub) - 0.5;
}

/* Paints a filled disc, offset from the frame centre so that a rotation
   actually moves it, into every plane -- each at its own resolution but at the
   same position in frame coordinates. */
static void cgPaintDisc(VSFrame* f, const VSFrameInfo* fi){
  const double cx = CG_W * 0.30, cy = CG_H * 0.30;
  const double r  = (CG_W < CG_H ? CG_W : CG_H) * 0.18;
  int plane, x, y;
  for(plane=0; plane < fi->planes; plane++){
    int wsub = vsGetPlaneWidthSubS(fi, plane);
    int hsub = vsGetPlaneHeightSubS(fi, plane);
    int w = CHROMA_SIZE(fi->width, wsub), h = CHROMA_SIZE(fi->height, hsub);
    for(y=0; y<h; y++){
      for(x=0; x<w; x++){
        double lx = cgToLuma(x, wsub), ly = cgToLuma(y, hsub);
        double dx = lx - cx, dy = ly - cy;
        f->data[plane][x + y*f->linesize[plane]] =
          (dx*dx + dy*dy <= r*r) ? CG_FG : 0;
      }
    }
  }
}

/* Centroid of the painted disc in a plane, expressed in luma coordinates.
   Returns 0 if the plane holds no foreground at all (which is itself a
   failure worth reporting rather than silently averaging nothing). */
static int cgCentroid(const VSFrame* f, const VSFrameInfo* fi, int plane,
                      double* ox, double* oy){
  int wsub = vsGetPlaneWidthSubS(fi, plane);
  int hsub = vsGetPlaneHeightSubS(fi, plane);
  int w = CHROMA_SIZE(fi->width, wsub), h = CHROMA_SIZE(fi->height, hsub);
  double sx=0, sy=0; long n=0;
  int x, y;
  for(y=0; y<h; y++){
    for(x=0; x<w; x++){
      if(f->data[plane][x + y*f->linesize[plane]] > CG_THRESH){
        sx += cgToLuma(x, wsub); sy += cgToLuma(y, hsub); n++;
      }
    }
  }
  if(n == 0) return 0;
  *ox = sx/n; *oy = sy/n;
  return 1;
}

/* Rotates the painted frame and compares the luma centroid with each chroma
   centroid. useFloat picks the float reference implementation instead of the
   fixed point one the library ships, so both carry the same geometry. */
static void cgCheckFormat(VSPixelFormat pf, const char* name, int useFloat){
  VSFrameInfo fi;
  VSFrame src, dest;
  VSTransformConfig conf;
  VSTransformData td;
  VSTransform t = null_transform();
  double lx, ly;
  int plane;

  fprintf(stderr, "--- chroma geometry, %-8s %s ---\n",
          name, useFloat ? "(float)" : "(fixed point)");
  test_bool(vsFrameInfoInit(&fi, CG_W, CG_H, pf) != 0);
  vsFrameAllocate(&src,  &fi);
  vsFrameAllocate(&dest, &fi);
  cgPaintDisc(&src, &fi);

  conf = vsTransformGetDefaultConfig("test_chroma_geometry");
  conf.interpolType = VS_Zero;   /* nearest: keeps the disc edge crisp */
  conf.crop         = VSCropBorder;
  conf.relative     = 0;
  test_bool(vsTransformDataInit(&td, &conf, &fi, &fi) == VS_OK);

  t.alpha = CG_ALPHA;            /* pure rotation: no shift, no zoom */
  test_bool(vsTransformPrepare(&td, &src, &dest) == VS_OK);
  if(useFloat)
    test_bool(transformPlanar_float(&td, t) == VS_OK);
  else
    test_bool(vsDoTransform(&td, t) == VS_OK);
  test_bool(vsTransformFinish(&td) == VS_OK);

  test_bool(cgCentroid(&td.dest, &fi, 0, &lx, &ly));
  for(plane=1; plane < fi.planes; plane++){
    double cx, cy;
    if(!cgCentroid(&td.dest, &fi, plane, &cx, &cy)){
      fprintf(stderr, "  plane %i: no foreground found\n", plane);
      test_bool(0);
      continue;
    }
    {
      double dx = cx-lx, dy = cy-ly;
      int ok = fabs(dx) < CG_TOL && fabs(dy) < CG_TOL;
      fprintf(stderr, "  plane %i %s luma=(%.2f,%.2f) chroma=(%.2f,%.2f) "
              "delta=(%+.2f,%+.2f) px\n",
              plane, ok ? "ok  " : "FAIL", lx, ly, cx, cy, dx, dy);
      test_bool(ok);
    }
  }

  vsTransformDataCleanup(&td);
  vsFrameFree(&src);
  vsFrameFree(&dest);
}

void test_chroma_geometry(void){
  int impl;
  for(impl=0; impl<2; impl++){
    /* symmetric subsampling: these were always correct and must stay so */
    cgCheckFormat(PF_YUV444P, "yuv444p", impl);
    cgCheckFormat(PF_YUV420P, "yuv420p", impl);
    cgCheckFormat(PF_YUV410P, "yuv410p", impl);
    /* asymmetric: wsub != hsub, the cases issue #79 was about */
    cgCheckFormat(PF_YUV422P, "yuv422p", impl);
    cgCheckFormat(PF_YUV440P, "yuv440p", impl);
    cgCheckFormat(PF_YUV411P, "yuv411p", impl);
  }
}
