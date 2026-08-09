/* bench_transform.c
 *
 * Timing for the transform stage, in particular the packed (RGB24/BGR24/RGBA)
 * path, whose per-pixel interpolation runs through interpolateN().
 * Not part of the test suite -- a measurement tool.
 *
 * Usage: bench_transform [width height nframes]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "transform.h"
#include "transform_internal.h"
#include "transformfixedpoint.h"
#include "transformfloat.h"
#include "frameinfo.h"

static double now_s(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static unsigned int rstate = 12345;
static unsigned int xrand(void) {
  rstate = rstate * 1103515245u + 12345u;
  return (rstate >> 16) & 0x7FFF;
}

static void fill(VSFrame* f, const VSFrameInfo* fi) {
  int p, y, x;
  for (p = 0; p < fi->planes; p++) {
    int w = (p == 0) ? fi->width * fi->bytesPerPixel : fi->width / 2;
    int h = (p == 0) ? fi->height : fi->height / 2;
    for (y = 0; y < h; y++) {
      unsigned char* row = f->data[p] + y * f->linesize[p];
      for (x = 0; x < w; x++)
        row[x] = (unsigned char)xrand();
    }
  }
}

/* FNV-1a over every byte of every plane -- the point of running the same
   benchmark against a modified interpolator is that this must not change. */
static unsigned long long frame_hash(const VSFrame* f, const VSFrameInfo* fi) {
  unsigned long long h = 14695981039346656037ULL;
  int p, y, x;
  for (p = 0; p < fi->planes; p++) {
    int w = (p == 0) ? fi->width * fi->bytesPerPixel : fi->width / 2;
    int hgt = (p == 0) ? fi->height : fi->height / 2;
    for (y = 0; y < hgt; y++) {
      const unsigned char* row = f->data[p] + y * f->linesize[p];
      for (x = 0; x < w; x++) {
        h ^= row[x];
        h *= 1099511628211ULL;
      }
    }
  }
  return h;
}

static VSTransform mk_transform(void) {
  VSTransform t;
  memset(&t, 0, sizeof(t));
  /* deliberately not identity and not axis aligned, so every destination
     pixel takes the interpolating path */
  t.x     = 7.3;
  t.y     = -4.7;
  t.alpha = 1.5 * M_PI / 180.0;
  t.zoom  = 2.0;
  t.extra = 0;
  return t;
}

typedef int (*trfn)(VSTransformData*, VSTransform);

static void bench_t(const char* name, trfn fn, VSPixelFormat pf,
                    int width, int height, int nframes,
                    VSTransform t, int crop) {
  VSFrameInfo fi;
  VSFrame src, dest;
  VSTransformData td;
  VSTransformConfig conf = vsTransformGetDefaultConfig("bench");
  double t0, t1;
  int i;
  unsigned long long chk = 0;

  conf.crop = crop;
  vsFrameInfoInit(&fi, width, height, pf);
  vsFrameAllocate(&src, &fi);
  vsFrameAllocate(&dest, &fi);
  fill(&src, &fi);

  if (vsTransformDataInit(&td, &conf, &fi, &fi) != VS_OK) {
    fprintf(stderr, "vsTransformDataInit failed\n");
    exit(1);
  }

  /* one warm up pass, not timed */
  vsTransformPrepare(&td, &src, &dest);
  fn(&td, t);
  vsTransformFinish(&td);

  t0 = now_s();
  for (i = 0; i < nframes; i++) {
    vsTransformPrepare(&td, &src, &dest);
    fn(&td, t);
    vsTransformFinish(&td);
  }
  t1 = now_s();

  chk = frame_hash(&dest, &fi);
  printf("%-26s %4dx%-4d  %8.3f ms/frame   [%016llx]\n", name, width, height,
         (t1 - t0) * 1000.0 / nframes, chk);

  vsTransformDataCleanup(&td);
  vsFrameFree(&src);
  vsFrameFree(&dest);
}

/* A set of transforms chosen to hit every branch of the interpolator:
   interior pixels, the last row/column (where the "ceil" neighbour is one
   past the end), and source coordinates far outside the frame. */
static void verify(void) {
  VSPixelFormat fmts[3] = {PF_RGB24, PF_BGR24, PF_RGBA};
  const char* names[3]  = {"RGB24", "BGR24", "RGBA "};
  int f, c, i;
  for (f = 0; f < 3; f++) {
    for (c = 0; c <= 1; c++) {
      for (i = 0; i < 5; i++) {
        VSTransform t;
        char label[64];
        memset(&t, 0, sizeof(t));
        switch (i) {
          case 0: t.x = 7.3;  t.y = -4.7; t.alpha = 1.5*M_PI/180.0; t.zoom = 2.0; break;
          case 1: t.x = 0.5;  t.y =  0.5; break;               /* half pixel, all borders */
          case 2: t.x = -400; t.y = -300; break;               /* mostly outside */
          case 3: t.alpha = 45*M_PI/180.0; break;              /* corners outside */
          case 4: t.zoom = -60; break;                         /* zoom out, big borders */
        }
        snprintf(label, sizeof(label), "verify %s crop=%d case=%d", names[f], c, i);
        bench_t(label, transformPacked, fmts[f], 61, 37, 2, t, c);
        snprintf(label, sizeof(label), "verify %s crop=%d case=%d flt", names[f], c, i);
        bench_t(label, transformPacked_float, fmts[f], 61, 37, 2, t, c);
      }
    }
  }
}

int main(int argc, char** argv) {
  int width = 1920, height = 1080, nframes = 20;
  VSTransform t = mk_transform();

  if (argc >= 2 && strcmp(argv[1], "verify") == 0) {
    verify();
    return 0;
  }
  if (argc >= 3) {
    width  = atoi(argv[1]);
    height = atoi(argv[2]);
  }
  if (argc >= 4)
    nframes = atoi(argv[3]);

  bench_t("packed RGB24 fixedpoint", transformPacked, PF_RGB24,
          width, height, nframes, t, 0);
  bench_t("packed RGBA  fixedpoint", transformPacked, PF_RGBA,
          width, height, nframes, t, 0);
  bench_t("packed RGB24 float", transformPacked_float, PF_RGB24,
          width, height, nframes, t, 0);
  bench_t("planar YUV420 fixedpoint", transformPlanar, PF_YUV420P,
          width, height, nframes, t, 0);
  return 0;
}
