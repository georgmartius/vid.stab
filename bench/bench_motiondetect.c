/* bench_motiondetect.c
 *
 * End-to-end and kernel-level benchmark for the motion detection stage.
 * Not part of the test suite -- a measurement tool for the SIMD work.
 *
 * Usage: bench_motiondetect [width height nframes]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <limits.h>

#include "motiondetect.h"
#include "motiondetect_internal.h"
#include "motiondetect_opt.h"
#include "cpudetect.h"
#include "frameinfo.h"
#include "vsvector.h"

static double now_s(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* deterministic pseudo random, so runs are comparable */
static unsigned int rstate = 12345;
static unsigned int xrand(void) {
  rstate = rstate * 1103515245u + 12345u;
  return (rstate >> 16) & 0x7FFF;
}

/* A textured frame that actually has contrast everywhere, shifted by (dx,dy)
   so the motion search has something to find. */
static void fill_frame(VSFrame* f, const VSFrameInfo* fi, int dx, int dy) {
  int x, y;
  for (y = 0; y < fi->height; y++) {
    unsigned char* row = f->data[0] + y * f->linesize[0];
    for (x = 0; x < fi->width; x++) {
      int u = x + dx, v = y + dy;
      /* a mix of smooth gradients and high frequency detail */
      row[x] = (unsigned char)((u * 3 + v * 5 + ((u >> 3) ^ (v >> 3)) * 17
                                + ((u * v) >> 6)) & 0xFF);
    }
  }
  if (fi->pFormat == PF_YUV420P) {
    int p;
    for (p = 1; p < 3; p++) {
      for (y = 0; y < fi->height / 2; y++) {
        unsigned char* row = f->data[p] + y * f->linesize[p];
        memset(row, 128, fi->width / 2);
      }
    }
  }
}

static void bench_end_to_end(int width, int height, int nframes) {
  VSMotionDetect md;
  VSFrameInfo fi;
  VSMotionDetectConfig conf = vsMotionDetectGetDefaultConfig("bench");
  VSFrame frame;
  int i;
  double t0, t1;

  conf.show = 0;
  vsFrameInfoInit(&fi, width, height, PF_YUV420P);
  if (vsMotionDetectInit(&md, &conf, &fi) != VS_OK) {
    fprintf(stderr, "vsMotionDetectInit failed\n");
    exit(1);
  }
  vsFrameAllocate(&frame, &fi);

  /* warm up: one frame so the "prev" buffer is populated */
  fill_frame(&frame, &fi, 0, 0);
  {
    LocalMotions lms;
    vsMotionDetection(&md, &lms, &frame);
    vs_vector_del(&lms);
  }

  t0 = now_s();
  for (i = 0; i < nframes; i++) {
    LocalMotions lms;
    fill_frame(&frame, &fi, (i % 7) - 3, (i % 5) - 2);
    vsMotionDetection(&md, &lms, &frame);
    vs_vector_del(&lms);
  }
  t1 = now_s();

  printf("end-to-end  %4dx%-4d  %3d frames: %8.2f ms/frame  (%6.1f fps)\n",
         width, height, nframes, (t1 - t0) * 1000.0 / nframes,
         nframes / (t1 - t0));

  vsFrameFree(&frame);
  vsMotionDetectionCleanup(&md);
}

/* ---------------- kernel level ---------------- */

typedef unsigned int (*cmpfn)(unsigned char* const, unsigned char* const,
                              const Field*, int, int, int, int, int, int,
                              unsigned int);
typedef double (*confn)(unsigned char* const, const Field*, int, int);

static double contrast_C_wrap(unsigned char* const I, const Field* f,
                              int linesize, int height) {
  return contrastSubImg(I, f, linesize, height, 1);
}

#define BENCH_W 1920
#define BENCH_H 1080

static unsigned char *img1, *img2;

static void alloc_images(void) {
  int i;
  img1 = malloc((size_t)BENCH_W * BENCH_H);
  img2 = malloc((size_t)BENCH_W * BENCH_H);
  for (i = 0; i < BENCH_W * BENCH_H; i++) {
    img1[i] = (unsigned char)xrand();
    img2[i] = (unsigned char)(img1[i] + (xrand() & 7) - 3);
  }
}

static void bench_compare(const char* name, cmpfn fn, int fieldsize, int reps) {
  Field field;
  double t0, t1;
  int r, dx, dy;
  unsigned long long acc = 0;

  field.size = fieldsize;
  field.x = BENCH_W / 2;
  field.y = BENCH_H / 2;

  t0 = now_s();
  for (r = 0; r < reps; r++) {
    for (dy = -4; dy <= 4; dy++)
      for (dx = -4; dx <= 4; dx++)
        acc += fn(img1, img2, &field, BENCH_W, BENCH_W, BENCH_H, 1, dx, dy,
                  UINT_MAX);
  }
  t1 = now_s();
  {
    double calls = (double)reps * 81.0;
    double bytes = calls * (double)fieldsize * fieldsize * 2.0;
    printf("  compare  %-10s size=%-3d  %8.1f ns/call  %7.2f GB/s   [%llu]\n",
           name, fieldsize, (t1 - t0) * 1e9 / calls, bytes / (t1 - t0) / 1e9,
           acc);
  }
}

static void bench_contrast(const char* name, confn fn, int fieldsize, int reps) {
  Field field;
  double t0, t1;
  int r;
  double acc = 0;

  field.size = fieldsize;
  field.x = BENCH_W / 2;
  field.y = BENCH_H / 2;

  t0 = now_s();
  for (r = 0; r < reps; r++)
    acc += fn(img1, &field, BENCH_W, BENCH_H);
  t1 = now_s();
  printf("  contrast %-10s size=%-3d  %8.1f ns/call  %7.2f GB/s   [%.3f]\n",
         name, fieldsize, (t1 - t0) * 1e9 / reps,
         (double)reps * fieldsize * fieldsize / (t1 - t0) / 1e9, acc);
}

int main(int argc, char** argv) {
  int width = 1920, height = 1080, nframes = 30;
  int sizes[] = {32, 48, 112};
  int i;
  unsigned int cpu;

  if (argc >= 3) {
    width = atoi(argv[1]);
    height = atoi(argv[2]);
  }
  if (argc >= 4)
    nframes = atoi(argv[3]);

  alloc_images();

  /* Only time kernels this CPU can actually execute -- a build machine's
     compiler routinely supports more than its processor does. */
  cpu = vs_cpu_flags();
  printf("=== kernels (this CPU: %s) ===\n", vs_cpu_flags_name(cpu));
  for (i = 0; i < (int)(sizeof(sizes) / sizeof(sizes[0])); i++) {
    int s = sizes[i];
    int reps = 20000000 / (s * s);
    bench_compare("C", compareSubImg_thr, s, reps / 81 + 1);
#ifdef VS_HAVE_SSE2
    if (cpu & VS_CPU_SSE2)
      bench_compare("SSE2", compareSubImg_thr_sse2, s, reps / 81 + 1);
#endif
#ifdef VS_HAVE_AVX2
    if (cpu & VS_CPU_AVX2)
      bench_compare("AVX2", compareSubImg_thr_avx2, s, reps / 81 + 1);
#endif
#ifdef VS_HAVE_AVX512
    if (cpu & VS_CPU_AVX512)
      bench_compare("AVX512", compareSubImg_thr_avx512, s, reps / 81 + 1);
#endif
#ifdef VS_HAVE_NEON
    bench_compare("NEON", compareSubImg_thr_neon, s, reps / 81 + 1);
#endif
    bench_contrast("C", contrast_C_wrap, s, reps);
#ifdef VS_HAVE_SSE2
    if (cpu & VS_CPU_SSE2)
      bench_contrast("SSE2", contrastSubImg1_SSE, s, reps);
#endif
#ifdef VS_HAVE_AVX2
    if (cpu & VS_CPU_AVX2)
      bench_contrast("AVX2", contrastSubImg1_avx2, s, reps);
#endif
#ifdef VS_HAVE_AVX512
    if (cpu & VS_CPU_AVX512)
      bench_contrast("AVX512", contrastSubImg1_avx512, s, reps);
#endif
#ifdef VS_HAVE_NEON
    bench_contrast("NEON", contrastSubImg1_neon, s, reps);
#endif
    printf("\n");
  }

  printf("=== end to end ===\n");
  bench_end_to_end(width, height, nframes);
  return 0;
}
