/*
 * test_lensdistortion.c
 *
 *  Tests for recovering the barrel distortion parameter from local motions.
 *  See docs/lens-distortion.md for the math.
 */
#include "lensdistortion.h"

/* --- cycle 1: the model is an exact inverse pair ------------------------- */

/* Radii to probe, as a fraction of rho (=1 at the image corner).  0.999 rather
   than 1.0 so the corner itself is inside the open domain of U_k. */
static const double LD_TEST_RADII[] = {0.0, 0.05, 0.2, 0.5, 0.75, 0.999};
#define LD_NUM_RADII ((int)(sizeof(LD_TEST_RADII)/sizeof(LD_TEST_RADII[0])))

static const double LD_TEST_KS[] = {-0.3, -0.1, 0.0, 0.1};
#define LD_NUM_KS ((int)(sizeof(LD_TEST_KS)/sizeof(LD_TEST_KS[0])))

static void test_lensdistortion_roundtrip(void){
  VSFrameInfo fi;
  int ki, ri, ai;
  /* a few angles so we check the map is radial in every direction, not just
     along an axis where a sign error would cancel */
  const double angles[] = {0.0, 0.7, 2.4, 4.1, 5.9};
  const int numAngles = (int)(sizeof(angles)/sizeof(angles[0]));

  test_bool(vsFrameInfoInit(&fi, 1280, 720, PF_GRAY8) != 0);

  fprintf(stderr, "--- lens distortion model round trip ---\n");
  for(ki=0; ki<LD_NUM_KS; ki++){
    VSLensDistortion ld = vsLensDistortionInit(&fi, LD_TEST_KS[ki]);
    double worst = 0.0;
    for(ri=0; ri<LD_NUM_RADII; ri++){
      for(ai=0; ai<numAngles; ai++){
        /* build a point at the requested normalised radius */
        double r  = LD_TEST_RADII[ri] * ld.rho;
        double px = ld.cx + r*cos(angles[ai]);
        double py = ld.cy + r*sin(angles[ai]);
        double ux, uy, dx, dy, err;

        test_bool(vsLensUndistortPoint(&ld, px, py, &ux, &uy) == VS_OK);
        test_bool(vsLensDistortPoint(&ld, ux, uy, &dx, &dy) == VS_OK);

        err = sqrt(sqr(dx-px) + sqr(dy-py));
        if(err > worst) worst = err;
      }
    }
    fprintf(stderr, "k=%6.3f: worst round-trip error %.3e px\n",
            LD_TEST_KS[ki], worst);
    test_bool(worst < 1e-9);
  }

  /* k=0 must be exactly the identity, not merely close: the rationalised
     form exists precisely so this needs no special case */
  {
    VSLensDistortion ld = vsLensDistortionInit(&fi, 0.0);
    double ux, uy, dx, dy;
    test_bool(vsLensUndistortPoint(&ld, 100.0, 640.0, &ux, &uy) == VS_OK);
    test_bool(ux == 100.0 && uy == 640.0);
    test_bool(vsLensDistortPoint(&ld, 100.0, 640.0, &dx, &dy) == VS_OK);
    test_bool(dx == 100.0 && dy == 640.0);
  }
}

static void test_lensdistortion_domain(void){
  VSFrameInfo fi;
  double ux, uy;

  test_bool(vsFrameInfoInit(&fi, 1280, 720, PF_GRAY8) != 0);

  fprintf(stderr, "--- lens distortion domain guards ---\n");

  /* Strong pincushion: U_k needs 1 + k*r^2 > 0, so a large enough radius
     leaves the domain.  It must report an error rather than divide by
     something near zero and hand back a NaN. */
  {
    VSLensDistortion ld = vsLensDistortionInit(&fi, -1.5);
    double r = 0.95*ld.rho;
    test_bool(vsLensUndistortPoint(&ld, ld.cx + r, ld.cy, &ux, &uy) != VS_OK);
  }
  /* D_k needs 1 - 4*k*r^2 >= 0, which only bites for positive k */
  {
    VSLensDistortion ld = vsLensDistortionInit(&fi, 0.9);
    double r = 0.95*ld.rho;
    test_bool(vsLensDistortPoint(&ld, ld.cx + r, ld.cy, &ux, &uy) != VS_OK);
  }
  /* barrel is always in the domain of D_k, however strong */
  {
    VSLensDistortion ld = vsLensDistortionInit(&fi, -0.9);
    double r = 0.999*ld.rho;
    test_bool(vsLensDistortPoint(&ld, ld.cx + r, ld.cy, &ux, &uy) == VS_OK);
    test_bool(!isnan(ux) && !isnan(uy));
  }
}

/* Barrel (k<0) must pull the undistorted point outward, pincushion inward.
   This pins the sign convention, which is the easiest thing to get backwards
   and the hardest to notice once everything else is symmetric. */
static void test_lensdistortion_sign_convention(void){
  VSFrameInfo fi;
  double ux, uy, r;

  test_bool(vsFrameInfoInit(&fi, 1280, 720, PF_GRAY8) != 0);
  fprintf(stderr, "--- lens distortion sign convention ---\n");

  {
    VSLensDistortion ld = vsLensDistortionInit(&fi, -0.2); /* barrel */
    r = 0.6*ld.rho;
    test_bool(vsLensUndistortPoint(&ld, ld.cx + r, ld.cy, &ux, &uy) == VS_OK);
    fprintf(stderr, "barrel     k=-0.2: r %.1f -> %.1f (expect outward)\n",
            r, ux - ld.cx);
    test_bool(ux - ld.cx > r);
    test_bool(fabs(uy - ld.cy) < 1e-12);  /* stays on the ray */
  }
  {
    VSLensDistortion ld = vsLensDistortionInit(&fi, 0.2); /* pincushion */
    r = 0.6*ld.rho;
    test_bool(vsLensUndistortPoint(&ld, ld.cx + r, ld.cy, &ux, &uy) == VS_OK);
    fprintf(stderr, "pincushion k= 0.2: r %.1f -> %.1f (expect inward)\n",
            r, ux - ld.cx);
    test_bool(ux - ld.cx < r);
  }
}

void test_lensdistortion_model(void){
  test_lensdistortion_roundtrip();
  test_lensdistortion_domain();
  test_lensdistortion_sign_convention();
}

/* --- cycle 2: synthetic local motions ------------------------------------ */

/* Camera path shapes.  They exist to probe identifiability: translation is the
   only one that constrains k well, rotation not at all.  See the docs. */
typedef enum {
  LD_PATH_TRANSLATION,  /* shift only, the favourable case */
  LD_PATH_ROTATION,     /* rotation about the centre only, the degenerate case */
  LD_PATH_MIXED,        /* translation plus large rotation */
  LD_PATH_ZOOM          /* zoom plus small translation, the weak case */
} LDPathMode;

typedef struct {
  int    width, height;
  int    gridX, gridY;   /* number of measurement fields across and down */
  int    margin;         /* inset of the field grid from the frame border, px */
  double k;              /* ground-truth distortion to bake into the motions */
  int    numFrames;      /* number of frame pairs */
  double noiseSigma;     /* gaussian pixel noise added to each displacement */
  int    quantise;       /* round displacements to integers, as int16 Vec would */
  double outlierFrac;    /* fraction of fields belonging to an independently moving object */
  double outlierShiftX, outlierShiftY;  /* that object's own motion, px */
  uint32_t seed;         /* LCG seed, so every run is bit-reproducible */
  LDPathMode pathMode;
} LDSynthConfig;

/* Everything a generated clip consists of, owned by the caller. */
typedef struct {
  VSFrameInfo    fi;
  VSPointMatches* frames;     /* numFrames entries */
  int            numFrames;
  int            numFields;   /* correspondences per frame */
  VSTransform*   truth;       /* ground-truth inter-frame similarity per frame */
  double*        storage;     /* backing buffer for all the px/py/qx/qy arrays */
} LDSynthClip;

static LDSynthConfig ldDefaultSynthConfig(void){
  LDSynthConfig c;
  c.width = 1280; c.height = 720;
  c.gridX = 16;   c.gridY = 12;
  c.margin = 40;
  c.k = -0.1;
  c.numFrames = 12;
  c.noiseSigma = 0.0;
  c.quantise = 0;
  c.outlierFrac = 0.0;
  c.outlierShiftX = 11.0; c.outlierShiftY = -8.0;
  c.seed = 20260805u;
  c.pathMode = LD_PATH_TRANSLATION;
  return c;
}

/* Numerical Recipes style LCG.  Deliberately local: the tests must not depend
   on rand(), whose sequence differs between platforms and would be perturbed
   by any other test that happens to draw from it first. */
static uint32_t ldRandNext(uint32_t* s){
  *s = (*s) * 1664525u + 1013904223u;
  return *s;
}
/* uniform in [0,1): 24 kept bits over 2^24.  Dividing by 2^23 instead would
   silently give [0,2), which makes ldRandGauss take the log of a number above
   one and return NaN for half its draws. */
static double ldRandUnit(uint32_t* s){
  return (double)(ldRandNext(s) >> 8) / 16777216.0;
}
/* uniform in [-1,1) */
static double ldRandUniform(uint32_t* s){
  return ldRandUnit(s)*2.0 - 1.0;
}
/* standard normal, Box-Muller; returns one of the pair and discards the other,
   which costs a little entropy and keeps the call site simple */
static double ldRandGauss(uint32_t* s){
  double u1, u2;
  do { u1 = ldRandUnit(s); } while(u1 <= 1e-12);
  u2 = ldRandUnit(s);
  return sqrt(-2.0*log(u1)) * cos(2.0*M_PI*u2);
}

/* Applies a VSTransform to a point in full double precision.

   This mirrors transform_vec_double() in transformtype.c exactly, but that one
   only accepts an integer Vec, which would reintroduce quantisation in the very
   place the test is trying to avoid it.  test_lensdistortion_similarity_matches_lib
   below pins the two together so this copy cannot drift. */
static void ldApplySimilarity(const VSTransform* t, double cx, double cy,
                              double xi, double yi, double* xo, double* yo){
  double z = 1.0 + t->zoom/100.0;
  double zcos_a = z*cos(t->alpha);
  double zsin_a = z*sin(t->alpha);
  double rx = xi - cx;
  double ry = yi - cy;
  *xo =  zcos_a*rx + zsin_a*ry + t->x + cx;
  *yo = -zsin_a*rx + zcos_a*ry + t->y + cy;
}

static VSTransform ldSamplePathStep(LDPathMode mode, uint32_t* s){
  VSTransform t = null_transform();
  switch(mode){
   case LD_PATH_TRANSLATION:
    t.x = 15.0*ldRandUniform(s);
    t.y = 15.0*ldRandUniform(s);
    break;
   case LD_PATH_ROTATION:
    t.alpha = 0.05*ldRandUniform(s);
    break;
   case LD_PATH_MIXED:
    t.x     = 15.0*ldRandUniform(s);
    t.y     = 15.0*ldRandUniform(s);
    t.alpha = 0.087*ldRandUniform(s);   /* about +/- 5 degrees */
    break;
   case LD_PATH_ZOOM:
    t.zoom  = 2.0*ldRandUniform(s);
    t.x     = 2.0*ldRandUniform(s);
    t.y     = 2.0*ldRandUniform(s);
    break;
  }
  return t;
}

/* Builds a clip of synthetic correspondences:
     p_u = U_k(p);   q_u = S_i * p_u;   q = D_k(q_u)
   so the observed pair (p,q) is exactly what a rigid scene seen through a lens
   of strength k would produce under inter-frame camera motion S_i. */
static LDSynthClip ldGenerate(const LDSynthConfig* cfg){
  LDSynthClip clip;
  VSLensDistortion ld;
  int i, gx, gy, idx;
  uint32_t seed = cfg->seed;
  double stepX, stepY;
  int objW = 0, objH = 0;

  test_bool(vsFrameInfoInit(&clip.fi, cfg->width, cfg->height, PF_GRAY8) != 0);
  ld = vsLensDistortionInit(&clip.fi, cfg->k);

  clip.numFrames = cfg->numFrames;
  clip.numFields = cfg->gridX * cfg->gridY;
  clip.frames    = (VSPointMatches*)vs_malloc(sizeof(VSPointMatches)*cfg->numFrames);
  clip.truth     = (VSTransform*)vs_malloc(sizeof(VSTransform)*cfg->numFrames);
  clip.storage   = (double*)vs_malloc(sizeof(double)*4*clip.numFields*cfg->numFrames);

  stepX = (double)(cfg->width  - 2*cfg->margin) / (cfg->gridX - 1);
  stepY = (double)(cfg->height - 2*cfg->margin) / (cfg->gridY - 1);

  /* a roughly square block covering the requested fraction of the grid */
  if(cfg->outlierFrac > 0){
    double side = sqrt(cfg->outlierFrac);
    objW = (int)(cfg->gridX*side + 0.5);
    objH = (int)(cfg->gridY*side + 0.5);
    if(objW < 1) objW = 1;
    if(objH < 1) objH = 1;
  }

  for(i=0; i<cfg->numFrames; i++){
    double* base = clip.storage + 4*clip.numFields*i;
    double* px = base;
    double* py = base +   clip.numFields;
    double* qx = base + 2*clip.numFields;
    double* qy = base + 3*clip.numFields;

    clip.truth[i] = ldSamplePathStep(cfg->pathMode, &seed);
    idx = 0;
    for(gy=0; gy<cfg->gridY; gy++){
      for(gx=0; gx<cfg->gridX; gx++){
        /* field centres are integers, as real measurement fields are */
        double sx = floor(cfg->margin + gx*stepX);
        double sy = floor(cfg->margin + gy*stepY);
        double ux, uy, vx, vy, dx, dy;

        test_bool(vsLensUndistortPoint(&ld, sx, sy, &ux, &uy) == VS_OK);
        ldApplySimilarity(&clip.truth[i], ld.cx, ld.cy, ux, uy, &vx, &vy);
        test_bool(vsLensDistortPoint(&ld, vx, vy, &dx, &dy) == VS_OK);

        /* A moving object: a contiguous block of fields carries its own motion
           on top of the camera's.  Contiguous rather than scattered because
           that is what a real object looks like to the detector, and because
           scattered outliers are much easier to reject. */
        if(gx < objW && gy < objH){
          dx += cfg->outlierShiftX;
          dy += cfg->outlierShiftY;
        }

        if(cfg->noiseSigma > 0){
          dx += cfg->noiseSigma*ldRandGauss(&seed);
          dy += cfg->noiseSigma*ldRandGauss(&seed);
        }
        if(cfg->quantise){
          /* what a LocalMotion would store: an integer displacement */
          dx = sx + floor(dx - sx + 0.5);
          dy = sy + floor(dy - sy + 0.5);
        }
        px[idx] = sx; py[idx] = sy;
        qx[idx] = dx; qy[idx] = dy;
        idx++;
      }
    }
    clip.frames[i].px = px; clip.frames[i].py = py;
    clip.frames[i].qx = qx; clip.frames[i].qy = qy;
    clip.frames[i].active = 0;   /* the estimator masks internally */
    clip.frames[i].n  = clip.numFields;
  }
  return clip;
}

static void ldFreeClip(LDSynthClip* clip){
  vs_free(clip->frames); vs_free(clip->truth); vs_free(clip->storage);
  clip->frames = 0; clip->truth = 0; clip->storage = 0;
}

/* The double similarity used by the generator must agree with the library's
   own, or the estimator could be fitting a different convention than
   production code and every test would still pass. */
static void test_lensdistortion_similarity_matches_lib(void){
  VSFrameInfo fi;
  VSTransform t = new_transform(7.0, -3.0, 0.04, 1.5, 0, 0, 0);
  PreparedTransform pt;
  int i;
  const int pts[][2] = {{0,0},{100,50},{639,359},{1279,719},{320,600}};

  test_bool(vsFrameInfoInit(&fi, 1280, 720, PF_GRAY8) != 0);
  pt = prepare_transform(&t, &fi);

  fprintf(stderr, "--- generator similarity matches transform_vec_double ---\n");
  for(i=0; i<5; i++){
    Vec v = {(int16_t)pts[i][0], (int16_t)pts[i][1]};
    double ex, ey, gx, gy;
    transform_vec_double(&ex, &ey, &pt, &v);
    ldApplySimilarity(&t, fi.width/2.0, fi.height/2.0, pts[i][0], pts[i][1], &gx, &gy);
    test_bool(fabs(ex-gx) < 1e-9 && fabs(ey-gy) < 1e-9);
  }
}

/* With no distortion, the observed displacement must be exactly the camera
   translation at every field.  If this fails the generator is wrong, not the
   estimator. */
static void test_lensdistortion_generator_undistorted(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  int i, j;
  double worst = 0.0;

  cfg.k = 0.0;
  cfg.pathMode = LD_PATH_TRANSLATION;
  clip = ldGenerate(&cfg);

  fprintf(stderr, "--- generator: k=0 translation reproduces the shift exactly ---\n");
  for(i=0; i<clip.numFrames; i++){
    for(j=0; j<clip.numFields; j++){
      double ex = clip.frames[i].qx[j] - clip.frames[i].px[j] - clip.truth[i].x;
      double ey = clip.frames[i].qy[j] - clip.frames[i].py[j] - clip.truth[i].y;
      double e = sqrt(ex*ex + ey*ey);
      if(e > worst) worst = e;
    }
  }
  fprintf(stderr, "worst deviation from pure shift: %.3e px\n", worst);
  test_bool(worst < 1e-9);
  ldFreeClip(&clip);
}

/* The physical signature the estimator lives on: under barrel distortion a
   constant ideal translation shows up as a field that is compressed towards
   the frame edges.  No compression, no signal, nothing to recover. */
static void test_lensdistortion_generator_edge_compression(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  VSLensDistortion ld;
  int i, j;
  double centreSum = 0, edgeSum = 0;
  int centreN = 0, edgeN = 0;

  cfg.k = -0.25;
  cfg.numFrames = 1;
  cfg.pathMode = LD_PATH_TRANSLATION;
  clip = ldGenerate(&cfg);
  ld = vsLensDistortionInit(&clip.fi, cfg.k);

  for(i=0; i<clip.numFrames; i++){
    for(j=0; j<clip.numFields; j++){
      double dx = clip.frames[i].px[j] - ld.cx;
      double dy = clip.frames[i].py[j] - ld.cy;
      double r  = sqrt(dx*dx + dy*dy)/ld.rho;
      double mx = clip.frames[i].qx[j] - clip.frames[i].px[j];
      double my = clip.frames[i].qy[j] - clip.frames[i].py[j];
      double mag = sqrt(mx*mx + my*my);
      if(r < 0.25){ centreSum += mag; centreN++; }
      else if(r > 0.65){ edgeSum += mag; edgeN++; }
    }
  }
  test_bool(centreN > 0 && edgeN > 0);
  fprintf(stderr, "--- generator: barrel compresses motion toward the edge ---\n");
  fprintf(stderr, "mean |v| centre %.3f px, edge %.3f px (k=%.2f)\n",
          centreSum/centreN, edgeSum/edgeN, cfg.k);
  /* the whole approach depends on this being a clearly measurable difference */
  test_bool(edgeSum/edgeN < 0.9 * centreSum/centreN);
  ldFreeClip(&clip);
}

/* Same seed must give byte-identical motions, or failures will not reproduce. */
static void test_lensdistortion_generator_deterministic(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip a, b;
  int i, j, same = 1;

  a = ldGenerate(&cfg);
  b = ldGenerate(&cfg);
  for(i=0; i<a.numFrames && same; i++){
    if(a.truth[i].x != b.truth[i].x || a.truth[i].y != b.truth[i].y) same = 0;
    for(j=0; j<a.numFields && same; j++){
      if(a.frames[i].qx[j] != b.frames[i].qx[j]) same = 0;
      if(a.frames[i].qy[j] != b.frames[i].qy[j]) same = 0;
    }
  }
  fprintf(stderr, "--- generator: reproducible across runs ---\n");
  test_bool(same);
  ldFreeClip(&a); ldFreeClip(&b);
}

/* --- cycle 3: the inner similarity fit ----------------------------------- */

/* Given the true k, the fit must return the camera motion that generated the
   data.  Everything downstream profiles this fit out, so if it is biased the
   distortion estimate inherits the bias. */
static void ldCheckFitRecoversPath(LDPathMode mode, double k, const char* label){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  VSLensDistortion ld;
  int i;
  double worstXY = 0, worstA = 0, worstZ = 0, worstRes = 0;

  cfg.k = k;
  cfg.pathMode = mode;
  clip = ldGenerate(&cfg);
  ld = vsLensDistortionInit(&clip.fi, k);

  for(i=0; i<clip.numFrames; i++){
    VSTransform got;
    double residual;
    test_bool(vsLensFitSimilarity(&ld, &clip.frames[i], 3, &got, &residual) == VS_OK);
    if(fabs(got.x - clip.truth[i].x) > worstXY) worstXY = fabs(got.x - clip.truth[i].x);
    if(fabs(got.y - clip.truth[i].y) > worstXY) worstXY = fabs(got.y - clip.truth[i].y);
    if(fabs(got.alpha - clip.truth[i].alpha) > worstA) worstA = fabs(got.alpha - clip.truth[i].alpha);
    if(fabs(got.zoom - clip.truth[i].zoom) > worstZ) worstZ = fabs(got.zoom - clip.truth[i].zoom);
    if(residual > worstRes) worstRes = residual;
  }
  fprintf(stderr, "%s (k=%.2f): worst dxy %.2e  dalpha %.2e  dzoom %.2e  residual %.2e\n",
          label, k, worstXY, worstA, worstZ, worstRes);
  test_bool(worstXY < 1e-6);
  test_bool(worstA  < 1e-9);
  test_bool(worstZ  < 1e-6);
  test_bool(worstRes < 1e-6);
  ldFreeClip(&clip);
}

static void test_lensdistortion_fit_recovers_path(void){
  fprintf(stderr, "--- inner fit recovers the camera path given true k ---\n");
  ldCheckFitRecoversPath(LD_PATH_TRANSLATION, 0.0,   "translation, no distortion");
  ldCheckFitRecoversPath(LD_PATH_TRANSLATION, -0.25, "translation, strong barrel");
  ldCheckFitRecoversPath(LD_PATH_MIXED,       -0.25, "translation+rotation, barrel");
  ldCheckFitRecoversPath(LD_PATH_ZOOM,        -0.15, "zoom, barrel");
  ldCheckFitRecoversPath(LD_PATH_ROTATION,    -0.25, "rotation only, barrel");
}

/* The objective must actually discriminate: fitting with the wrong k has to
   leave a residual the fit cannot absorb, otherwise there is nothing for the
   search in cycle 4 to descend. */
static void test_lensdistortion_wrong_k_leaves_residual(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  int i;
  double resTrue = 0, resZero = 0;
  VSLensDistortion ldTrue, ldZero;

  cfg.k = -0.25;
  cfg.pathMode = LD_PATH_TRANSLATION;
  clip = ldGenerate(&cfg);
  ldTrue = vsLensDistortionInit(&clip.fi, cfg.k);
  ldZero = vsLensDistortionInit(&clip.fi, 0.0);

  for(i=0; i<clip.numFrames; i++){
    VSTransform t; double r;
    test_bool(vsLensFitSimilarity(&ldTrue, &clip.frames[i], 3, &t, &r) == VS_OK);
    resTrue += r;
    test_bool(vsLensFitSimilarity(&ldZero, &clip.frames[i], 3, &t, &r) == VS_OK);
    resZero += r;
  }
  fprintf(stderr, "--- wrong k leaves residual the similarity cannot absorb ---\n");
  fprintf(stderr, "residual with true k=-0.25: %.4e, with k=0: %.4e px\n",
          resTrue/clip.numFrames, resZero/clip.numFrames);
  test_bool(resTrue/clip.numFrames < 1e-6);
  /* a clearly measurable gap, well above any plausible noise floor */
  test_bool(resZero/clip.numFrames > 0.5);
  ldFreeClip(&clip);
}

/* Rotation commutes with any radial map, so a rotation-only path gives the
   SAME residual for every k.  This is the degeneracy, verified directly on the
   objective rather than inferred from the estimator's output. */
static void test_lensdistortion_rotation_is_blind_to_k(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  const double ks[] = {-0.4, -0.2, 0.0, 0.2};
  int ki, i;
  double res[4];

  cfg.k = -0.25;
  cfg.pathMode = LD_PATH_ROTATION;
  clip = ldGenerate(&cfg);

  fprintf(stderr, "--- rotation-only: the objective is flat in k ---\n");
  for(ki=0; ki<4; ki++){
    VSLensDistortion ld = vsLensDistortionInit(&clip.fi, ks[ki]);
    res[ki] = 0;
    for(i=0; i<clip.numFrames; i++){
      VSTransform t; double r;
      test_bool(vsLensFitSimilarity(&ld, &clip.frames[i], 3, &t, &r) == VS_OK);
      res[ki] += r/clip.numFrames;
    }
    fprintf(stderr, "  k=%5.2f -> residual %.3e px\n", ks[ki], res[ki]);
  }
  /* every k explains rotation-only data equally and perfectly */
  for(ki=0; ki<4; ki++) test_bool(res[ki] < 1e-6);
  ldFreeClip(&clip);
}

/* --- cycle 4: recovering k ------------------------------------------------ */

static void ldCheckRecovery(double trueK, LDPathMode mode, int quantise,
                            double noiseSigma, double tol, const char* label){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  VSLensEstimateConfig ecfg = vsLensEstimateGetDefaultConfig();
  VSLensEstimate est;

  cfg.k = trueK;
  cfg.pathMode = mode;
  cfg.quantise = quantise;
  cfg.noiseSigma = noiseSigma;
  clip = ldGenerate(&cfg);

  est = vsEstimateLensDistortionFromMatches(&clip.fi, clip.frames, clip.numFrames, &ecfg);
  fprintf(stderr,
          "%-34s true k %6.3f -> %8.5f  (err %8.2e, res %.3e px, curv %.2e, %i evals)\n",
          label, trueK, est.k, fabs(est.k-trueK), est.residual, est.curvature,
          est.iterations);
  test_bool(est.determined);
  test_bool(fabs(est.k - trueK) < tol);
  ldFreeClip(&clip);
}

static void test_lensdistortion_recover_noisefree(void){
  fprintf(stderr, "--- recovery from exact displacements ---\n");
  ldCheckRecovery(-0.25, LD_PATH_TRANSLATION, 0, 0.0, 1e-4, "strong barrel");
  ldCheckRecovery(-0.10, LD_PATH_TRANSLATION, 0, 0.0, 1e-4, "mild barrel");
  ldCheckRecovery( 0.15, LD_PATH_TRANSLATION, 0, 0.0, 1e-4, "pincushion");
}

/* An undistorted clip must come back as undistorted.  A estimator that always
   reports some distortion would pass every test above and be useless. */
static void test_lensdistortion_null_case(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  VSLensEstimateConfig ecfg = vsLensEstimateGetDefaultConfig();
  VSLensEstimate est;

  cfg.k = 0.0;
  cfg.pathMode = LD_PATH_TRANSLATION;
  clip = ldGenerate(&cfg);
  est = vsEstimateLensDistortionFromMatches(&clip.fi, clip.frames, clip.numFrames, &ecfg);

  fprintf(stderr, "--- null case: no distortion is not invented ---\n");
  fprintf(stderr, "true k 0 -> %.3e (residual %.3e px)\n", est.k, est.residual);
  test_bool(fabs(est.k) < 1e-4);
  ldFreeClip(&clip);
}

/* The real input: integer displacements as LocalMotion.v stores them. */
static void test_lensdistortion_recover_quantised(void){
  /* Integer rounding costs about 2e-4 in k, so 1e-3 keeps a safety factor of
     five while staying tight enough to catch a real regression.  The spec
     budgeted 1e-2 here; the estimator turned out to do far better than that. */
  fprintf(stderr, "--- recovery from integer displacements ---\n");
  ldCheckRecovery(-0.25, LD_PATH_TRANSLATION, 1, 0.0, 1e-2, "strong barrel, quantised");
  ldCheckRecovery(-0.10, LD_PATH_TRANSLATION, 1, 0.0, 1e-2, "mild barrel, quantised");
}

/* Converts a generated clip into the LocalMotions the detector would emit, so
   the public entry point is exercised on its real input type. */
static void ldClipToLocalMotions(const LDSynthClip* clip, VSManyLocalMotions* mlms){
  int i, j;
  vs_vector_init(mlms, clip->numFrames);
  for(i=0; i<clip->numFrames; i++){
    LocalMotions lms;
    vs_vector_init(&lms, clip->numFields);
    for(j=0; j<clip->numFields; j++){
      LocalMotion lm;
      lm.f.x    = (int16_t)clip->frames[i].px[j];
      lm.f.y    = (int16_t)clip->frames[i].py[j];
      lm.f.size = 32;
      lm.v.x    = (int16_t)lrint(clip->frames[i].qx[j] - clip->frames[i].px[j]);
      lm.v.y    = (int16_t)lrint(clip->frames[i].qy[j] - clip->frames[i].py[j]);
      lm.contrast = 1.0;
      lm.match    = 1.0;
      vs_vector_append_dup(&lms, &lm, sizeof(LocalMotion));
    }
    vs_vector_append_dup(mlms, &lms, sizeof(LocalMotions));
  }
}

static void test_lensdistortion_from_localmotions(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  VSManyLocalMotions mlms;
  VSLensEstimateConfig ecfg = vsLensEstimateGetDefaultConfig();
  VSLensEstimate est;
  int i;

  cfg.k = -0.25;
  cfg.quantise = 1;
  cfg.pathMode = LD_PATH_TRANSLATION;
  clip = ldGenerate(&cfg);
  ldClipToLocalMotions(&clip, &mlms);

  est = vsEstimateLensDistortion(&clip.fi, &mlms, &ecfg);
  fprintf(stderr, "--- recovery through the LocalMotions entry point ---\n");
  fprintf(stderr, "true k -0.25 -> %.5f (err %.2e)\n", est.k, fabs(est.k+0.25));
  test_bool(est.determined);
  test_bool(fabs(est.k + 0.25) < 1e-2);

  for(i=0; i<vs_vector_size(&mlms); i++) vs_vector_del(VSMLMGet(&mlms, i));
  vs_vector_del(&mlms);
  ldFreeClip(&clip);
}

/* --- cycle 5: noise, degeneracy, mixed motion ---------------------------- */

/* Runs one recovery and hands back the error, so cases can be compared to
   each other rather than only to a fixed tolerance. */
static double ldRecoveryError(double trueK, LDPathMode mode, int quantise,
                              double noiseSigma, VSLensEstimate* estOut){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  VSLensEstimateConfig ecfg = vsLensEstimateGetDefaultConfig();
  VSLensEstimate est;
  double err;

  cfg.k = trueK; cfg.pathMode = mode;
  cfg.quantise = quantise; cfg.noiseSigma = noiseSigma;
  clip = ldGenerate(&cfg);
  est = vsEstimateLensDistortionFromMatches(&clip.fi, clip.frames, clip.numFrames, &ecfg);
  err = fabs(est.k - trueK);
  if(estOut) *estOut = est;
  ldFreeClip(&clip);
  return err;
}

/* Half a pixel of matcher noise on every displacement.  The estimate must not
   merely land inside a tolerance -- it must be decisively better than assuming
   no distortion at all, or it is not buying anything. */
static void test_lensdistortion_noise(void){
  VSLensEstimate est;
  const double trueK = -0.25;
  double err = ldRecoveryError(trueK, LD_PATH_TRANSLATION, 1, 0.5, &est);

  fprintf(stderr, "--- recovery with 0.5px gaussian matcher noise ---\n");
  fprintf(stderr, "true k %.3f -> %.5f (err %.2e, residual %.3f px, sigma_k %.2e)\n",
          trueK, est.k, err, est.residual, est.uncertainty);
  /* the reported standard error must be a usable predictor of the actual
     error, not merely small; otherwise it cannot gate anything */
  test_bool(err < 5.0*est.uncertainty);
  test_bool(est.determined);
  test_bool(err < 0.02);
  /* at least ten times closer to the truth than the k=0 assumption */
  test_bool(err < 0.1*fabs(trueK));
}

/* The degeneracy, through the estimator rather than the objective: a
   rotation-only path must be reported as undetermined, not answered with a
   confident-looking number. */
static void test_lensdistortion_rotation_undetermined(void){
  VSLensEstimate clean, noisy;

  ldRecoveryError(-0.25, LD_PATH_ROTATION, 0, 0.0, &clean);
  ldRecoveryError(-0.25, LD_PATH_ROTATION, 1, 0.5, &noisy);

  fprintf(stderr, "--- rotation-only path is reported undetermined ---\n");
  fprintf(stderr, "exact:  k=%9.5f curv %.3e sigma_k %.2e determined=%i\n",
          clean.k, clean.curvature, clean.uncertainty, clean.determined);
  fprintf(stderr, "noisy:  k=%9.5f curv %.3e sigma_k %.2e determined=%i\n",
          noisy.k, noisy.curvature, noisy.uncertainty, noisy.determined);
  test_bool(!clean.determined);
  test_bool(!noisy.determined);
}

/* Rotation post-composes onto the distorted field and cannot absorb radial
   error, so adding a large rotation to a translating path must not degrade the
   estimate.  This is the empirical check on that argument. */
static void test_lensdistortion_mixed_motion(void){
  VSLensEstimate tEst, mEst;
  const double trueK = -0.25;
  double errTrans = ldRecoveryError(trueK, LD_PATH_TRANSLATION, 1, 0.5, &tEst);
  double errMixed = ldRecoveryError(trueK, LD_PATH_MIXED,       1, 0.5, &mEst);

  fprintf(stderr, "--- large rotation does not degrade recovery ---\n");
  fprintf(stderr, "translation only     : k=%.5f err %.2e\n", tEst.k, errTrans);
  fprintf(stderr, "translation+rotation : k=%.5f err %.2e\n", mEst.k, errMixed);
  test_bool(mEst.determined);
  test_bool(errMixed < 0.02);
  /* allow a factor of two of sampling slack, but not a qualitative loss */
  test_bool(errMixed < 2.0*errTrans + 1e-3);
}

/* A low residual is not enough on its own: the estimator could in principle
   buy it by bending the camera motions.  Check the recovered path too. */
static void test_lensdistortion_recovered_campath(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  VSLensEstimateConfig ecfg = vsLensEstimateGetDefaultConfig();
  VSLensEstimate est;
  VSLensDistortion ld;
  int i;
  double worstXY = 0, worstA = 0;

  cfg.k = -0.25; cfg.quantise = 1; cfg.pathMode = LD_PATH_MIXED;
  clip = ldGenerate(&cfg);
  est = vsEstimateLensDistortionFromMatches(&clip.fi, clip.frames, clip.numFrames, &ecfg);
  ld  = vsLensDistortionInit(&clip.fi, est.k);

  for(i=0; i<clip.numFrames; i++){
    VSTransform got;
    test_bool(vsLensFitSimilarity(&ld, &clip.frames[i], 3, &got, 0) == VS_OK);
    if(fabs(got.x - clip.truth[i].x) > worstXY) worstXY = fabs(got.x - clip.truth[i].x);
    if(fabs(got.y - clip.truth[i].y) > worstXY) worstXY = fabs(got.y - clip.truth[i].y);
    if(fabs(got.alpha - clip.truth[i].alpha) > worstA) worstA = fabs(got.alpha - clip.truth[i].alpha);
  }
  fprintf(stderr, "--- camera path recovered alongside k ---\n");
  fprintf(stderr, "k=%.5f, worst path error: dxy %.4f px, dalpha %.2e rad\n",
          est.k, worstXY, worstA);
  test_bool(worstXY < 0.1);
  test_bool(worstA  < 1e-4);
  ldFreeClip(&clip);
}

/* --- cycle 6: outlier rejection inside the global optimisation ----------- */

/* Robust threshold, median + stddevs*1.4826*MAD.  Deliberately not the
   mean+sigma of disableFields() in localmotion2transform.c: a moving object
   covering a fifth of the fields inflates sigma enough to hide itself, whereas
   MAD has a 50% breakdown point. */
static double ldRobustThreshold(const double* vals, int n, double stddevs){
  double* tmp = (double*)vs_malloc(sizeof(double)*n);
  double median, mad;
  int i;
  for(i=0; i<n; i++) tmp[i] = vals[i];
  qsort(tmp, n, sizeof(double), cmp_double);
  median = tmp[n/2];
  for(i=0; i<n; i++) tmp[i] = fabs(vals[i] - median);
  qsort(tmp, n, sizeof(double), cmp_double);
  mad = tmp[n/2];
  vs_free(tmp);
  return median + stddevs*1.4826*mad;
}

/* The reason the existing per-frame rejection must not simply be reused.

   On clean barrel-distorted data with no outliers at all, residuals evaluated
   under a no-distortion model are systematically radial: the fields that look
   worst are the ones furthest from the centre, because that is where the
   unmodelled distortion is largest.  Rejecting on that basis would delete
   exactly the fields that carry the distortion signal.  Evaluated at the true
   k the same test finds essentially nothing to reject. */
static void test_lensdistortion_blind_rejection_eats_the_edge(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  const double ks[2] = {0.0, -0.25};
  int ki, j;

  cfg.k = -0.25; cfg.quantise = 1; cfg.pathMode = LD_PATH_TRANSLATION;
  clip = ldGenerate(&cfg);

  fprintf(stderr, "--- residual rejection at the wrong k targets the frame edge ---\n");
  for(ki=0; ki<2; ki++){
    VSLensDistortion ld = vsLensDistortionInit(&clip.fi, ks[ki]);
    double* res = (double*)vs_malloc(sizeof(double)*clip.numFields);
    VSTransform t;
    double thresh, rejRadius = 0, allRadius = 0;
    int rejected = 0;

    /* one representative frame is enough; they all behave the same way */
    test_bool(vsLensFitSimilarity(&ld, &clip.frames[0], 3, &t, 0) == VS_OK);
    test_bool(vsLensMatchResiduals(&ld, &clip.frames[0], &t, res) == VS_OK);
    thresh = ldRobustThreshold(res, clip.numFields, 2.5);

    for(j=0; j<clip.numFields; j++){
      double dx = clip.frames[0].px[j] - ld.cx;
      double dy = clip.frames[0].py[j] - ld.cy;
      double r  = sqrt(dx*dx + dy*dy)/ld.rho;
      allRadius += r/clip.numFields;
      if(res[j] > thresh){ rejected++; rejRadius += r; }
    }
    if(rejected) rejRadius /= rejected;

    fprintf(stderr, "  assumed k=%5.2f: %3i/%i rejected, mean radius %.3f vs %.3f overall\n",
            ks[ki], rejected, clip.numFields, rejRadius, allRadius);

    if(ks[ki] == 0.0){
      /* blind rejection throws away a meaningful number of fields, and the ones
         it throws away sit markedly further out than average */
      test_bool(rejected > clip.numFields/20);
      test_bool(rejRadius > 1.25*allRadius);
    }else{
      /* at the true k there is nothing systematic left to find */
      test_bool(rejected <= clip.numFields/50);
    }
    vs_free(res);
  }
  ldFreeClip(&clip);
}

/* The payoff.  A moving object corrupts a contiguous block of correspondences;
   contiguous because that is what an object looks like to the detector, and
   because scattered outliers would be far easier to reject.

   Ten percent of the fields with motion of their own is an ordinary situation
   -- someone walking through the shot.  Without rejection that is enough to
   move k by more than a third of its value. */
static void test_lensdistortion_moving_object(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  VSLensEstimateConfig on = vsLensEstimateGetDefaultConfig();
  VSLensEstimateConfig off = vsLensEstimateGetDefaultConfig();
  VSLensEstimate estOn, estOff;
  const double trueK = -0.25;

  cfg.k = trueK; cfg.quantise = 1; cfg.noiseSigma = 0.3;
  cfg.pathMode = LD_PATH_TRANSLATION;
  cfg.outlierFrac = 0.1;
  cfg.outlierShiftX = 30.0; cfg.outlierShiftY = -21.0;
  clip = ldGenerate(&cfg);

  off.rejectOutliers = 0;
  on.rejectOutliers  = 1;
  estOff = vsEstimateLensDistortionFromMatches(&clip.fi, clip.frames, clip.numFrames, &off);
  estOn  = vsEstimateLensDistortionFromMatches(&clip.fi, clip.frames, clip.numFrames, &on);

  fprintf(stderr, "--- moving object over 10%% of the fields ---\n");
  fprintf(stderr, "rejection off: k=%8.5f (err %.2e)\n", estOff.k, fabs(estOff.k-trueK));
  fprintf(stderr, "rejection on : k=%8.5f (err %.2e, %i of %i dropped)\n",
          estOn.k, fabs(estOn.k-trueK), estOn.rejected, estOn.rejected+estOn.used);

  /* the outliers must actually do damage, or the test proves nothing */
  test_bool(fabs(estOff.k - trueK) > 0.05);
  test_bool(fabs(estOn.k  - trueK) < 0.01);
  test_bool(estOn.rejected > 0);
  test_bool(estOn.determined);
  ldFreeClip(&clip);
}

/* Where it stops working, recorded deliberately rather than left to be
   discovered later.

   The inner similarity fit is plain least squares, which has no breakdown
   resistance at all.  Once outliers are numerous enough to drag the fit itself,
   the inliers acquire large residuals too, the outliers no longer stand out
   against the robust spread, and rejection finds nothing to remove.  Measured,
   that happens between a quarter and a third of the fields.  Fixing it needs a
   robust inner fit -- IRLS or RANSAC -- not a different threshold.

   The estimate does at least fail loudly rather than quietly: the corrupted fit
   leaves a large residual, which inflates the standard error past
   maxUncertainty, so determined comes back 0.  A caller that honours that flag
   gets no answer rather than a confidently wrong one. */
static void test_lensdistortion_outlier_breakdown(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  VSLensEstimateConfig on = vsLensEstimateGetDefaultConfig();
  VSLensEstimate est;
  const double trueK = -0.25;

  cfg.k = trueK; cfg.quantise = 1; cfg.noiseSigma = 0.3;
  cfg.outlierFrac = 0.4;
  cfg.outlierShiftX = 30.0; cfg.outlierShiftY = -21.0;
  clip = ldGenerate(&cfg);

  on.rejectOutliers = 1;
  est = vsEstimateLensDistortionFromMatches(&clip.fi, clip.frames, clip.numFrames, &on);

  fprintf(stderr, "--- known limit: 40%% outliers defeat the least squares fit ---\n");
  fprintf(stderr, "k=%8.5f (err %.2e), %i dropped, determined=%i\n",
          est.k, fabs(est.k-trueK), est.rejected, est.determined);
  /* Documents the limit.  If a robust inner fit is ever added this assertion
     should start failing, which is exactly the intent. */
  test_bool(fabs(est.k - trueK) > 0.05);
  test_bool(!est.determined);   /* and it says so */
  ldFreeClip(&clip);
}

/* Rejection must not damage data that has no outliers. */
static void test_lensdistortion_rejection_harmless_when_clean(void){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  VSLensEstimateConfig on = vsLensEstimateGetDefaultConfig();
  VSLensEstimateConfig off = vsLensEstimateGetDefaultConfig();
  VSLensEstimate estOn, estOff;
  const double trueK = -0.25;

  cfg.k = trueK; cfg.quantise = 1; cfg.noiseSigma = 0.5;
  cfg.pathMode = LD_PATH_TRANSLATION;
  clip = ldGenerate(&cfg);

  off.rejectOutliers = 0;
  on.rejectOutliers  = 1;
  estOff = vsEstimateLensDistortionFromMatches(&clip.fi, clip.frames, clip.numFrames, &off);
  estOn  = vsEstimateLensDistortionFromMatches(&clip.fi, clip.frames, clip.numFrames, &on);

  fprintf(stderr, "--- rejection does not hurt clean data ---\n");
  fprintf(stderr, "rejection off: k=%8.5f   on: k=%8.5f\n", estOff.k, estOn.k);
  test_bool(fabs(estOn.k - trueK) < 0.02);
  /* and it must not move the answer much compared to not rejecting at all */
  test_bool(fabs(estOn.k - estOff.k) < 0.01);
  ldFreeClip(&clip);
}


/* --- cycle 7: end to end through the real motion detector ---------------- */

/* Everything above feeds the estimator analytic correspondences.  This section
   renders actual barrel-distorted frames, runs vsMotionDetection on them, and
   estimates k from whatever the matcher really found -- outliers, quantisation,
   matching error and all. */

static uint8_t ldSampleBilinear(const uint8_t* src, int w, int h, int stride,
                                double x, double y){
  int x0, y0;
  double fx, fy, top, bot;
  if(x < 0) x = 0;
  if(x > w-1.001) x = w-1.001;
  if(y < 0) y = 0;
  if(y > h-1.001) y = h-1.001;
  x0 = (int)x; y0 = (int)y;
  fx = x - x0; fy = y - y0;
  top = src[y0*stride + x0]*(1-fx)      + src[y0*stride + x0+1]*fx;
  bot = src[(y0+1)*stride + x0]*(1-fx)  + src[(y0+1)*stride + x0+1]*fx;
  return (uint8_t)(top*(1-fy) + bot*fy + 0.5);
}

/* Texture with detail at every scale, so measurement fields have something to
   lock onto right out to the frame edge where the distortion signal lives. */
static void ldFillTexture(VSFrame* f, const VSFrameInfo* fi, uint32_t* seed){
  int x, y, i;
  uint8_t* d = f->data[0];
  int stride = f->linesize[0];
  for(y=0; y<fi->height; y++)
    for(x=0; x<fi->width; x++)
      d[y*stride+x] = (uint8_t)(110 + 30*ldRandUniform(seed));
  for(i=0; i<900; i++){
    int rw = 4 + (int)(40*ldRandUnit(seed));
    int rh = 4 + (int)(40*ldRandUnit(seed));
    int rx = (int)((fi->width  - rw)*ldRandUnit(seed));
    int ry = (int)((fi->height - rh)*ldRandUnit(seed));
    int v  = (int)(255*ldRandUnit(seed));
    for(y=ry; y<ry+rh; y++)
      for(x=rx; x<rx+rw; x++)
        d[y*stride+x] = (uint8_t)v;
  }
}

/* Renders src as seen after camera motion cum, through a lens of strength k.

   A pixel y of the output shows the scene point that lands there, so the source
   is sampled at D_k(S^-1(U_k(y))): undistort the output position, undo the
   camera motion in ideal coordinates, then distort back into the source image.
   That is exactly the inverse of the forward relation the estimator models. */
static void ldRenderWarped(const VSFrame* src, VSFrame* dst, const VSFrameInfo* fi,
                           const VSLensDistortion* ld, const VSTransform* cum){
  double z = 1.0 + cum->zoom/100.0;
  double c = z*cos(cum->alpha), sn = z*sin(cum->alpha);
  double det = c*c + sn*sn;
  int x, y;
  for(y=0; y<fi->height; y++){
    for(x=0; x<fi->width; x++){
      double ux, uy, wx, wy, rx, ry, sx, sy;
      if(vsLensUndistortPoint(ld, x, y, &ux, &uy) != VS_OK){
        dst->data[0][y*dst->linesize[0]+x] = 0;
        continue;
      }
      /* undo the similarity: r = M^-1 (u - c - t), M = [[c,s],[-s,c]] */
      wx = ux - ld->cx - cum->x;
      wy = uy - ld->cy - cum->y;
      rx = ( c*wx - sn*wy)/det;
      ry = ( sn*wx + c*wy)/det;
      if(vsLensDistortPoint(ld, rx + ld->cx, ry + ld->cy, &sx, &sy) != VS_OK){
        dst->data[0][y*dst->linesize[0]+x] = 0;
        continue;
      }
      dst->data[0][y*dst->linesize[0]+x] =
        ldSampleBilinear(src->data[0], fi->width, fi->height, src->linesize[0], sx, sy);
    }
  }
}

#define LD_E2E_FRAMES 10

static void ldRunEndToEnd(double trueK, const char* label,
                          double tol, int requireDetermined){
  VSFrameInfo fi;
  VSFrame frames[LD_E2E_FRAMES];
  VSTransform cum[LD_E2E_FRAMES];
  VSLensDistortion ld;
  VSMotionDetectConfig mdconf = vsMotionDetectGetDefaultConfig("lens-e2e");
  VSMotionDetect md;
  VSManyLocalMotions mlms;
  VSLensEstimateConfig ecfg = vsLensEstimateGetDefaultConfig();
  VSLensEstimate est;
  uint32_t seed = 4242u;
  int i, totalFields = 0;

  test_bool(vsFrameInfoInit(&fi, 1280, 720, PF_GRAY8) != 0);
  ld = vsLensDistortionInit(&fi, trueK);

  for(i=0; i<LD_E2E_FRAMES; i++) vsFrameAllocate(&frames[i], &fi);
  ldFillTexture(&frames[0], &fi, &seed);

  /* a random walk, so consecutive frames differ by roughly ten pixels of shift
     and a fraction of a degree -- ordinary handheld shake */
  cum[0] = null_transform();
  for(i=1; i<LD_E2E_FRAMES; i++){
    cum[i] = cum[i-1];
    cum[i].x     += 10.0*ldRandUniform(&seed);
    cum[i].y     += 10.0*ldRandUniform(&seed);
    cum[i].alpha += 0.008*ldRandUniform(&seed);
  }
  /* frame 0 is the undisplaced view through the same lens */
  {
    VSFrame base;
    vsFrameAllocate(&base, &fi);
    memcpy(base.data[0], frames[0].data[0], (size_t)frames[0].linesize[0]*fi.height);
    for(i=0; i<LD_E2E_FRAMES; i++) ldRenderWarped(&base, &frames[i], &fi, &ld, &cum[i]);
    vsFrameFree(&base);
  }

  test_bool(vsMotionDetectInit(&md, &mdconf, &fi) == VS_OK);
  md.conf.numThreads = 1;
  vs_vector_init(&mlms, LD_E2E_FRAMES);
  for(i=0; i<LD_E2E_FRAMES; i++){
    LocalMotions lms;
    test_bool(vsMotionDetection(&md, &lms, &frames[i]) == VS_OK);
    /* the first call only establishes the reference frame and reports no
       motion, which carries no information about k */
    if(i == 0){ vs_vector_del(&lms); continue; }
    totalFields += vs_vector_size(&lms);
    vs_vector_append_dup(&mlms, &lms, sizeof(LocalMotions));
  }

  est = vsEstimateLensDistortion(&fi, &mlms, &ecfg);
  fprintf(stderr,
          "%s: true k %6.3f -> %8.5f  (err %.2e, %i fields over %i pairs, "
          "%i dropped, res %.3f px, sigma_k %.2e, determined=%i)\n",
          label, trueK, est.k, fabs(est.k-trueK), totalFields,
          vs_vector_size(&mlms), est.rejected, est.residual,
          est.uncertainty, est.determined);

  test_bool(fabs(est.k - trueK) < tol);
  if(requireDetermined) test_bool(est.determined);

  for(i=0; i<vs_vector_size(&mlms); i++) vs_vector_del(VSMLMGet(&mlms, i));
  vs_vector_del(&mlms);
  vsMotionDetectionCleanup(&md);
  for(i=0; i<LD_E2E_FRAMES; i++) vsFrameFree(&frames[i]);
}

void test_lensdistortion_endtoend(void){
  fprintf(stderr, "--- end to end: rendered frames through vsMotionDetection ---\n");
  ldRunEndToEnd(-0.25, "strong barrel", 0.05, 1);
  ldRunEndToEnd(-0.10, "mild barrel  ", 0.05, 1);
  ldRunEndToEnd( 0.00, "no distortion", 0.05, 0);
}


/* --- cycle 8: the transform pass uses it -------------------------------- */

/* The wiring lives in vsLocalmotions2Transforms and nowhere else.  It has to:
   k is one parameter for the whole clip, and the detection pass is streaming,
   so it never holds more than a single frame pair.  This pass gets the lot.

   What is checked here is not that a number is produced but that the resulting
   per-frame transforms are closer to the truth than the distortion-blind path
   they replace. */
static void ldCheckTransformPass(double trueK, int enable, const char* label,
                                 double* worstXYOut){
  LDSynthConfig cfg = ldDefaultSynthConfig();
  LDSynthClip clip;
  VSManyLocalMotions mlms;
  VSTransformConfig tdconf = vsTransformGetDefaultConfig("lens-phase2");
  VSTransformData td;
  VSTransformations trans;
  int i;
  double worstXY = 0;

  cfg.k = trueK; cfg.quantise = 1; cfg.noiseSigma = 0.3;
  cfg.pathMode = LD_PATH_MIXED;
  clip = ldGenerate(&cfg);
  ldClipToLocalMotions(&clip, &mlms);

  tdconf.estimateLensDistortion = enable;
  tdconf.verbose = 1;
  test_bool(vsTransformDataInit(&td, &tdconf, &clip.fi, &clip.fi) == VS_OK);
  memset(&trans, 0, sizeof(trans));
  test_bool(vsLocalmotions2Transforms(&td, &mlms, &trans) == VS_OK);
  test_bool(trans.len == clip.numFrames);

  for(i=0; i<trans.len; i++){
    double dx = fabs(trans.ts[i].x - clip.truth[i].x);
    double dy = fabs(trans.ts[i].y - clip.truth[i].y);
    if(dx > worstXY) worstXY = dx;
    if(dy > worstXY) worstXY = dy;
  }
  fprintf(stderr, "%s: worst per-frame shift error %.4f px\n", label, worstXY);
  if(worstXYOut) *worstXYOut = worstXY;

  vsTransformationsCleanup(&trans);
  vsTransformDataCleanup(&td);
  for(i=0; i<vs_vector_size(&mlms); i++) vs_vector_del(VSMLMGet(&mlms, i));
  vs_vector_del(&mlms);
  ldFreeClip(&clip);
}

static void test_lensdistortion_transform_pass(void){
  double off = 0, on = 0;

  fprintf(stderr, "--- transform pass: distortion aware vs blind ---\n");
  ldCheckTransformPass(-0.25, 0, "barrel k=-0.25, estimation off", &off);
  ldCheckTransformPass(-0.25, 1, "barrel k=-0.25, estimation on ", &on);
  /* Uncorrected barrel leaks into the reported camera motion; correcting for it
     has to measurably shrink that error or the feature is not worth its cost. */
  test_bool(off > 0.5);
  test_bool(on < 0.5*off);
}

/* On footage with no distortion the feature must change nothing measurable:
   the estimate comes back near zero, fails the "large enough to matter" guard,
   and the old code path runs untouched. */
static void test_lensdistortion_transform_pass_is_noop_when_undistorted(void){
  double off = 0, on = 0;

  fprintf(stderr, "--- transform pass: no distortion, must be a no-op ---\n");
  ldCheckTransformPass(0.0, 0, "no distortion, estimation off", &off);
  ldCheckTransformPass(0.0, 1, "no distortion, estimation on ", &on);
  test_bool(fabs(on - off) < 1e-9);
}

void test_lensdistortion_phase2(void){
  test_lensdistortion_transform_pass();
  test_lensdistortion_transform_pass_is_noop_when_undistorted();
}

void test_lensdistortion_outliers(void){
  test_lensdistortion_blind_rejection_eats_the_edge();
  test_lensdistortion_moving_object();
  test_lensdistortion_rejection_harmless_when_clean();
  test_lensdistortion_outlier_breakdown();
}

void test_lensdistortion_robustness(void){
  test_lensdistortion_noise();
  test_lensdistortion_rotation_undetermined();
  test_lensdistortion_mixed_motion();
  test_lensdistortion_recovered_campath();
}

void test_lensdistortion_estimate(void){
  test_lensdistortion_recover_noisefree();
  test_lensdistortion_null_case();
  test_lensdistortion_recover_quantised();
  test_lensdistortion_from_localmotions();
}

void test_lensdistortion_fit(void){
  test_lensdistortion_fit_recovers_path();
  test_lensdistortion_wrong_k_leaves_residual();
  test_lensdistortion_rotation_is_blind_to_k();
}

void test_lensdistortion_generator(void){
  test_lensdistortion_similarity_matches_lib();
  test_lensdistortion_generator_undistorted();
  test_lensdistortion_generator_edge_compression();
  test_lensdistortion_generator_deterministic();
}
