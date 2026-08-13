/*
 * generate_fovclip.c
 *
 *  A synthetic clip for the rotational motion model: the same analytic
 *  checkerboard scene as generate_lensclip.c, but moved by a real camera
 *  rotation at a known field of view rather than by a translation.
 *
 *  A camera that yaws does not slide its picture sideways.  The ray through a
 *  pixel is K^-1 p, rotating the camera by R sends it to R K^-1 p, and the
 *  point that ray came from sits at K R K^-1 p in the unrotated frame.  That
 *  homography is a translation only in the limit of a long lens: the
 *  perspective divide leaves a residual that grows as the square of the
 *  distance from the optical centre and as the reciprocal of the focal
 *  length, which is exactly why the similarity model fails on wide glass and
 *  exactly what this clip is built to expose.  See docs/fov-model.md.
 *
 *  Everything about the scene, the supersampled rendering and the ground
 *  truth being the pattern function itself carries over unchanged from
 *  generate_lensclip.c, whose statics this file reuses; it must be included
 *  after it.  The lens is composed in the same position too, so a clip can
 *  carry barrel distortion and rotation at once -- the pairing the field of
 *  view parameter is only well defined for.
 */

#define FC_NUM_FRAMES 6

/* The scene, as lcPatternTone but with the checkerboard's parity scrambled
   per cell instead of alternating.

   The plain checkerboard cannot be used here and the reason is worth stating,
   because it is not a defect of that pattern: it repeats with a period of two
   cells, so a 64 px block sitting on it matches equally well at several
   offsets, and block matching picks among them by noise.  The lens round trip
   never noticed because it never runs a detector -- it compares rendered
   images against the analytic scene.  This test does run the detector, and on
   the periodic pattern it mis-solves whole frames at narrow field of view,
   which is precisely where the model error it is trying to measure is
   smallest.  A measurement floor above the effect is no measurement.

   Scrambling the parity with a hash of the cell indices leaves every edge,
   every tone and the radial band term exactly as they were -- so the clip
   still looks like the lens clip and still exercises the same interpolation
   -- while making each 64 px neighbourhood unique.  The hash is the usual
   pair of large odd primes; the multiply is done in unsigned to keep the
   overflow defined, and bit 16 is taken rather than bit 0 because the low
   bits of such a product are far too structured. */
static int fcPatternTone(double x, double y){
  double dx = x - LC_WIDTH/2.0;
  double dy = y - LC_HEIGHT/2.0;
  unsigned ix = (unsigned)((int)floor(x/LC_CELL) + 4096);
  unsigned iy = (unsigned)((int)floor(y/LC_CELL) + 4096);
  unsigned h  = (ix * 73856093u) ^ (iy * 19349663u);
  int cell = (int)((h >> 16) & 1u);
  int band = ((int)floor(sqrt(dx*dx + dy*dy)/LC_BAND)) & 1;
  return cell ^ band;
}

/* Horizontal field of view -> focal length in pixels.  The half angle spans
   half the width, so f = (w/2) / tan(fov/2); the /360 folds in the degree
   conversion and the halving at once. */
static double fcFocal(double fovDeg){
  return (LC_WIDTH/2.0) / tan(fovDeg * M_PI/360.0);
}

/* out = a . b, aliasing-safe so callers can accumulate in place. */
static void fcMat3Mul(const double a[9], const double b[9], double out[9]){
  double t[9];
  int i, j, k;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++){
      double s = 0;
      for(k=0; k<3; k++) s += a[3*i+k] * b[3*k+j];
      t[3*i+j] = s;
    }
  for(i=0; i<9; i++) out[i] = t[i];
}

/* R = Ry(-yaw) . Rx(pitch) . Rz(roll).
   Two conventions are pinned here, and both are pinned by assertion in
   test_fovmodel.c (test_fov_degenerates_to_similarity) rather than by this
   comment alone:

   ORDER.  The rightmost factor acts first, so roll is applied before the two
   terms that read as translation at long focal length.  That is
   rotate-then-translate, which is what lcBackwardAffine does
   (A(alpha)(p-c) + c - d).  With roll outermost instead the two models would
   disagree by |d| sin(alpha) -- 0.4 px at the amplitudes below, small but far
   too big for a limit that is supposed to be exact.

   SIGN.  Ry carries -yaw so that a positive yaw maps to a positive
   VSTransform.x.  Expanding K Ry(w) K^-1 for small w gives xs = xd + w f
   while lcBackwardAffine gives xs = xd - t.x, hence the flip; pitch and roll
   already come out with the sign VSTransform wants and are left alone. */
static void fcRotation(double yaw, double pitch, double roll, double r[9]){
  double cy = cos(-yaw),  sy = sin(-yaw);
  double cp = cos(pitch), sp = sin(pitch);
  double cr = cos(roll),  sr = sin(roll);
  double Ry[9] = {  cy, 0, sy,    0, 1,  0,  -sy,  0, cy };
  double Rx[9] = {   1, 0,  0,    0, cp,-sp,   0, sp, cp };
  double Rz[9] = {  cr,-sr, 0,   sr, cr,  0,   0,  0,  1 };
  fcMat3Mul(Ry, Rx, r);
  fcMat3Mul(r,  Rz, r);
}

/* The camera path.  Deliberately the SAME three sinusoids as lcClipTransform,
   with yaw and pitch specified as the centre-pixel excursion they produce and
   converted to an angle by dividing by f.

   Parameterising by pixels rather than by degrees is what makes the field of
   view the only variable in the experiment.  Fixed angles would move the
   picture by f times more on a long lens -- 82 px per step at 10 degrees,
   which is simply outside what block matching is asked to find -- and any
   accuracy difference between the narrow and wide clips would then be
   confounded with the size of the motion.  Fixed pixels instead give
   literally the same apparent shake at every field of view, differing only
   in the perspective term, which is the thing under test.

   It also makes the ground truth exactly lcClipTransform's own step, so the
   rotational clip and the lens clip describe the same camera. */
#define FC_X_AMP     12.0   /* px, as lcClipTransform */
#define FC_Y_AMP      9.0   /* px */
#define FC_ROLL_AMP   1.2   /* degrees */

/* The absolute attitude of frame i at focal length f.  Bounded, drift free,
   and every term vanishing at i == 0 so frame 0 is the base scene and
   nothing else -- the same properties lcClipTransform has, for the same
   reasons. */
static void fcCumAngles(int i, double f, double* yaw, double* pitch, double* roll){
  double s = (double)i;
  *yaw   = FC_X_AMP    * sin(s * 0.7) / f;
  *pitch = FC_Y_AMP    * sin(s * 0.9) / f;
  *roll  = FC_ROLL_AMP * sin(s * 0.5) * M_PI / 180.0;
}

/* The step from frame i-1 to frame i, which is what motion detection actually
   measures. */
static void fcStepAngles(int i, double f, double* yaw, double* pitch, double* roll){
  double y0, p0, r0, y1, p1, r1;
  fcCumAngles(i-1, f, &y0, &p0, &r0);
  fcCumAngles(i,   f, &y1, &p1, &r1);
  *yaw = y1 - y0; *pitch = p1 - p0; *roll = r1 - r0;
}

/* The attitude of frame i, accumulated from the steps rather than built
   directly from fcCumAngles.

   The distinction matters and is the whole reason the step angles are
   primary.  Frame i-1 and frame i are related by R_{i-1}^-1 R_i, and only if
   the poses are composed as R_i = R_{i-1} . dR_i is that product exactly
   dR_i -- the single rotation whose ground truth fcStepTransform can state in
   closed form.  Building each pose independently from three Euler angles
   would leave the relative motion a product of four rotations that is not any
   one of them, and rotations do not commute, so the residual would not be
   zero and the test would be measuring its own bookkeeping.

   The cumulative path therefore only approximately retraces fcCumAngles, by
   the second-order commutator; at these amplitudes that is well under a
   hundredth of a degree and it does not matter, because nothing asserts on
   the absolute pose. */
static void fcPose(int i, double f, double r[9]){
  double id[9] = {1,0,0, 0,1,0, 0,0,1};
  int j;
  for(j=0; j<9; j++) r[j] = id[j];
  for(j=1; j<=i; j++){
    double dyaw, dpitch, droll, d[9];
    fcStepAngles(j, f, &dyaw, &dpitch, &droll);
    fcRotation(dyaw, dpitch, droll, d);
    fcMat3Mul(r, d, r);
  }
}

/* The similarity transform the step from i-1 to i degenerates to at long
   focal length: yaw and pitch scaled into centre-pixels by f, roll straight
   through as alpha.  This is the ground truth the fit is scored against, and
   it is the same reinterpretation of the VSTransform fields the library makes
   itself; see VSTransformConfig.fov. */
static VSTransform fcStepTransform(int i, double f){
  double yaw, pitch, roll;
  fcStepAngles(i, f, &yaw, &pitch, &roll);
  return new_transform(f*yaw, f*pitch, roll, 0, 0, 0, 0);
}

/* Context for the clip's backward map: the attitude, the focal length, and
   optionally the lens. */
typedef struct {
  double           r[9];
  double           f;
  VSLensDistortion ld;
  int              useLens;
} FCMapCtx;

/* frame_i(x) = pattern( K R_i K^-1 ( U_k(x) ) ) -- undistort to the ideal
   rectilinear frame, then rotate.  The composition order is the physical one
   and it is the reason the lens has to be corrected before a field of view
   means anything: the rotation is a homography of the RECTILINEAR image, so
   the distortion has to be undone first for it to apply to the right points.

   Z cannot vanish for any pose reachable here -- it is f cos(angle) to first
   order and the angles are degrees -- so the divide needs no guard. */
static void fcRotMap(const void* ctx, double xd, double yd,
                     double* xs, double* ys){
  const FCMapCtx* c = (const FCMapCtx*)ctx;
  double cx = LC_WIDTH/2.0, cy = LC_HEIGHT/2.0;
  double ux = xd, uy = yd, rx, ry, X, Y, Z;
  if(c->useLens)
    if(vsLensUndistortPoint(&c->ld, xd, yd, &ux, &uy) != VS_OK){ ux = xd; uy = yd; }
  rx = ux - cx; ry = uy - cy;
  X = c->r[0]*rx + c->r[1]*ry + c->r[2]*c->f;
  Y = c->r[3]*rx + c->r[4]*ry + c->r[5]*c->f;
  Z = c->r[6]*rx + c->r[7]*ry + c->r[8]*c->f;
  *xs = c->f * X/Z + cx;
  *ys = c->f * Y/Z + cy;
}

/* Allocates and fills numFrames frames.  k == 0 leaves the lens out
   entirely.  Caller frees them. */
static void fcGenerateClip(VSFrame* frames, VSFrameInfo* fi, VSPixelFormat pf,
                           double fovDeg, double k, int numFrames){
  int i;
  test_bool(vsFrameInfoInit(fi, LC_WIDTH, LC_HEIGHT, pf) != 0);
  for(i=0; i<numFrames; i++){
    FCMapCtx ctx;
    ctx.f       = fcFocal(fovDeg);
    ctx.ld      = vsLensDistortionInit(fi, k);
    ctx.useLens = (k != 0.0);
    fcPose(i, ctx.f, ctx.r);
    vsFrameAllocate(&frames[i], fi);
    lcRenderMappedTone(&frames[i], fi, fcRotMap, &ctx, fcPatternTone);
  }
}
