# Recovering barrel distortion from detected local motions

Date: 2026-08-05
Status: design approved, ready for planning

## Goal

Determine whether the single lens-distortion parameter of a barrel transform can be
recovered from the local motion fields that `vsMotionDetection` already produces, using
only the assumption that the observed scene is rigid.

This is a feasibility study. It delivers the distortion model, an estimator, a synthetic
test that answers the question quantitatively, and the written derivation. It does **not**
correct video.

## Scope

In scope:

- A single-parameter radial distortion model with analytic forward and inverse maps.
- An estimator that recovers one distortion parameter for a whole clip from
  `VSManyLocalMotions`.
- A test built on analytically generated local motions — no images, no motion detection —
  so estimator error is not conflated with matcher error.
- `docs/lens-distortion.md` carrying the full derivation.

Explicitly out of scope:

- Estimating the distortion centre (fixed at the frame centre).
- Tangential distortion, and any second radial coefficient.
- Applying the correction to video, or wiring anything into `motiondetect.c` /
  `transform.c`.
- Rolling shutter.

`VSTransform.barrel` already exists as a struct field but is read and written nowhere in
the codebase. This work leaves it untouched; adopting it is a later decision.

## Model

Coordinates are taken relative to the distortion centre `c`, fixed at
`(width/2, height/2)`, and normalised by `rho`, the half-diagonal of the frame. A radius of
`1` therefore lands on the image corner, and the distortion parameter `k` is dimensionless.

The division model (Fitzgibbon) is used. Undistortion — observed to ideal — is the closed
direction:

    U_k(x) = x / (1 + k * r_d^2),        r_d = |x|

Distortion — ideal to observed — is obtained by solving `k*r_u*r_d^2 - r_d + r_u = 0` for
`r_d` and taking the branch that tends to `r_u` as `k -> 0`:

    r_d = (1 - sqrt(1 - 4*k*r_u^2)) / (2*k*r_u)
        = 2*r_u / (1 + sqrt(1 - 4*k*r_u^2))          [rationalised]

    D_k(x) = x * 2 / (1 + sqrt(1 - 4*k*r_u^2)),   r_u = |x|

The rationalised form is the one to implement: it is numerically stable and has no
removable singularity at `k = 0`, where it reduces exactly to the identity.

Sign convention: **barrel distortion is `k < 0`**. Undistorting then pushes edge points
outward, which is the correction a barrel-distorted frame needs. Typical magnitudes for
real lenses are `|k|` in the range 0.05 to 0.3.

Domain: `D_k` requires `1 - 4*k*r_u^2 >= 0`, automatically satisfied for all `k <= 0`. `U_k`
requires `1 + k*r_d^2 > 0`, i.e. `|k| < 1` at the corners. The implementation must
guard both and report a domain error rather than producing NaN.

## The rigid-scene constraint

A measurement field centred at observed position `p` is detected to move to `q = p + v`. If
the scene is rigid and the inter-frame camera motion is the similarity `S` that vid.stab
already models — rotation `alpha`, scale `z`, translation `t`, all about the frame centre —
then in ideal coordinates the two undistorted points are related exactly by `S`:

    U_k(q) = S * U_k(p)

The residual is taken in **image space**:

    e = q - D_k( S * U_k(p) )

rather than in undistorted space. Motion-detection error is homoscedastic in pixels;
undistorting first amplifies it near the frame edge, which is precisely where the signal
about `k` lives, and would bias the fit toward the frame interior. Image space also matches
the convention of the existing `calcTransformQuality`.

## Identifiability

This determines whether the whole idea works, and it is the least obvious part.

**Rotation carries no information about `k`.** Work in centred coordinates with
`S(x) = z*R*x + t`. Both `U_k` and `D_k` are radial maps about the origin, so each commutes
exactly with the rotation `R`. Then:

    phi(x) = D_k( S( U_k(x) ) )
           = D_k( R( z*U_k(x) + R^-1 * t ) )       pull R out of the linear part
           = R * D_k( z*U_k(x) + t' )              R commutes with the radial map D_k
           = R * psi(x),      where t' = R^-1 * t

The observed field is therefore always a rotation-free distorted field post-multiplied by a
global rotation. Two consequences:

1. With `t = 0`, `psi` is itself a radial map, so `phi = R * (radial)`. A rotation-only
   camera path produces observations identical for every `k`. `k` is completely
   unobservable in that case.
2. With `t != 0`, `alpha` is a nuisance parameter that is *well determined* by the data but
   *orthogonal to* `k`. Since `‖q - R*psi(p)‖^2 = ‖R^-1*q - psi(p)‖^2` and `R^-1` is an
   isometry, estimating `alpha` is a Procrustes rotation of the observations that leaves
   every `|q|` unchanged. Rotation can only ever fit tangential residual; it cannot absorb
   the radial mismatch a wrong `k` produces.

Consequence 2 is why the estimator compares full displacement vectors rather than radii
only. Restricting to radii would discard the tangential residual — which does depend on `k`
once `t' != 0`, because `psi` is then not a radial map — inflating the variance of the
estimate while protecting against a failure mode that provably cannot occur.

Both claims above depend on the rotation centre coinciding with the distortion centre. In
vid.stab they do: `prepare_transform` applies rotation and zoom about `(width/2, height/2)`.
If the distortion centre is ever estimated as well, `alpha` couples to the centre offset and
this separation is lost.

**Pure zoom is weakly informative.** A scaling composed with radial maps is itself a radial
map, so `(z, k)` trade off against one another. Zoom-dominated segments contribute little.

**Translation is the workhorse.** Barrel distortion makes a constant ideal translation
appear as a field that compresses toward the frame edges, in a pattern that is a distinctive
function of `k`. Because handheld camera shake is translation-dominated, this is the
favourable case for real footage.

## Estimator

One `k` is estimated for the entire clip — the lens does not change between frames, and
pooling every frame pair onto a single scalar is far better conditioned than estimating per
pair. The per-frame similarities are profiled out rather than alternated with:

    E(k) = sum over frames i of  min over T_i of  sum over fields j of
               ‖ (p_j + v_ij) - D_k( T_i * U_k(p_j) ) ‖^2

`E(k)` is minimised by Brent's method on the scalar `k` over a bracket of `[-0.6, 0.3]`.
Each evaluation of `E(k)` runs a complete inner similarity fit for every frame:

1. Undistort the correspondences with the current `k`.
2. Solve the classic closed-form 4-parameter similarity least squares on the undistorted
   points. This is exact for noise-free data and serves as the initialiser.
3. Refine with a few Gauss-Newton steps on the image-space residual. The Jacobian is the
   2x2 derivative of `D_k` composed with the derivative of `S` with respect to its four
   parameters.

Profiling is preferred over alternating optimisation: it cannot stall in a curved valley the
way block coordinate descent can, it has no convergence criterion to tune, and it yields the
profile curve `E(k)` as a by-product. The curvature of that curve at the minimum is a direct
uncertainty estimate, and it is what allows the rotation-only degenerate case to be reported
as *undetermined* rather than silently returning an arbitrary value. The inner problem is
small, so the roughly 40 Brent evaluations are inexpensive.

Outlier handling reuses the pattern already in `localmotion2transform.c`: fields whose
residual exceeds one standard deviation may be masked out and the fit repeated. For the
clean synthetic cases this is inert.

## Code layout

`src/lensdistortion.h` / `src/lensdistortion.c`, a self-contained module:

    typedef struct { double k, cx, cy, rho; } VSLensDistortion;

    VSLensDistortion vsLensDistortionInit(const VSFrameInfo* fi, double k);
    int  vsLensUndistortPoint(const VSLensDistortion*, double xi, double yi,
                              double* xo, double* yo);
    int  vsLensDistortPoint  (const VSLensDistortion*, double xi, double yi,
                              double* xo, double* yo);

    typedef struct {
      double kMin, kMax;     /* Brent bracket, default -0.6 .. 0.3 */
      double tolerance;      /* Brent convergence tolerance on k, default 1e-6 */
      int    maxIterations;  /* Brent iteration cap, default 100 */
      int    gaussNewtonSteps; /* inner refinement steps, default 3 */
      double outlierStddevs; /* field masking threshold; <= 0 disables, default 0 */
      double minCurvature;   /* below this, determined is reported as 0 */
    } VSLensEstimateConfig;

    VSLensEstimateConfig vsLensEstimateGetDefaultConfig(void);

    typedef struct {
      double k;          /* recovered parameter */
      double residual;   /* RMS image-space residual per field, in pixels, at the minimum */
      double curvature;  /* d^2E/dk^2 at the minimum; small means undetermined */
      int    iterations;
      int    determined; /* 0 when curvature is below cfg->minCurvature */
    } VSLensEstimate;

    VSLensEstimate vsEstimateLensDistortion(const VSFrameInfo* fi,
                                            const VSManyLocalMotions* motions,
                                            const VSLensEstimateConfig* cfg);

`VSManyLocalMotions` comes from `serialize.h`. Nothing else in the library is modified; the
module is added to the test build and to the library build so it does not rot.

`tests/test_lensdistortion.c`, registered in `tests/tests.c` behind its own flag following
the existing `contains(argv, argc, ...)` pattern.

## Synthetic data

Local motions are generated analytically, with no frames and no motion detection:

    for each frame i, for each field centre p on a grid:
        p_u = U_k(p)
        q_u = S_i * p_u                    S_i from the sampled camera path
        q   = D_k(q_u)
        v   = round(q - p)                 stored in LocalMotion.v, an int16 Vec

Frame geometry is 1280x720 with a grid of field centres inset from the border. The camera
path is produced by a small local linear congruential generator with a fixed seed, not
`rand()`, so runs are bit-reproducible across platforms and independent of any other test.

## Test cases

1. **Model round-trip.** `D_k(U_k(p)) = p` to within 1e-9 over a grid of radii, for `k` in
   `{-0.3, -0.1, 0, 0.1}`. `k = 0` is exactly the identity. Domain guards reject
   out-of-range radii instead of returning NaN.
2. **Noise-free, continuous displacements.** Recovery to `|k_hat - k| < 1e-4`.
3. **Integer-quantised displacements.** The real `Vec` is int16, so this is the honest case.
   Tolerance `|k_hat - k| < 0.01`.
4. **Gaussian noise, sigma = 0.5 px.** Tolerance `|k_hat - k| < 0.02`, *and* the assertion
   that `k_hat` is substantially closer to the truth than `k = 0` is — that the estimate
   earns its keep rather than merely being within tolerance of a small number.
5. **Null test.** True `k = 0` recovers `k_hat` approximately 0: no hallucinated distortion
   on an undistorted clip.
6. **Rotation-only degeneracy.** A rotation-only camera path yields a flat profile curve and
   is reported with `determined == 0`, confirming the commutation result empirically rather
   than returning a spurious value.
7. **Mixed translation and rotation.** A path with large rotation (approximately +/- 5
   degrees) superimposed on translation recovers `k` no worse than the translation-only
   case. This is the empirical check on the claim that `alpha` is orthogonal to `k`.
8. **Recovered camera path.** With the recovered `k`, the per-frame similarity fits match the
   ground-truth camera path within tolerance — the distortion estimate does not achieve a low
   residual by distorting the motion estimates.

Each case runs across `k` in `{0, -0.1, -0.25}` where meaningful.

## Documentation

`docs/lens-distortion.md` carries the model and its conventions, the derivation of the
inverse map including the rationalised form, the rigid-scene constraint, the full
commutation argument and its two consequences, the identifiability discussion, and the
estimator formulation. It states the fixed-distortion-centre assumption and notes what
breaks if that assumption is later relaxed.

## Success criterion

The study succeeds if cases 2 through 5 pass at the stated tolerances and case 7 confirms
rotation does not degrade recovery. That result would justify a follow-up to apply the
correction. If `k` proves recoverable only in the noise-free case, the documented outcome —
that motion fields alone are too weak a constraint at realistic noise levels — is itself the
deliverable.
