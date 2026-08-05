# Recovering lens distortion from local motions

How vid.stab estimates a barrel/pincushion distortion parameter from the motion fields the
detector already produces, using only the assumption that the filmed scene is rigid.

Implementation: `src/lensdistortion.{c,h}`. Tests: `tests/test_lensdistortion.c`,
run with `tests --testLENS`.

## 1. The model

Take coordinates relative to the distortion centre `c` and normalise them by `rho`:

    rho = sqrt(width^2 + height^2) / 2          (half the frame diagonal)

so a normalised radius of `r = 1` lands exactly on the image corner. The centre is fixed at
the frame centre, `(width/2, height/2)`.

Normalising matters. Without it `k` would carry units of 1/pixel^2, and the same physical
lens would report different values at 1080p and at 4K. With it, `k` is a dimensionless
property of the lens, comparable across resolutions and aspect ratios.

**The value of `k` is meaningless without stating the normalisation.** Half-diagonal is one
convention among several — OpenCV normalises by focal length instead — so this `k` is *not*
directly comparable to an OpenCV `k1` without conversion.

The division model (Fitzgibbon) is used. Undistortion, mapping an observed pixel to where an
ideal lens would have put it, is the cheap direction:

    U_k(x) = x / (1 + k*r_d^2),          r_d = |x|

Distortion is its inverse. Substituting `r_u = r_d/(1 + k*r_d^2)` and clearing denominators
gives a quadratic in `r_d`:

    k*r_u*r_d^2 - r_d + r_u = 0

    r_d = (1 - sqrt(1 - 4*k*r_u^2)) / (2*k*r_u)

selecting the root that tends to `r_u` as `k -> 0`. Implemented in the rationalised form

    r_d = 2*r_u / (1 + sqrt(1 - 4*k*r_u^2))

    D_k(x) = x * 2/(1 + sqrt(1 - 4*k*r_u^2)),    r_u = |x|

which is numerically stable for small `k` and has no removable singularity at `k = 0`, where
it collapses to exactly `1` with no special case in the code.

### Sign convention

**Barrel distortion is `k < 0`.** Barrel means magnification falls off with radius, so the
observed radius is smaller than the ideal one, and undistorting must push edge points
*outward* — which requires `1 + k*r^2 < 1`. Pincushion is `k > 0`.

Both directions occur in practice. Zoom lenses characteristically show barrel at the wide end
and pincushion at the telephoto end, crossing near zero somewhere mid-range; telephoto primes
and teleconverters add pincushion. Most relevant for vid.stab, footage that has *already*
been corrected — in-camera lens profiles, a previous software pass, GoPro-style linear
de-fishing — routinely over-corrects and leaves residual pincushion. Since vid.stab usually
runs on processed footage rather than raw sensor output, `k > 0` is a realistic case, not an
exotic one.

### Domain

`U_k` requires `1 + k*r^2 > 0`; `D_k` requires `1 - 4*k*r^2 >= 0`. Both are automatically
satisfied for all barrel (`k <= 0`) at every radius. For pincushion the second is a genuine
model limit: beyond

    k = 1 / (4*r^2)      (= 0.25 at the image corner)

points have no preimage under the model at all. This is why the search bracket stops at
`kMax = 0.2` rather than something larger. Both functions return `VS_ERROR` rather than
producing a NaN when asked to leave the domain.

## 2. The rigid-scene constraint

A measurement field centred at observed position `p` is detected to have moved to
`q = p + v`. If the scene is rigid and the inter-frame camera motion is the similarity `S`
that vid.stab already models — rotation `alpha`, zoom `z`, translation `t`, all about the
frame centre — then the two points are related by `S` exactly *in ideal coordinates*:

    U_k(q) = S * U_k(p)

The residual is taken in **image space**:

    e = q - D_k( S * U_k(p) )

not in undistorted space. Motion-detection error is roughly homoscedastic in pixels;
undistorting first amplifies it near the frame edge, which is exactly where the information
about `k` lives, and would bias the fit toward the frame interior. Image space also matches
the convention of the existing `calcTransformQuality` in `localmotion2transform.c`.

## 3. Identifiability

This section decides whether the whole approach can work, and it contains the one genuinely
non-obvious result.

### Rotation carries no information about k

Work in centred coordinates with `S(x) = z*R*x + t`. Both `U_k` and `D_k` are radial maps
about the origin, so each commutes *exactly* with the rotation `R`:

    phi(x) = D_k( S( U_k(x) ) )
           = D_k( R( z*U_k(x) + R^-1 * t ) )       pulling R out of the linear part
           = R * D_k( z*U_k(x) + t' )              R commutes with the radial map D_k
           = R * psi(x),        where t' = R^-1 * t

**The observed field is always a rotation-free distorted field, post-multiplied by a global
rotation.** Two consequences follow.

*First*, if `t = 0` then `psi` is itself a radial map and `phi = R * (radial)`. A
rotation-only camera path produces observations that are **identical for every `k`**. The
parameter is not merely hard to estimate there — it is entirely unobservable.

*Second*, when `t != 0`, `alpha` is a nuisance parameter that is well determined by the data
but **orthogonal to** `k`. Since

    ‖q - R*psi(p)‖^2 = ‖R^-1*q - psi(p)‖^2

and `R^-1` is an isometry, fitting `alpha` amounts to a Procrustes rotation of the
*observations*, which leaves every `|q|` unchanged. Rotation can therefore only ever absorb
*tangential* residual; it cannot touch the *radial* mismatch that a wrong `k` produces.

This is why the estimator compares full displacement vectors rather than radii only.
Restricting to radii would discard the tangential residual — which does depend on `k` once
`t' != 0`, because `psi` is then not a radial map — inflating the variance of the estimate
while guarding against a failure mode that provably cannot occur.

Both results depend on the rotation centre coinciding with the distortion centre. In vid.stab
they do: `prepare_transform` applies rotation and zoom about `(width/2, height/2)`. **If the
distortion centre is ever estimated rather than fixed, `alpha` couples to the centre offset
and this clean separation is lost.**

### Zoom is weakly informative

A scaling composed with radial maps is itself a radial map, so `(z, k)` trade off against one
another. Zoom-dominated segments contribute little.

### Translation is the workhorse

Barrel distortion makes a constant ideal translation appear as a field that compresses toward
the frame edges, in a pattern that is a distinctive function of `k`. The test measures this
directly: at `k = -0.25`, mean displacement is 4.73 px for fields inside `r < 0.25` against
3.35 px for fields beyond `r > 0.65` — a 29% compression, far above any plausible noise floor.
Because handheld camera shake is translation-dominated, this is the favourable case for real
footage.

## 4. The estimator

One `k` is estimated for the whole clip — the lens does not change between frames, and
pooling every frame pair onto a single scalar is far better conditioned than estimating per
pair. The per-frame similarities are **profiled out** rather than alternated with:

    E(k) = sum over frames i of  min over T_i of
               sum over fields j of  ‖ (p_j + v_ij) - D_k( T_i * U_k(p_j) ) ‖^2

normalised to a mean squared residual per correspondence, in px^2. `E(k)` is minimised over
the scalar `k` by Brent's method (golden section with parabolic interpolation), which is
derivative-free — appropriate for an objective whose every evaluation is itself a nonlinear
fit.

Each evaluation of `E(k)` runs a complete inner fit for every frame:

1. Undistort the correspondences at the current `k`.
2. Solve the closed-form 4-parameter similarity least squares on the undistorted points.
   Writing the linear part as `[[c,s],[-s,c]]` to match `transform_vec_double`, the normal
   equations decouple into `c = sum(a.b)/sum(|a|^2)` and `s = sum(a x b)/sum(|a|^2)` on
   mean-centred data. This is exact for noise-free input.
3. Refine with a few Gauss-Newton steps on the image-space residual, using the analytic
   Jacobian of the radial map, `J = g*I + 2*g'*u*u^T` with `g(t) = 2/(1+sqrt(1-4kt))`.

Profiling is preferred over alternating optimisation: it cannot stall in a curved valley the
way block coordinate descent can, it has no convergence criterion to tune, and it yields the
profile curve as a by-product.

### Confidence

The curvature `C = d^2E/dk^2` at the minimum is obtained by central difference. Raw curvature
is *not* a usable confidence measure on its own — it scales with field count, motion
magnitude and noise level, so no fixed threshold is meaningful. The standard least-squares
result `var(k) = 2*sigma^2 / (d^2 SSE/dk^2)` reduces, for `E` a mean over `N`
correspondences, to

    sigma_k = residual / sqrt(N * C)

which is scale free. An estimate is reported as `determined` only when

- `sigma_k < maxUncertainty` (default 0.02), **and**
- the minimum is not pinned to either end of the bracket. A minimum at an endpoint was never
  bracketed at all — Brent converges there when the objective is still decreasing through it
  — so the value is a boundary artefact, not an estimate.

A separate absolute floor on `C` catches the genuinely flat objective, where residual and
curvature vanish together and the ratio above would otherwise report spurious confidence.

### Why the bracket is not clamped at zero

Although barrel (`k < 0`) is the expected case, `kMax` stays positive. Clamping at zero would
put the undistorted case exactly on the boundary, where no minimum can be bracketed and the
curvature is meaningless — making "it correctly found zero" unfalsifiable. It would also hide
an estimator that wants to go positive, which is real information: either already-corrected
footage, or a mis-specified model, a bad distortion centre, or non-rigid scene content. The
search range stays honest; priors belong in the interpretation, not in the optimiser.

## 5. Measured behaviour

From `tests --testLENS`, on a 1280x720 frame with a 16x12 field grid over 12 frame pairs
(2304 correspondences), camera path from a fixed-seed generator:

| Case | true `k` | recovered | error | `sigma_k` |
|---|---|---|---|---|
| exact displacements, strong barrel | -0.25 | -0.250000 | 5.2e-09 | — |
| exact displacements, mild barrel | -0.10 | -0.100000 | 1.8e-10 | — |
| exact displacements, pincushion | +0.15 | +0.150000 | 4.0e-08 | — |
| null case, no distortion | 0.00 | -1.4e-12 | 1.4e-12 | — |
| integer displacements, strong barrel | -0.25 | -0.25089 | 8.9e-04 | — |
| integer displacements, mild barrel | -0.10 | -0.10497 | 5.0e-03 | — |
| integer + 0.5px gaussian noise | -0.25 | -0.25491 | 4.9e-03 | 2.9e-03 |
| translation + large rotation, noisy | -0.25 | -0.24956 | 4.4e-04 | — |
| **rotation only, exact** | -0.25 | — | — | undetermined |
| **rotation only, noisy** | -0.25 | — | — | 0.47, undetermined |

The search converges in 12 to 14 objective evaluations.

Three things are worth drawing out. Recovery from the *integer* displacements a `LocalMotion`
actually stores is accurate to about 1e-3, an order of magnitude better than the feasibility
study set as its target. Adding a large rotation does not degrade the estimate, confirming the
commutation argument empirically. And the reported `sigma_k` is a usable predictor of the
actual error — 2.9e-3 against 4.9e-3 — while separating the identifiable cases from the
rotation-only degeneracy by three orders of magnitude.

## 6. Status and limitations

This is a feasibility study. The estimator is not wired into `motiondetect.c` or
`transform.c`, and `VSTransform.barrel` remains unused.

Not addressed:

- **The distortion centre is fixed** at the frame centre. Real lenses are decentred by a few
  pixels. Estimating it would also break the clean `alpha`/`k` separation of section 3.
- **One radial coefficient only.** Strong wide-angle and fisheye lenses need more, but `k1`
  and `k2` fit from 2D motion alone are likely to be poorly conditioned against each other
  and against zoom.
- **No tangential distortion.**
- **Real motion fields are not yet tested.** The synthetic fields here are exact
  correspondences plus noise; real ones carry outliers from moving objects and mismatches.
  `vsLensFitSimilarity` has no outlier rejection yet — the pattern in
  `localmotion2transform.c`, masking fields beyond one standard deviation and refitting, is
  the obvious thing to reuse.
- **Rolling shutter** is a separate distortion and is not modelled.
