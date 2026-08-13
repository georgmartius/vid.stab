# The rotational motion model (`fov`)

A camera that yaws does not slide its picture sideways. vid.stab's default
motion model says it does — one translation, one rotation, one zoom, applied
uniformly across the frame — and on a long lens that is very nearly true. On
wide glass it is not, and this document describes the option that fixes it and
the conditions under which it helps.

Implemented 2026-08-11 for issue #139. The planning document that preceded it
is `fov_correction.md`; this one describes what was built.

## The geometry

The ray through a pixel `p` is `K⁻¹p`, where `K` is the intrinsic matrix built
from the focal length. Rotating the camera by `R` sends that ray to `R K⁻¹p`,
and the point it came from sits at

    H = K R K⁻¹

in the unrotated frame. `H` is a homography, not a translation: expanding it
for a small yaw `ω` gives, for a point `r` pixels from the optical centre,

    x_s ≈ x_d + ω·f + ω·r²/f + …

The first correction term is the translation the current model captures. The
second is the one it cannot: it grows with the square of the distance from the
centre and with the reciprocal of the focal length. Corners move differently
from the middle, and no similarity transform can express that.

Measured on the synthetic clip (`tests/generate_fovclip.c`), holding the
picture motion fixed at 12 px and lengthening the lens:

| focal length | field of view | worst disagreement with the similarity model |
| ---: | ---: | ---: |
| 320 | 90° | 25.85 px |
| 1000 | 35° | 2.51 px |
| 3000 | 12° | 0.28 px |
| 10000 | 3.7° | 0.02 px |
| 30000 | 1.2° | 0.003 px |

The gap falls off as the square of the focal length. That quadratic decay is
what licenses `fov = 0` remaining the old code rather than a special case of
the new one — the models genuinely coincide in the limit.

## Using it

Set `VSTransformConfig.fov` to the horizontal field of view of the frame, in
degrees. `0`, the default, keeps the similarity model and is bit-identical to
the behaviour before this existed.

    conf.fov = 110.0;   /* a wide action-camera lens */

The focal length follows as `f = (width/2)/tan(fov/2)`, in pixels.

Nothing else changes. The `VSTransform` fields are reused rather than
extended: `x` and `y` become yaw and pitch scaled by `f`, so they stay in
centre-pixel units. Composition, camera path smoothing (Gaussian and L1),
`maxShift`, `maxAngle` and the `.trf` file format are all untouched, and
smoothing now happens in angle space, which is the more correct space for it.

`fov` is a transform-side parameter. Existing `.trf` files stay valid and no
re-detection is needed to try a value.

### What the number has to be

**The field of view of the stored, rectilinear frame** — not the number
printed on the lens. In particular:

- after lens correction, not before. This is why the lens work is a
  prerequisite: on the wide lenses this option is meant to help, the field of
  view of the *distorted* frame is not a well-defined quantity;
- after any crop. A centre crop of a 110° frame is not a 110° frame;
- horizontal, and of the actual pixel grid. Anamorphic and already-defished
  footage both differ from the nominal figure.

**A wrong `fov` is worse than `fov = 0`.** The model only helps when the
parameter is roughly right, and it introduces a distortion of its own when it
is not.

## What it buys

On the synthetic clip — the same shake, the same scene, the same detector, the
same fit, with only the field of view varying — the worst per-frame error of
the recovered transform:

| clip | `fov = 0` | `fov` set correctly |
| --- | ---: | ---: |
| 10° | 0.79 px | — |
| 50° | 1.08 px | — |
| 110° | 3.65 px | **0.43 px** |

0.79 px on the 10° clip is the detector's own noise floor on this scene, not a
model error. Told the field of view, the 110° clip is fitted *better than that
floor* — the perspective term is removed essentially completely, and what is
left is measurement noise.

### Interaction with lens distortion estimation

`k` is estimated from the local motions before the transforms are fitted, and
that estimate is fitted through a motion model too. Perspective expands the
periphery and pincushion expands the periphery, so an estimator that does not
know `f` explains one with the other. On synthetic footage whose true `k` is
−0.25 throughout:

| clip FOV | `k` estimated blind | `k` with the estimator told `f` |
| --- | ---: | ---: |
| 20° | −0.2418 *(determined)* | — |
| 40° | −0.1472 *(determined, and used)* | **−0.2405** |
| 70° | **+0.0504** *(determined, and used)* | **−0.2473** |
| 110° | +0.2000 *(not determined)* | −0.3763 |

Blind, the estimate is not merely imprecise — by 70° it has the **wrong sign**
and is still reported as determined, because the confidence gate keys on
scatter (the uncertainty stays near 0.01) while this is systematic bias. A
wrong `k` is then handed to the render path and a wrong distortion is applied
to the picture.

`VSLensEstimateConfig.f` fixes this, and `vsLocalmotions2Transforms` sets it
from `conf.fov` automatically — for the estimate, for the lens-aware transform
fit, and for the residual the outlier rejection is built on, which has to use
the same model or it cuts the frame edge systematically.

Two things this does *not* fix, both measured:

- **110° still lands at −0.3763.** That residual error is not the model.
  `test_fov_estimator_exact()` feeds the same estimator correspondences
  computed straight from the model and recovers −0.2537. What is left is the
  **detector**: block matching assumes a local translation, and at 110° the
  perspective term stretches a 16 px field enough to bias the match it
  returns. Fixing that means changing the matcher.
- **At 70° the good estimate is discarded.** −0.2473 is accurate, but its
  uncertainty (0.0233) exceeds the default `maxUncertainty` of 0.02, so it is
  reported "not determined" and not used. See below.

### On `maxUncertainty`, and why not simply raise it

`maxUncertainty` (0.02) gates whether an estimate is used at all. It dates from
`71360a0`, which replaced a raw-curvature test with a least-squares standard
error, `uncertainty = residual/sqrt(N·curvature)`. Its calibration then was
0.0029 predicted against 0.0049 actual under 0.5 px noise, versus 0.47 for the
unidentifiable case it had to reject. **0.02 is a round number in the middle of
a three-orders-of-magnitude gap, not a tuned one** — nothing was ever measured
near it.

Whether it can be raised depends on whether that standard error still bounds
the actual error under the rotational model. Measured, with `f` supplied
throughout and true k = −0.25 (`test_fov_uncertainty_calibration`):

| clip FOV | σ reported | actual error | ratio |
| ---: | ---: | ---: | ---: |
| 20° | 0.0112 | 0.0122 | 1.09 |
| 40° | 0.0133 | 0.0095 | 0.72 |
| 70° | 0.0233 | 0.0027 | 0.11 |
| 90° | 0.0286 | **0.1203** | **4.20** |
| 110° | 0.0417 | **0.1263** | **3.03** |

There is a sharp break between 70° and 90°. Below it σ bounds the error and the
gate is merely conservative. At and above it the error jumps by an order of
magnitude while σ barely doubles — because that error is detector *bias*, and a
standard error computed from scatter cannot see bias at any threshold.

**σ does scale with field of view** — 0.011 to 0.042, roughly fourfold from 20°
to 110° — so one could scale the threshold with `fov`. But that is exactly
backwards: the regime needing the larger allowance is the regime where σ stops
being trustworthy.

**The risk of 0.05 specifically.** Every row above has σ < 0.05, so 0.05 calls
all of them determined, including the 90° and 110° estimates that are wrong by
0.12. Applied, a k of −0.37 against a true −0.25 leaves **≈21 px of residual
distortion at the corner** of a 640×480 frame — better than the 69 px of
leaving it uncorrected, but not what "determined" ought to promise. What 0.05
buys is the accurate 70° case (error 0.0027, currently discarded).

The two are separated by 0.005 of σ on a single synthetic clip. Tuning a
threshold into that gap is curve-fitting, not calibration.

Two observations that matter more than the number:

- **The gate is doing less protective work than it looks.** The blind wide-FOV
  failure (k = +0.20, error 0.45) has σ = 0.0109 — *below* 0.02. It is rejected
  only by the separate `pinned` check, because its minimum lands on the `kMax`
  bracket edge. On footage whose blind minimum falls just inside the bracket,
  it would be reported determined and used, at any threshold.
- **Raising the threshold does not touch that case** (`pinned` is independent),
  so 0.05 is not the danger it might appear — the danger is already present at
  0.02, and lives in the `pinned` check being the thing that happens to catch
  it.

### A caveat about barrel distortion

On the 110° clip the similarity fit is *better* with barrel distortion present
(3.06 px) than without it (3.65 px). This is not noise. Perspective pushes the
periphery outward relative to a translation; barrel pulls it inward, and the
two partly cancel.

The consequence is worth stating plainly: **correcting the lens alone, without
also setting `fov`, can make the fit worse than leaving both uncorrected.** If
you enable lens correction on wide footage, set `fov` too.

## Implementation notes

- `prepare_transform_fov()` builds the homography; `transform_vec_double()`
  takes a perspective branch. `f <= 0` is exactly the old path.
- The fit needed one line: `calcTransformQuality()` already routed through
  `prepare_transform`. The detector needed nothing at all — it already
  produces displacements on a spatial grid, and the gradient *across* that
  grid is precisely what separates a rotation from a translation.
- All four warp loops (planar and packed, float and fixed point) implement the
  same map. The homography is applied in **luma units**, so `f` is one number
  for the whole frame rather than something scaled per plane; getting this
  wrong is invisible on 4:2:0 and wrong on 4:2:2 (cf. issue #79, commit
  9e2f8b7).
- The fixed-point loops compute the divide in double and hand 16.16
  coordinates to the existing interpolators. Interpolation dominates the cost,
  and nothing on the `fov = 0` path changed.
- Required zoom is measured off the frame border under the rotational model
  rather than taken from the closed-form similarity approximation, which would
  under-zoom and leave black borders.

## Can `fov` be estimated?

Partly, and not reliably enough to ship. `k` is estimated from the local
motions, so the same question arises here, and
`test_fov_identifiability()` measures it: fit the clip at a range of assumed
values and look at the residual.

| clip | best residual, as a fraction of the `fov = 0` residual | minimum at |
| --- | ---: | ---: |
| 110° | 0.867 | 100° (truth 110°) |
| 110° + barrel (k = −0.25) | 0.828 | 100° (truth 110°) |
| 60° | **0.999** | 40° (truth 60°) |

On a genuinely wide lens there is a clear interior minimum, 13% below what the
similarity model leaves, and a barrel lens on top does not destroy it. But the
minimum is biased low by about a tenth, and at 60° the curve is **flat** — a
fifth of a percent from end to end, which is noise. An estimator run on 60°
footage would return an arbitrary number, and an arbitrary `fov` is worse than
none.

So any estimator needs a confidence gate keyed on the depth of the minimum,
like the lens estimator's `determined` flag, and it would have to decline far
more often than it fires. Until that exists, `fov` is a user parameter. The
bounds in `test_fov_identifiability()` record what a future estimator has to
respect.

## Not addressed

Rolling shutter, and parallax. Neither is a camera rotation, and neither is
helped by this.
