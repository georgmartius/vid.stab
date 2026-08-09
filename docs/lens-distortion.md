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

### Outlier rejection

Local motions from real footage contain outliers: moving objects, mismatches. The
per-frame rejection already in `localmotion2transform.c` is **not** reused here, and the
reason is specific. Its first stage thresholds `lm->match`, the matcher's own confidence —
model-independent, and safe to apply before `k` is known. Its second stage thresholds the
residual under the currently fitted similarity, which assumes *no distortion*. Under
uncorrected barrel those residuals are systematically radial, largest at the frame edge, so
that stage would preferentially delete the fields carrying the entire distortion signal.

Measured on clean barrel-distorted data with no outliers at all: rejecting at `k = 0` drops
18 of 192 fields whose mean radius is 0.84 against 0.53 overall. Rejecting at the true `k`
drops none.

So the global search runs its own rejection, with two deliberate differences:

- **Evaluated at the current `k`**, never at zero, so systematic distortion residual is not
  mistaken for a frame full of outliers.
- **The mask is fixed for the duration of each Brent search** and updated only between
  passes. Rejecting inside the objective would let a wrong `k` discard more points and so
  lower `E(k)` for free — biasing the profile and making it discontinuous, which is not a
  function Brent can minimise.
- **Threshold is median + n·1.4826·MAD**, not mean + n·σ. A moving object covering a fifth of
  the fields inflates the standard deviation enough to hide inside its own threshold; MAD
  tolerates up to half the data being bad.

With a tenth of the fields carrying an object's own motion, this takes the error in `k` from
1.2e-01 to 3.5e-03. On clean data it moves the answer by under 1e-3.

**Known limit.** Beyond roughly a quarter to a third outliers it stops working: the inner
similarity fit is plain least squares with no breakdown resistance, so once outliers drag the
fit itself, the inliers' residuals inflate with it and nothing stands out to reject. Measured
at 40% outliers, rejection removes nothing and `k` is off by 0.28. It does fail loudly rather
than quietly — the inflated residual pushes the standard error past `maxUncertainty` and
`determined` comes back 0. Fixing it properly needs a robust inner fit (IRLS or RANSAC), not
a different threshold.

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

Through the real detector — barrel-distorted frames rendered and put through
`vsMotionDetection`, so the correspondences carry genuine matching error, quantisation and
outliers — over nine frame pairs and roughly 2500 fields:

| true `k` | recovered | error | dropped as outliers |
|---|---|---|---|
| -0.25 | -0.24613 | 3.9e-03 | 170 |
| -0.10 | -0.09937 | 6.3e-04 | 158 |
| 0.00 | +0.00147 | 1.5e-03 | 200 |

Three things are worth drawing out. Recovery from the *integer* displacements a `LocalMotion`
actually stores is accurate to about 1e-3, an order of magnitude better than the feasibility
study set as its target. Adding a large rotation does not degrade the estimate, confirming the
commutation argument empirically. And the reported `sigma_k` is a usable predictor of the
actual error — 2.9e-3 against 4.9e-3 — while separating the identifiable cases from the
rotation-only degeneracy by three orders of magnitude.

## 6. Where this runs

The estimate is made in the **transform pass**, in `vsLocalmotions2Transforms`, and nowhere
near motion detection. That is forced by the structure of the problem rather than chosen:
`k` is a single parameter pooled over the whole clip, while detection is streaming and never
holds more than one frame pair at a time. The transform pass is the first point at which the
complete set of local motions exists.

`VSTransformConfig.estimateLensDistortion` controls it, on by default. When enabled, the
clip's `k` is estimated once and the per-frame transforms are then fitted *through* the lens
model by `vsLensMotionsToTransform`, rather than assuming no distortion. The estimate is
acted on only when it comes back `determined` and `|k| > 0.01` — below that a corner pixel
moves less than a pixel, so acting on it would only add noise. On undistorted footage the
previous code path runs untouched, and the tests assert the two agree exactly rather than
merely closely.

The gain is in the accuracy of the reported camera motion: with `k = -0.25`, the worst
per-frame shift error falls from 2.02 px to 0.08 px, because uncorrected distortion was
previously being absorbed into the estimated motion. A verbose run prints the estimate, its
standard error, how many motions were used and dropped, and what it decided to do:

    lens distortion: k=-0.2507 +- 0.0015 from 2266 motions (38 dropped), correcting transforms
    lens distortion: k=0.0002 +- 0.0011 from 2257 motions (47 dropped), too small to matter

## 7. Status and limitations

The distortion is used to *interpret* the motions, not to correct the picture: the output
video still carries whatever barrel the lens produced. Undistorting the image itself is a
separate step in `transform.c`, and `VSTransform.barrel` is still only carried, never acted
on.

Not addressed:

- **The distortion centre is fixed** at the frame centre. Real lenses are decentred by a few
  pixels. Estimating it would also break the clean `alpha`/`k` separation of section 3.
- **One radial coefficient only.** Strong wide-angle and fisheye lenses need more, but `k1`
  and `k2` fit from 2D motion alone are likely to be poorly conditioned against each other
  and against zoom.
- **No tangential distortion.**
- **Heavy outlier loads**, past about a quarter of the fields, defeat the least-squares inner
  fit. See the known limit under outlier rejection above.
- **Rolling shutter** is a separate distortion and is not modelled.

## 8. Applying the correction to the output

Sections 1-7 cover using `k` to *interpret* the motions. This section covers the render-side
work that consumes that estimate: rewarping the actual output pixels. Implementation:
`src/lensmap.{c,h}`, integrated into the four warp loops in `src/transformfloat.c` and
`src/transformfixedpoint.c`. Tests: `tests/test_lensmap.c`, run with `tests --testLMAP`. Full
design: `docs/superpowers/specs/2026-08-09-lens-correction-render-design.md`.

### The three modes

The render path applies a backward map: for each destination pixel it computes a source
coordinate and interpolates. `VSTransformConfig.lensCorrection` selects which map:

| Mode | Backward map | Output looks like |
|---|---|---|
| `Off` | `M` | today's behaviour, bit for bit |
| `Wobble` (default) | `D_k ∘ M ∘ U_k` | the same lens, on a camera held still |
| `Full` | `D_k ∘ M` | an ideal lens; straight lines straight |

`M` is the stabilising similarity (rotation, zoom, translation) already computed by the transform
pass. `U_k` maps observed coordinates to ideal ones, `D_k` the reverse; `D_k` is always the last
step of a backward map because it is the step that lands on the sensor coordinate that must
actually be sampled.

### Why Wobble is the default

Lens distortion is anchored to the sensor, not the scene: a scene point near the frame centre in
one frame and near the edge in the next carries a different distortion displacement in each frame,
and a global similarity cannot remove that difference. It survives stabilisation as a residual
warp that grows toward the periphery — the wobble that motivated estimating `k` in the first
place. `Wobble` removes exactly that and nothing else: undistort into ideal coordinates, apply the
stabilising similarity there, redistort back into the sensor's own coordinates.

The property that makes it safe to default on is that `D_k` and `U_k` are exact inverses, so
`M = identity ⇒ D_k ∘ M ∘ U_k = identity`. A locked-off shot — or the identity-transform case any
shot passes through — is left completely untouched: no field of view lost, no peripheral
softening, no resampling at all. `Full`, by contrast, rewarps every pixel of every frame
unconditionally, whether or not the camera moved.

Measured on a synthetic clip filmed by a moving camera through a barrel lens (`k = -0.25`,
`test_lensmap_removes_wobble`): the mean absolute inter-frame difference over the region stable
across all frames is 26.93 with the lens correction off and 2.65 with `Wobble` on — the residual
wobble is not eliminated to numerical zero (interpolation error remains), but it drops to about a
tenth of its uncorrected size.

`Full` mode earns its keep on a different claim — straightness, not stability. On a distorted grid
(`test_lensmap_straightness`, same `k = -0.25`), the maximum deviation of a fitted grid column from
a straight line is 9.60 px on the distorted input, 0.00 px after `Full` correction, and 9.60 px
after `Wobble` — `Wobble` leaves the lens's own curvature exactly as it was, which is the point:
it corrects motion-induced *change*, not the lens's static signature.

### Config surface

- `VSTransformConfig.lensCorrection` (`VSLensCorrectMode`) selects the mode above. It defaults to
  `VSLensCorrectWobble`. Undistorting the picture outright (`Full`, the direct descendant of what
  an earlier design proposal, since superseded, called `correctLensDistortion`) stays opt-in:
  it is a visible, irreversible change to every frame with a real field-of-view cost for
  pincushion, so it is not switched on until a user asks for it, in contrast to
  `estimateLensDistortion`, which is on by default because merely *interpreting* the motions
  through the lens has no visible downside.
- `VSTransformConfig.lensK` is the manual override, and `0.0` is its "no override, use whatever
  was estimated" sentinel — not `NaN`. Both build with `-ffast-math`, under which `isnan()` can
  fold away and `NaN == x` can spuriously return true, so a `NaN` sentinel in a public header would
  be a trap; `0.0` costs nothing, because forcing `k = 0` and switching correction off already
  produce byte-identical output (both make `U_k` and `D_k` the identity). A user who wants no lens
  correction at all should set `lensCorrection = VSLensCorrectOff`, not `lensK = 0.0` — the latter
  reads as "I have no opinion about k", not "turn correction off", and a `lensCorrection` other
  than `Off` would keep the map wired up (just inactive, since `k = 0` builds no active map). An
  explicit non-zero `lensK` always wins over the estimate: see `vsLocalmotions2Transforms` in
  `src/localmotion2transform.c`, which only calls `vsTransformSetLensK` when
  `fabs(td->conf.lensK) <= 0.01`.

### The transforms-file path carries no k

`src/serialize.c`'s reader parses each line of a `.trf` file as `"%i %lf %lf %lf %lf %i"` — id, x,
y, alpha, zoom, extra. There is no field for `k`, and that is deliberate: extending the file format
would mean bumping `LIBVIDSTAB_FILE_FORMAT_VERSION` and carrying the compatibility burden of an old
reader meeting a new file forever, for a single scalar that is far more naturally supplied as a
command-line override than baked into a per-frame transform record. So a consumer that stabilises
by reading a transforms file, rather than by running detection and the transform pass back to
back in the same process, gets no estimate at all: `VSTransformData.lensK` stays at its `0.0`
default and correction is off, exactly like undistorted footage, unless that consumer sets
`VSTransformConfig.lensK` explicitly (or calls `vsTransformSetLensK` after init). This is the
scenario `vsTransformSetLensK` exists for in the first place — a consumer that does not share one
`VSTransformData` between the estimation pass and the render pass has no other way to hand the
number across.

### Full mode softens the periphery

For barrel distortion, `D_k`'s derivative falls below 1 away from the centre — `dD/dr ≈ 0.71` at
`r = 1`, `k = -0.25` — so the outer region of the frame is magnified: a small patch of source
pixels is stretched to cover a larger patch of output pixels. That shows up as softening toward the
edges under `Full` correction, worst at the corners. Bicubic interpolation (`VS_BiCubic`) reduces
the visible effect noticeably compared to bilinear, but **the interpolation type is not changed
automatically** when `lensCorrection` is set to `Full` — a caller who wants the sharper result
has to select `VS_BiCubic` themselves via `VSTransformConfig.interpolType`.

Chroma tracks luma correctly through this softening: on a hard vertical edge warped with the lens
active (`test_lensmap_chroma_render`, `k = -0.25`), the worst luma/chroma step offset measured is
0.27 luma px on 4:2:0, 0.28 on 4:2:2, and 0.00 on 4:4:4 (which has no subsampling to get wrong in
the first place). The 4:2:0 and 4:2:2 figures are nearly identical because both share the same
horizontal subsampling factor, and the residual is real curvature error from averaging a nonlinear
radial map over a chroma sample's footprint, not a bug — see the derivation in
`tests/test_lensmap.c`'s `lmCheckChromaAlignment`.

The lookup tables that make this all cheap enough to run per pixel are accurate to 7.13e-07 in `g`
over the part of their domain an in-frame sample can actually land on, i.e. a worst-case pixel
displacement of 0.0002 px (`test_lensmap_lut`) — negligible next to the interpolation error above.
Beyond that domain — samples that are off-frame by construction, reachable only under zoom-out or
past a pincushion lens's genuine mathematical domain edge — the tables are only required to stay
finite, positive and monotone, not accurate, since nothing depends on their precision there.

The production, fixed-point warp loop that ships agrees with an independent double-precision
reference through the same interpolator to a mean of 0.081 pixel levels (`test_lensmap_fixed_
reference`), and a companion test (`test_lensmap_fixed_float_equivalence`) is sensitive enough to
detect a systematic 0.02% error deliberately injected into the distortion lookup — the accuracy
bar is set from measurement, not chosen to make a test pass.

### Zoom budget: it cuts both ways

With the lens active, the required-zoom calculation that keeps the stabilised frame free of border
is computed from the real backward map (`vsTransformRequiredZoom` sampling
`vsLensMapBackward`), not from the old closed-form corner calculation, because the two disagree in
both directions once a lens is in the picture:

- For pure translation under barrel distortion, the old closed form **over-budgets** — it zooms in
  more than necessary — by up to 6.25 percentage points, because it reasons about corner motion
  under a pure similarity and does not know that barrel distortion already pulls the periphery
  inward. The lens-aware budget zooms less and preserves more of the field of view.
- Combined with rotation, the closed form instead **under-budgets**: in the measured worst case it
  leaves up to 2.47 px of the destination frame uncovered by the source, because rotation moves the
  point of maximum required zoom off the corner in a way the old calculation, tuned for a pure
  similarity's corner-is-worst assumption, does not track once the backward map is no longer a
  similarity at all.

Both failure directions matter equally: over-budgeting silently throws away field of view the user
never needed to lose, and under-budgeting silently leaves border pixels in the output. Neither is
acceptable, which is why the budget calculation was changed rather than patched with a fixed
margin in one direction.

### Seeing it: the checkerboard round trip

`tests/generate_lensclip.c` builds a synthetic clip that makes the whole round trip visible, and
`tests/test_lenscorrect_roundtrip.c` (`tests --testLCR`) asserts it numerically. The scene is an
analytic checkerboard whose two tones swap inside every other radial band, so cell lines show the
bending and the band circles show the radial compression that bending hides near the centre.
Frame `i` is `pattern(S_i(U_k(x)))` — the scene through a known lens, moved by a known pose —
evaluated directly at its final coordinates, so the footage carries no resampling blur and the
ground truth is exact. Correcting it with the same `k` and the exact inverse pose must return the
original scene. Design: `docs/superpowers/specs/2026-08-09-lens-checkerboard-footage-design.md`.

To regenerate the figure:

```
tests --dumpLensClip                        # writes testout/lensclip/
pnmtopng testout/lensclip/sheet.ppm > sheet.png
```

`sheet.ppm` is a 1936x480 triptych of frame 0 — base | barrel-distorted at `k = -0.25` |
`Full`-corrected — and the directory also holds every frame of the clip as `distorted_NNN.ppm`,
`full_NNN.ppm` and `wobble_NNN.ppm`. A plain `tests --all` writes none of it.

Two things worth knowing when reading the figure. The corrected panel is full-bleed, with no
border fill: under barrel `D_k` contracts, so every destination pixel samples strictly inward and
the correction crops into the source rather than running off it. Pincushion is the sign that costs
field of view — about 18.5% of the frame at `k = +0.15`. And the band period is 61 px, not a round
60, because at 60 the circle `r = 240` runs exactly tangent to the cell lines at `y = 0` and
`y = 480`, where both terms of the pattern flip along the same curve and leave a sliver thinner
than the supersampler can resolve. Any retune of the cell size or band period has to keep the two
coprime.

### Out of scope: zoom lenses

`k` is one value for the whole clip, fit once by the estimator in section 4 and carried unchanged
through every frame the render path processes. Footage that zooms during the shot has a physical
`k` that genuinely drifts — typically barrel at the wide end and pincushion at the telephoto end —
and the estimator has no way to see that: it will return a single confident average across the
whole range, and the render path will apply that average everywhere, correctly by its own model but
not by the lens's actual, changing shape. If this needs to be handled later, the natural extension
is a windowed estimate feeding a `k` that varies over the clip, not a change to the render-path
plumbing described here.
