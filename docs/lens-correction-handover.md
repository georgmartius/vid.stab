# Handover: applying the lens correction to the output frames

Written 2026-08-07. Branch `worktree-lens-distortion-estimation`.

## 1. Where things stand

`k` is estimated and **used to interpret** the motions. It is not yet used to **change the
picture**.

Already done and tested:

- `src/lensdistortion.{c,h}` — division model (`U_k`, `D_k`), per-frame similarity fit through
  the lens, global `k` estimation by Brent search over a profiled objective, outlier rejection.
- Wired into `vsLocalmotions2Transforms` (transform pass only, never detection —
  `k` is global and detection is streaming). `VSTransformConfig.estimateLensDistortion`,
  on by default, guarded by `determined && |k| > 0.01`.
- Measured: `k` recovered to ~4e-3 through the real detector; worst per-frame shift error under
  `k = -0.25` drops 2.02 px → 0.08 px.
- `docs/lens-distortion.md` — model, identifiability, estimator, outlier rejection, limits.

**Read `docs/lens-distortion.md` first.** In particular the sign convention (barrel is `k < 0`),
the `rho` normalisation (half-diagonal, so `r = 1` at the corner — the numeric value of `k` is
meaningless without it), and section 3 on identifiability.

### An honest caveat about the current intermediate state

The transforms are now fitted *in ideal (undistorted) coordinates*, but the renderer still
applies them *in distorted image coordinates*. That mismatch is O(distortion × motion) at the
frame periphery — comparable in size to the estimate bias it replaced.

So: the reported transforms are measurably better, and the camera-path smoothing consumes those,
which is a real gain. But **the final rendered stabilisation will not show the full benefit until
this handover is implemented.** The two halves are consistent only once the render applies the
transforms in ideal space too. Worth confirming on real footage before assuming the current state
is a net win end-to-end for the *output*, as opposed to for the transforms.

## 2. What "correction" means here

Output frames become **undistorted and stabilised**. Straight lines in the scene become straight
in the output.

Not in scope, and worth keeping out: estimating the distortion centre, a second radial
coefficient, tangential distortion, rolling shutter.

## 3. The core change is small

Current backward map, `src/transformfloat.c` `_FLT(transformPlanar)` ~line 335 (fixed-point twin
in `src/transformfixedpoint.c` ~line 299):

```c
float x_d1 = (x - c_d_x);
float y_d1 = (y - c_d_y);
float x_s  =  zcos_a * x_d1 + zsin_a * y_d1 + c_s_x - tx;
float y_s  = -zsin_a * x_d1 + zcos_a * y_d1 + c_s_y - ty;
interpolate(dest, x_s, y_s, ...);
```

`(x_s, y_s)` is currently read as "the source pixel". Under correction it is instead **the ideal
(undistorted) coordinate**, and the source pixel is one more step away:

```c
/* ... x_s, y_s exactly as before ... */
lensDistort(&x_src, &y_src, x_s, y_s);   /* D_k, about the plane centre */
interpolate(dest, x_src, y_src, ...);
```

That is the whole geometric change. `D_k` maps ideal → observed, which is the correct direction
for a backward (destination-driven) warp.

Note the ordering that falls out for free: the zoom lives inside `M`, so it acts *before* `D_k`
in the backward map, i.e. it zooms the **corrected** image. That is what you want, and it means
the existing zoom machinery keeps working unchanged.

## 4. The traps, roughly in order of how likely they are to bite

### 4.1 Field of view — the one real design decision

Barrel and pincushion behave **oppositely**, and this surprises people:

With half-diagonal normalisation the frame corner is at `r = 1`, so:

| | `D_k(1)` | output corner reads source radius | consequence |
|---|---|---|---|
| barrel `k = -0.25` | 0.828 | **inside** the source | no black; the outer 17% of the diagonal is discarded |
| pincushion `k = +0.15` | 1.225 | **outside** the source | **black corners** unless you zoom in |

So barrel correction is automatically safe and silently crops FOV. Pincushion correction needs
compensating zoom. The required zoom has a clean closed form: the usable output radius is
`U_k(1) = 1/(1+k)`, so

```
requiredLensZoom = 100 * max(0, k)      /* in vid.stab's zoom percentage units */
```

Feed this into the zoom budget alongside `transform_get_required_zoom` (`src/transformtype.c:310`)
before `optZoom` runs (`src/transform.c:456-491`).

Decide and expose a policy:

- **crop** (recommended default) — keep the output frame full, accept the FOV loss for barrel.
- **fit** — keep all source content by zooming out by `1/(1+k)`; produces black wedges, since a
  radial expansion does not preserve a rectangle, and costs linear resolution.

### 4.2 The identity early-out must not fire

`transformfloat.c:296` and `transformfixedpoint.c:~283`:

```c
if (t.alpha==0 && t.x==0 && t.y==0 && t.zoom == 0){ /* copy, no interpolation */ }
```

With correction active this is wrong — the frame still needs undistorting. Add the lens condition.
Conversely, keep the early-out reachable when the lens is inactive, or you lose the fast path on
undistorted footage.

### 4.3 Chroma subsampling, and 4:2:2 in particular

The loop runs per plane with `wsub`/`hsub` and per-plane centres. For **4:2:0** (`wsub == hsub`)
the plane is an isotropic scaling of luma, so a radial map in plane coordinates is the same map —
no special care needed.

For **4:2:2** (`wsub=1, hsub=0`) the plane is anisotropically scaled. A circle in luma is an
ellipse in chroma coordinates, so applying the radial formula naively in plane coordinates is
**wrong** and will show up as colour fringing that grows toward the frame edge.

The fix is cheap. The radial scale `g` is a scalar, and a scalar commutes with the per-axis
stretch, so compute the *radius* in luma-equivalent units and apply the scale in plane units:

```c
double dx = x_s - c_s_x, dy = y_s - c_s_y;
double lx = dx * (1 << wsub), ly = dy * (1 << hsub);   /* luma-equivalent */
double r2 = (lx*lx + ly*ly) * inv_rho2_luma;
double g  = lensScale(r2);
x_src = c_s_x + dx*g;  y_src = c_s_y + dy*g;           /* scale in plane units */
```

Worth knowing: the **existing rotation has the same latent issue** for 4:2:2 — `zcos_a`/`zsin_a`
are applied in plane coordinates. It matters far less there because rotation angles are small,
whereas barrel is strongly radius-dependent. Don't fix it as part of this work, but don't
replicate it either.

### 4.4 The fixed-point path

`transformfixedpoint.c` is the production path; `_FLT` variants are `TESTING` only. Coordinates
are 16.16 (`iToFp16`, `fp16ToI`, `fToFp16` at lines 39-56).

A per-pixel `sqrt` is the obvious concern. The saving grace is that **`D_k` is radial**, so the
whole map is a 1-D function of `r²`:

```
g(t) = 2 / (1 + sqrt(1 - 4*k*t)),   t = r²
```

Precompute `g` into a LUT indexed by `r²` with linear interpolation, built once in
`vsTransformDataInit`. Notes:

- Cover `t ∈ [0, ~2]` — the corner is `t = 1`, plus headroom for zoom-out.
- ~2048 entries is ample; `g` is smooth and monotone. Verify against direct computation in a test
  rather than assuming.
- Index it from integer pixel radius: `dx*dx + dy*dy` fits comfortably in `int32` (1920²+1080² ≈
  4.9e6). Dropping sub-pixel precision *in the radius only* is harmless because `g` is smooth —
  but do not drop it in the coordinate itself.
- Multiply back with a 64-bit intermediate: `x = c + (((int64_t)dx_fp16 * g_fp16) >> 16)`.

In the float path, just compute `g` directly — it is clearer, and modern `sqrtps` is cheap.
Measure before adding the LUT there.

### 4.5 Peripheral softness

For barrel, `dD/dr < 1` at the periphery (≈0.71 at `r=1`, `k=-0.25`), so the outer region is
*magnified* — a small source area is stretched over a larger output area. Expect softening at the
edges. Bicubic (`VS_BiCubic`) helps noticeably; consider whether the default interpolation should
change when correction is active, and at minimum document the effect.

### 4.6 Transforms supplied directly

`src/serialize.c:561` parses a transforms file as `"%i %lf %lf %lf %lf %i"` (id, x, y, alpha,
zoom, extra) — **`barrel` is not serialised**, and this path has no local motions to estimate from.
Correction must degrade cleanly to `k = 0` there. If you want it to work on that path, either
extend the format (bump `LIBVIDSTAB_FILE_FORMAT_VERSION`, keep the reader backward compatible) or
require the manual `k` option.

Recommendation: keep `k` in `VSTransformData` as a single clip-global value, not per
`VSTransform`. `VSTransform.barrel` is currently set by `vsLensMotionsToTransform` for diagnostics
only; leave it that way rather than making it load-bearing.

### 4.7 Zoom lenses

`k` is one value for the whole clip. Footage that zooms during the shot has a `k` that genuinely
drifts (barrel at the wide end, pincushion at the tele end). Out of scope, but the estimator will
happily return a confident average. If this matters later, the hook is a windowed estimate.

## 5. Suggested order of work

Each step is independently testable; steps 1-3 are the bulk of the value.

1. **`k = 0` bit-exactness guard first.** Before changing anything, add a test pinning current
   output for a few transforms. Every later step must keep it passing.
2. **Float path, luma only, `crop` policy, no zoom compensation.** Add `VSLensDistortion` +
   `lensActive` to `VSTransformData`, stash the estimate from `vsLocalmotions2Transforms`, insert
   `D_k` into `_FLT(transformPlanar)`, handle the early-out. Prove it with the round-trip test
   (6.2) — this is where you find out whether the whole thing works.
3. **All planes, with the 4:2:2 handling of 4.3.** Add the chroma alignment test (6.5).
4. **Fixed-point path + LUT**, with the float/fixed equivalence test (6.4).
5. **`transformPacked`** — same insertion, no subsampling, simpler.
6. **Zoom compensation and the crop/fit policy** (4.1), integrated with `optZoom`.
7. **Config surface** (section 7) and docs.

## 6. Test plan

Reusable assets already in `tests/test_lensdistortion.c`: `ldRenderWarped` (renders a frame
through `D_k` and a similarity — exactly the forward direction you need to *create* distorted
input), `ldFillTexture`, `ldSampleBilinear`, `ldGenerate`. Style references: `test_transform.c`,
`test_interpolate.c`, and `test_simd_equivalence.c` for the equivalence-test pattern.

1. **`k = 0` is bit-identical** to current output. The single most important regression guard.
2. **Round-trip** — the money test. Take a textured frame, render it distorted with `ldRenderWarped`
   at known `k`, then run the correction with the same `k` and an identity stabilising transform.
   Compare against the original over the region that maps inside the source. Residual should be
   interpolation error only; assert a mean absolute difference of a few LSB, and report it rather
   than only thresholding.
3. **Straight lines stay straight.** Render a grid, distort, correct, then fit lines to the grid
   rows/columns and measure maximum deviation. This is the classic visual check and it is
   quantifiable — much better evidence than eyeballing a PNG.
4. **Fixed-point vs float equivalence**, tolerance 1-2 LSB, over several `k` and interpolation
   types.
5. **Chroma alignment** on YUV420P *and* YUV422P. The 4:2:2 case is the one that catches 4.3;
   without it that bug ships silently.
6. **LUT vs direct computation** — max error in `g` across the table, and the resulting worst-case
   pixel displacement in px.
7. **Pincushion border behaviour** — `k > 0` produces black corners under `crop`; confirm the
   border/`def` handling does the right thing rather than sampling garbage.

## 7. Config surface

In `VSTransformConfig`, alongside the existing `estimateLensDistortion`:

- `correctLensDistortion` — apply the correction to the picture. **Keep this separate from
  `estimateLensDistortion`.** Estimating for better transforms and rewarping the picture are
  genuinely different choices, and a user may reasonably want the first without the second.
- `lensK` — manual override; `NaN`/sentinel means "use the estimate". Needed for the
  transforms-file path (4.6) and for users who know their lens.
- `lensFovPolicy` — crop / fit (4.1).

**Decided: `correctLensDistortion` defaults off**, even though `estimateLensDistortion` defaults
on. Rewarping every output pixel is a visible, irreversible change to the picture with a real FOV
cost, so it stays opt-in until it has been run over a decent range of real footage. Do not flip
this as part of implementing the feature — it is a separate call to make later, on evidence.

The ffmpeg filter lives in ffmpeg's tree, not here (`libavfilter/` holds only a README). The
in-tree consumer is `transcode/filter_transform.c:202`, which already calls
`vsLocalmotions2Transforms(td, &mlms, &fd->trans)` with the same `td` — so the estimate is
available at render time with no plumbing changes.

## 8. Open questions for whoever picks this up

1. **Default FOV policy.** Crop loses ~17% of the diagonal at `k = -0.25`. Is that acceptable
   silently, or should it warn?
2. **Should correction force bicubic?** See 4.5.
3. **Does the intermediate state actually help the rendered output?** See the caveat in section 1.
   This is worth measuring on real footage *before* building the render change, because it tells
   you whether to expect a modest improvement or a large one.
4. **Is per-clip `k` good enough on real footage**, or does the estimate wander enough between
   scenes to need windowing?

## 9. File and symbol map

| What | Where |
|---|---|
| Model, estimator, fit | `src/lensdistortion.{c,h}` |
| Maths write-up | `docs/lens-distortion.md` |
| Estimate hook | `src/localmotion2transform.c:39` `vsLocalmotions2Transforms` |
| Float warp loop | `src/transformfloat.c:290` `_FLT(transformPlanar)`, inner loop ~335 |
| Fixed-point warp loop | `src/transformfixedpoint.c` `transformPlanar`, inner loop ~299 |
| Packed warp | `transformPacked` in both files |
| Dispatch | `src/transform.c:182` |
| Fixed-point helpers | `src/transformfixedpoint.c:39-56` |
| Zoom budget | `src/transformtype.c:310` `transform_get_required_zoom`; `src/transform.c:456-491` |
| Config structs | `src/transform.h:83` `VSTransformConfig`, `:112` `VSTransformData` |
| Transforms file parser | `src/serialize.c:561` |
| Tests | `tests/test_lensdistortion.c`, `tests --testLENS` |
| In-tree consumer | `transcode/filter_transform.c:202` |

Build and run:

```
cmake -S tests -B build/tests && cmake --build build/tests -j8
build/tests/tests --all        # 43/43 at handover
build/tests/tests --testLENS   # the distortion tests alone
```
