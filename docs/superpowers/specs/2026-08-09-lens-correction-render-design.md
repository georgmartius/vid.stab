# Design: applying the lens distortion to the output frames

Written 2026-08-09. Branch `feature/lens-distortion-estimation`.

Successor to `docs/lens-correction-handover.md`, which described the render-side work as a single
"undistort the output" feature. This design keeps that mode but makes it the opt-in one, and
introduces a second, cheaper mode as the default.

Read `docs/lens-distortion.md` first for the model, the sign convention (barrel is `k < 0`) and the
`rho` normalisation (half-diagonal, `r = 1` at the corner).

## 1. The two modes

The render path applies a backward map: for each destination pixel it computes a source coordinate
and interpolates. Today that map is the stabilising similarity `M` (rotation, zoom, translation)
alone. This design defines three settings.

| Mode | Backward map | Output looks like |
|---|---|---|
| `Off` | `M` | today's behaviour, bit for bit |
| `Wobble` (default) | `D_k ∘ M ∘ U_k` | the same lens, on a camera held still |
| `Full` | `D_k ∘ M` | an ideal lens; straight lines straight |

`U_k` maps observed → ideal, `D_k` maps ideal → observed. `D_k` is the correct direction for the
final step of a backward map because it takes the ideal coordinate we have computed to the sensor
coordinate we must sample.

### 1.1 Why `Wobble` is the right default

Lens distortion is anchored to the sensor, not to the scene. A scene point near the frame centre in
one frame and near the edge in the next carries a different distortion displacement in each. A
global similarity cannot remove that difference, so it survives stabilisation as a residual warp
that grows toward the frame periphery — the wobble. This is the same residual that motivated
estimating `k` in the first place.

`Wobble` removes exactly that and nothing else. Formally, the observed frame is `D_k(S_t(scene))`
for camera motion `S_t` in ideal coordinates, and the desired output is `D_k(scene)` — the same lens
with the camera still. Undistorting the output coordinate, applying the stabilising similarity in
ideal space and redistorting into the source sensor gives `D_k ∘ M ∘ U_k`.

Three properties follow, and together they are the argument for enabling it by default:

- **`M = identity ⇒ the map is exactly identity**, since `D_k ∘ U_k = id`. A locked-off shot is
  unchanged: no FOV loss, no peripheral softening, no resampling at all.
- The warp magnitude is proportional to how far the stabiliser actually moved the frame. It only
  touches pixels the stabiliser was already going to move.
- The picture keeps the lens's own character. Straight lines stay curved, as the lens drew them.

`Full` by contrast rewarps every pixel of every frame, costs field of view, and softens the
periphery. The handover's decision that it stays opt-in is preserved.

### 1.2 What is not in scope

Distortion centre estimation, a second radial coefficient, tangential distortion, rolling shutter,
per-window `k` for zoom lenses.

## 2. Architecture

### 2.1 A new unit: `src/lensmap.{c,h}`

The render-path radial algebra lives in one new file. `src/lensdistortion.{c,h}` keeps the estimator
and the double-precision point maps and is not modified.

```c
typedef enum {
  VSLensCorrectOff = 0,   /* x_src = M(x_out)           */
  VSLensCorrectWobble,    /* x_src = D_k(M(U_k(x_out))) */
  VSLensCorrectFull,      /* x_src = D_k(M(x_out))      */
} VSLensCorrectMode;

/* Everything one plane's inner loop needs. Built once per (mode, k, geometry). */
typedef struct _vslensplanemap {
  int      active;    /* 0 -> the caller takes the existing affine-only path        */
  float    invRho2;   /* 1/rho^2 in luma-equivalent units; rho from the SOURCE frame */
  float    sx, sy;    /* luma-equivalent axis scales, (1<<wsub) and (1<<hsub)        */
  float    tMax;      /* LUT domain in t = r^2                                        */
  int      n;         /* LUT entries, 2048                                            */
  int32_t* gU;        /* U_k radial scale, 16.16 fixed point                          */
  int32_t* gD;        /* D_k radial scale, 16.16 fixed point                          */
  float*   gUf;       /* same tables in float for the _FLT path                       */
  float*   gDf;
} VSLensPlaneMap;
```

The two scale functions are the whole model:

```
gU(t) = 1 / (1 + k*t)
gD(t) = 2 / (1 + sqrt(1 - 4*k*t))        t = r^2, r in units of rho
```

Both are 1-D functions of `t`, which is what makes a LUT possible at all. Both are scalars, which is
what makes the chroma handling in 2.3 cheap.

Public surface of the unit:

- `VSLensPlaneMap vsLensPlaneMapInit(const VSFrameInfo* fiSrc, const VSFrameInfo* fiDest, int plane, double k, VSLensCorrectMode mode, double zoomHeadroom)`
- `void vsLensPlaneMapFree(VSLensPlaneMap*)`
- inline `lensScaleU(const VSLensPlaneMap*, float t)` / `lensScaleD(...)` and fixed-point twins,
  each with a direct-computation fallback outside `[0, tMax]`
- `int vsLensMapBackward(const VSLensPlaneMap*, const VSTransform*, double xd, double yd, double* xs, double* ys)`
  — a double-precision reference implementation of the whole composite map, used by the zoom budget
  in 2.4 and by the tests. The inner loops do not call it; they inline the same arithmetic.

### 2.2 Anchoring

`U_k` is anchored at the **destination** plane centre, `D_k` at the **source** plane centre. Both use
the same `k` and the same `rho`, taken from the **source** frame geometry.

That is the literal statement of "the same lens on a camera held still", and it is what makes
`D_k ∘ U_k` collapse to the identity when `M` does. Using a destination-derived `rho` would break
that collapse whenever `fiSrc` and `fiDest` differ in size.

### 2.3 Chroma planes, including 4:2:2

Handover §4.3 applies unchanged, and now to both radial steps. For 4:2:0 (`wsub == hsub`) the plane
is an isotropic scaling of luma and needs no special care. For 4:2:2 the plane is anisotropically
scaled, so a circle in luma is an ellipse in plane coordinates and the naive radial formula produces
colour fringing that grows toward the frame edge.

Because `g` is a scalar it commutes with the per-axis stretch. Compute `t` in luma-equivalent units,
apply `g` in plane units:

```c
float lx = dx * m->sx, ly = dy * m->sy;      /* luma-equivalent */
float t  = (lx*lx + ly*ly) * m->invRho2;
float g  = lensScaleU(m, t);
dx *= g; dy *= g;                            /* scale in plane units */
```

The existing rotation has the same latent issue for 4:2:2 (`zcos_a`/`zsin_a` are applied in plane
coordinates). It matters far less there because angles are small. This work does not fix it and does
not replicate it.

### 2.4 LUT

`tMax` is derived from the actual geometry rather than hardcoded: the largest `t` reachable is at a
destination corner (`t = 1` after `U_k` at worst) carried through `M`, so

```
zMax = 1 - min(0, conf.zoom)/100          /* the most zoomed-out scale factor, >= 1 */
tMax = ((1 + maxShift/rho) * zMax)^2 * safetyMargin
```

with `safetyMargin = 1.1`. Note the sign convention: the loop uses `z = 1 - zoom/100`, so negative
`zoom` is zoom-out and pushes samples to larger radii — that is the case `tMax` must cover. 2048 entries with linear interpolation; `g` is smooth and monotone in `t`
on this domain. Beyond `tMax` the inline helpers fall back to direct computation — those samples are
off-frame anyway, so the branch is cold.

Fixed-point tables are 16.16, consistent with the coordinate representation in
`transformfixedpoint.c:39-56`. The multiply back uses a 64-bit intermediate:
`x = c + (((int64_t)dx_fp16 * g_fp16) >> 16)`.

The float path keeps a float table for consistency with the fixed-point path and to keep one code
shape across both; a test compares it against direct `sqrtf` evaluation, and if the table shows no
benefit there it can be dropped from the float path later without changing behaviour.

### 2.5 Domain failures

`gU` is undefined when `1 + k*t <= 0`, which for barrel `k = -0.25` first happens at `r = 2` — far
outside the frame. `gD` is undefined when `1 - 4*k*t < 0`, which for pincushion `k = +0.15` first
happens at `r ≈ 1.29`, outside the frame corner but reachable under zoom-out.

The pixel loop cannot report an error. An out-of-domain sample resolves to the border value `def`,
exactly as an out-of-frame sample already does. `vsLensMapBackward` returns `VS_ERROR` for these so
the zoom budget and the tests can distinguish them.

## 3. Integration

### 3.1 The inner loops

Four call sites: `transformPlanar` and `transformPacked` in each of `src/transformfloat.c` and
`src/transformfixedpoint.c`. The insertion is symmetric — `U_k` on the destination-centred vector
before `M`, `D_k` on the source-centred vector after it:

```c
float dx = x - c_d_x, dy = y - c_d_y;
if (mode == VSLensCorrectWobble) {
  float g = lensScaleU(m, tOf(m, dx, dy));
  dx *= g; dy *= g;
}
float x_i =  zcos_a*dx + zsin_a*dy + c_s_x - tx;   /* ideal source coordinate */
float y_i = -zsin_a*dx + zcos_a*dy + c_s_y - ty;
if (mode != VSLensCorrectOff) {
  float ex = x_i - c_s_x, ey = y_i - c_s_y;
  float g  = lensScaleD(m, tOf(m, ex, ey));
  x_i = c_s_x + ex*g; y_i = c_s_y + ey*g;
}
interpolate(dest, x_i, y_i, ...);
```

The mode is loop-invariant; the three cases are specialised so the inactive path retains its current
instruction sequence exactly.

Ordering note carried over from the handover: zoom lives inside `M`, so it acts between `U_k` and
`D_k` in wobble mode and before `D_k` in full mode. In both cases it zooms the geometrically
corrected image, which is what is wanted, and the existing zoom machinery keeps working.

### 3.2 The identity early-out

`transformfloat.c:297` and `transformfixedpoint.c:~283` short-circuit when
`t.alpha==0 && t.x==0 && t.y==0 && t.zoom==0`.

Wobble mode needs no change here: an identity `M` gives an identity composite map, so the fast path
is still correct and — importantly — still reachable, which is the performance half of the
default-on argument. Only full mode must suppress it:

```c
if (identity && !(td->lensActive && td->lensMode == VSLensCorrectFull)) { /* copy */ }
```

### 3.3 Zoom budget

`transform_get_required_zoom` (`src/transformtype.c:299`) reasons in closed form about corner
displacement under a similarity. With a lens active the map is no longer a similarity, and the extra
corner motion of order `|k|·t·(r/rho)^2` — a few pixels for `k = -0.25` and a 20 px shift — would
show as border slivers `optZoom` did not budget for.

Add to `src/transform.c`:

```c
double vsTransformRequiredZoom(const VSTransformData* td, const VSTransform* t);
```

When the lens is inactive it delegates to the existing closed form, so the `k = 0` path stays bit
identical. When active, note that zoom sits *inside* `M`, making the required zoom a fixed point
rather than a formula. Solve it directly: overshoot is monotone in `z`, so bisect on `z` over
`[0, zoomMax]`, at each step pushing the four destination corners and four edge midpoints through
`vsLensMapBackward` and asking whether all eight land inside the source rectangle. 15 iterations
reach 0.01% and cost ~120 evaluations per frame, which is nothing against a per-pixel warp.

The eight-point sample is justified by the extremum of a radial-plus-similarity map on a rectangle
lying on the boundary, with corners and midpoints the candidates; a test validates it against a
dense boundary sweep rather than relying on the argument.

Callers are the `optZoom` paths at `src/transform.c:456-491`.

For full mode this subsumes the handover's `requiredLensZoom = 100 * max(0, k)` special case — the
pincushion FOV loss falls out of the same bisection.

### 3.4 Where `k` comes from at render time

`VSTransformData` gains:

```c
VSLensDistortion  lens;
VSLensCorrectMode lensMode;
int               lensActive;
VSLensPlaneMap    lensMaps[3];
double            lensMapK;   /* k the maps were built for, for invalidation */
```

`k` only becomes known *after* `vsTransformDataInit` runs, so the maps cannot be built there. A
`lensEnsureMaps(td)` guard at the top of each transform entry point builds them on first use and
rebuilds when `k` changes.

Sources of `k`, in priority order:

1. `conf.lensK` — manual override, `0` means unset. NaN is deliberately **not** used as the
   sentinel: both CMakeLists build with `-ffast-math`, under which `isnan()` may fold to `0` and
   `NaN == x` may return true, so a NaN sentinel in a public header is a trap. Nothing is lost —
   forcing `k = 0` and disabling correction are the same operation, and "no correction" is spelled
   `lensCorrection = Off`.
2. The estimate stashed into `td` by `vsLocalmotions2Transforms`.
3. `vsTransformSetLensK(VSTransformData*, double k)` — a new public setter for consumers that do not
   share one `VSTransformData` across the two passes.

The handover assumed ffmpeg's `vf_vidstabtransform.c` reuses a single `VSTransformData`, which would
make source 2 sufficient. That is unverifiable from this repository, so source 3 exists regardless
and is the documented contract.

`lensActive` reuses the estimator's guard: `lensMode != Off && estimate.determined && |k| > 0.01`.

### 3.5 The transforms-file path

`src/serialize.c:561` parses `"%i %lf %lf %lf %lf %i"` and carries no `k`. The file format is **not**
extended. That path degrades to `lensActive = 0` unless the user supplies `conf.lensK`. Consistent
with the handover's recommendation to keep `k` a clip-global value on `VSTransformData` rather than
per-`VSTransform`; `VSTransform.barrel` stays diagnostic-only.

## 4. Config surface

In `VSTransformConfig`, alongside the existing `estimateLensDistortion`:

```c
VSLensCorrectMode lensCorrection;  /* default VSLensCorrectWobble */
double            lensK;           /* 0 -> use the estimate */
```

`estimateLensDistortion` stays a separate flag and keeps defaulting on. Estimating `k` to get better
transforms and rewarping the picture are different choices; `lensCorrection = Off` with estimation
on remains a meaningful configuration.

No `lensFovPolicy` (the handover's crop/fit choice). Crop is the only policy: fit produces black
wedges because a radial expansion does not preserve a rectangle, and it costs linear resolution.
If someone wants fit later, they can zoom out through the existing `zoom` option.

## 5. Order of work

Each step is independently testable and leaves the tree green.

1. **`k = 0` bit-exactness guard.** Pin current output for a set of transforms, interpolation types
   and pixel formats before touching anything. Every later step keeps it passing.
2. **`src/lensmap.{c,h}` plus its tests.** The unit and the LUT in isolation: `gU`/`gD` against
   direct computation, `D_k ∘ U_k = id` to round-off, LUT error in `g` and in implied pixel
   displacement, domain-failure reporting. No render path touched yet.
3. **Float planar, luma only, wobble mode.** `VSTransformData` fields, `lensEnsureMaps`, the
   insertion in `_FLT(transformPlanar)`, the early-out condition. This is where the geometry is
   proven — tests 6.1, 6.2 and 6.3 below.
4. **All planes, with the 4:2:2 handling.** Adds test 6.5.
5. **Fixed-point planar and the LUT there.** Adds test 6.4.
6. **`transformPacked`, both paths.** Same insertion, no subsampling.
7. **Zoom budget** via `vsTransformRequiredZoom`, wired into `optZoom`. Adds test 6.6.
8. **Full mode.** Mostly a mode flag by this point; adds tests 6.7 and 6.8.
9. **Config surface, `vsTransformSetLensK`, docs.** Update `docs/lens-distortion.md` and retire
   `docs/lens-correction-handover.md`.

## 6. Test plan

Reusable assets in `tests/test_lensdistortion.c`: `ldRenderWarped` (renders a frame through `D_k` and
a similarity — the forward direction needed to *create* distorted input), `ldFillTexture`,
`ldSampleBilinear`, `ldGenerate`. Style references: `test_transform.c`, `test_interpolate.c`, and
`test_simd_equivalence.c` for the equivalence pattern.

1. **`k = 0` is bit-identical** to pre-change output, across modes. The primary regression guard.
2. **Wobble with identity `M` is bit-identical** to the uncorrected output, for several `k`. This is
   the property the default rests on; if it ever fails, the default is wrong.
3. **The wobble claim itself — the money test.** Synthesise a still scene shot by a moving camera:
   render a textured frame through `ldRenderWarped` at known `k` under a sequence of similarities.
   Feed the exact inverse transforms with wobble correction on. Assert the corrected outputs are
   pixelwise stable frame to frame. Report both the residual and the uncorrected baseline, so the
   test states the size of the improvement rather than only that a threshold held.
4. **Fixed-point versus float equivalence**, 1-2 LSB, over several `k`, modes and interpolation
   types.
5. **Chroma alignment** on YUV420P *and* YUV422P. The 4:2:2 case is the one that catches §2.3;
   without it that bug ships silently.
6. **Zoom budget**: no border pixel is ever sampled, over a sweep of `k` × transform × mode; and the
   eight-point sample matches a dense boundary sweep.
7. **Full mode: straight lines become straight.** Render a grid, distort, correct, fit lines to the
   grid rows and columns, measure maximum deviation.
8. **Wobble mode: straight lines stay curved, identically in every frame.** The inverse of 7, and
   the direct statement of "the lens look survives".
9. **Pincushion borders** under full mode: `k > 0` produces black corners unless zoomed; confirm the
   `def` handling does the right thing rather than sampling garbage.

## 7. Known limits, carried forward

- **Peripheral softness in full mode.** `dD/dr < 1` at the periphery (≈0.71 at `r = 1`, `k = -0.25`),
  so the outer region is magnified and softens. Bicubic helps. Documented, not forced — an automatic
  interpolation change would be a surprising side effect of a mode flag. Wobble mode is largely
  spared, since its net warp is small wherever the stabiliser moved little.
- **Zoom lenses.** `k` is one value per clip. Footage that zooms during the shot has a `k` that
  genuinely drifts, and the estimator will return a confident average. Out of scope; the hook for
  later is a windowed estimate.
- **`Full` remains opt-in** until it has been run over a range of real footage. Flipping that default
  is a separate decision on evidence, not part of this work.

## 8. File and symbol map

| What | Where |
|---|---|
| Render-path lens map, LUT | `src/lensmap.{c,h}` (new) |
| Model, estimator, fit | `src/lensdistortion.{c,h}` (unchanged) |
| Maths write-up | `docs/lens-distortion.md` |
| Estimate hook | `src/localmotion2transform.c:39` `vsLocalmotions2Transforms` |
| Float warp loops | `src/transformfloat.c:291` `_FLT(transformPlanar)`, `transformPacked` |
| Fixed-point warp loops | `src/transformfixedpoint.c` `transformPlanar`, `transformPacked` |
| Fixed-point helpers | `src/transformfixedpoint.c:39-56` |
| Zoom budget | `src/transformtype.c:299`; new `vsTransformRequiredZoom` in `src/transform.c`; callers `src/transform.c:456-491` |
| Config structs | `src/transform.h:84` `VSTransformConfig`, `:123` `VSTransformData` |
| Transforms file parser | `src/serialize.c:561` |
| Tests | `tests/test_lensdistortion.c`, new `tests/test_lensmap.c`, `tests --testLENS` |
| Consumer | out of tree: ffmpeg `vf_vidstabtransform.c` |

Build and run:

```
cmake -S tests -B build/tests && cmake --build build/tests -j8
build/tests/tests --all
build/tests/tests --testLENS
```
