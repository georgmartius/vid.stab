# Checkerboard lens footage and the pixel round trip

Status: design, approved 2026-08-09.
Branch: `feature/lens-checkerboard-footage`, off `feature/lens-distortion-estimation`.

## Why

`test_lensdistortion.c` works entirely on synthetic *point matches*,
`test_lensmap.c` mostly on the map's numbers -- except for
`test_lensmap_straightness` (pre-dating this branch), which already renders a
distorted grid, corrects it in Full and Wobble mode, and measures how much a
column bends. That test establishes the map straightens lines; it does not
give per-pixel ground truth, does not exercise pose motion or pincushion, and
checks one 640x480 GRAY8 grid, not the packed/planar and 4:2:0 formats the
render path actually ships.

This adds an image-level test with an *analytic* ground truth -- every pixel
of the corrected frame has a known correct colour, not just "straighter than
before" -- covering pose motion, both barrel and pincushion, 4:2:0 chroma
subsampling, and per-pixel tolerances split by what a bilinear tap can and
cannot be expected to get exactly right. The same generator also produces the
figure for `docs/lens-distortion.md`.

The existing synthetic scenes (`generate_synthetic.c`: eight circles on a dark
background) are wrong for this. Circles have no straight lines, so barrel
distortion barely shows; and they are painted into a bitmap and then warped,
which folds a resampling step into the ground truth.

## The pattern

`lensPatternSample(x, y) -> RGB`, an analytic function of continuous image
coordinates. Not a bitmap.

- Checkerboard, 32 px cells. At 640x480 that is 20x15 cells.
- Two tones, both coloured so the PPM dumps work as a figure and the luma
  contrast stays high: dark blue `(25, 35, 90)` and light amber
  `(240, 205, 90)`.
- **Radial inversion bands.** The two tones swap inside every other annulus
  centred on the frame centre, with a band period of **61 px** in radius. Half
  the frame diagonal is 400 px, so roughly six band boundaries cross the
  picture.

  61 rather than a round 60, and the choice is load-bearing. Where a band
  circle runs tangent to a cell line, both terms of the `cell XOR band` flip
  along nearly the same curve and leave a lune of opposite tone whose
  thickness falls as `u^2/2R` — arbitrarily thin, and below the supersampler's
  pitch it renders at a different sub-pixel phase in each frame. At a 60 px
  period the circle `r = 240` is exactly tangent to the cell lines `y = 0` and
  `y = 480` (both multiples of the 32 px cell) at `x = 320`, producing a sliver
  under 0.25 px thick. That is a defect of the pattern, not of anything it is
  used to test: it cost a full diagnosis pass, having first presented as a
  Wobble-mode accuracy failure at exactly the two destination rows
  `y = 18.41` and `y = 461.59` those scene points map to.

  61 is coprime with the 32 px cell, so no multiple of the band period
  coincides with a distance from the frame centre to a cell line anywhere in
  the picture — the first coincidence sits at `r = 976`, far outside it. The
  thinnest in-frame lune becomes about 3 px, twelve times the subsample pitch.
  Any future retune of `LC_CELL` or `LC_BAND` must preserve that property.

The bands are what make the pattern worth more than a plain checkerboard.
A cell boundary is a straight line, and distortion bends it — that reads well
but only away from the centre. A band boundary is a circle, and distortion
moves it purely radially, changing the *spacing* between successive
boundaries: compressed at the edge under barrel, stretched under pincushion.
That signal exists at every radius, including where the line bending is
invisible.

Each output pixel is the mean of 4x4 subsamples of the pattern, so edges are
antialiased rather than stair-stepped.

### Why a function and not a painted frame

Every frame of the clip is evaluated directly at its own final coordinates.
No source bitmap is ever resampled to build the footage, so:

- the clip carries zero generation blur, and
- the ground truth for the round trip is exact — it is `lensPatternSample`
  itself, not "whatever the painter produced".

## The generator — `tests/generate_lensclip.c`

Frame `i` of the clip is

```
frame_i(x) = pattern( S_i( U_k(x) ) )
```

- `U_k` is `vsLensUndistortPoint()` from `lensdistortion.h` — the model, which
  `test_lensdistortion.c` already covers independently.
- `S_i` is the render path's own backward affine, transcribed from
  `transformfixedpoint.c:279-361` (`transformPacked`):
  `S_i(x) = z * R(-alpha_i) * (x - c) + c - t_i`. These tests build with
  `-DTESTING` (`tests/CMakeLists.txt`), which renames the float path's entry
  points to the `_float` suffix (`transformfloat.h:35`) while the
  fixed-point path's stay unqualified, and `vsDoTransform`
  (`src/transform.c:241-243`) calls the unqualified names -- so it is the
  fixed-point path these tests actually exercise. `transformfloat.c`
  implements the same map in floating point; `test_lensmap.c`'s fixed/float
  equivalence tests keep the two paths in step, not this generator.

The generator deliberately does **not** call `vsLensPlaneMapInit()`. Building
the footage with the same lookup tables the test then checks would make the
test circular — it would confirm only that the LUT agrees with itself.

Clip parameters: 6 frames; bounded shake, `|x| <= 12 px`, `|y| <= 9 px`,
`|alpha| <= 1.2 deg`, no net drift, `zoom = 0`; one fixed barrel `k = -0.25`.

## The round trip — `tests/test_lenscorrect_roundtrip.c`

Run the render path with `lensCorrection = VSLensCorrectFull`, `lensK = k`,
and the transform set to `S_i^-1`. Composing that with the generator:

```
out(x) = pattern( S_i( U_k( D_k( S_i^-1(x) ) ) ) ) = pattern(x)
```

Every frame must come back as the undistorted, unshaken base image.

`VSLensCorrectWobble` is checked the same way. Its backward map is
`D_k(M(U_k(x)))`, so with `M = S_i^-1` it lands on `pattern(U_k(x))` — the
distorted base with the shake removed and the lens left in place, which is
exactly what the mode promises.

### What is asserted

Bilinear resampling of a hard edge is off by up to half a step *at that edge*
no matter how correct the geometry is. A flat per-pixel tolerance would
therefore be either vacuous or false, so the comparison is split:

- **Validity mask.** A pixel is excluded when its backward map leaves the
  source frame. Computed exactly from the map, not approximated by an
  arbitrary inset border.
- **Flat pixels** — a pixel qualifies when every pixel of its 7x7
  neighbourhood in the ideal image is byte-identical to it. Exact equality
  rather than a gradient threshold, so there is no magic number to tune:
  inside a checkerboard cell the pattern is constant to the byte, and any
  antialiased edge pixel fails the test outright. (7x7, not smaller: the
  radial derivative of the distortion at the frame corner plus pose motion
  maps a source-space kernel reach to a destination-space extent ~1.76x as
  wide (equivalently, an output neighbourhood draws on a source neighbourhood
  about 0.567x as wide), and source pixels are
  themselves box-averaged over the supersampling window, pushing the true
  support a bilinear tap can draw on out to about 2.6 px -- a half-width of
  3 covers that, a half-width of 2 does not.) Flat pixels must satisfy
  `max |delta| <= cap`.
- **Whole valid mask**: PSNR above a floor, so a gross geometric error in the
  textured regions still fails even though it cannot be caught per pixel.
- **Anti-vacuity control.** The same clip run with `lensCorrection = Off` must
  score dramatically worse against the base. Without this the whole test would
  pass with the lens map stubbed out to a no-op.

The flat-pixel cap and the PSNR floor are fixed during implementation, to the
tightest values that pass with a clear margin, and the margin actually
measured is recorded in a comment next to each. A cap loose enough that the
`Off` control would also pass it is a bug in the test.

There is no single global cap. Four scalar constants and three per-channel
4:2:0 caps cover three groups of cases, because the render paths they check
genuinely differ:

- **Full and pincushion (PF_RGB24)**: `max |delta| <= 2` on every channel.
  Both measure a worst of 1; the extra count is margin confirmed by a
  perturbation test (see `LC_MAX_FLAT_DELTA`'s comment in
  `test_lenscorrect_roundtrip.c`).
- **Wobble (PF_RGB24)**: the same `<= 2` cap, but its own, looser PSNR floor
  (`LC_MIN_PSNR_WOBBLE`) -- Wobble's check is verifying mode semantics (shake
  removed, lens retained), not the same sub-pixel geometry guarantee Full and
  pincushion's floor polices.
- **PF_YUV420P**: a *per-channel* cap, not one shared number. `setPixelRGB`
  writes one pixel's chroma per 2x2 block and `getPixelRGB` reads it back
  nearest neighbour, so chroma is quantised on both sides of the comparison
  in a way the packed path never is, and the RGB reconstruction weights that
  quantised chroma differently per channel -- B carries the largest
  coefficient, G the smallest. Measured worst per channel: R = 7, G = 1,
  B = 25 (5-count frame-to-frame spread). R and G get worst + 1; B gets
  worst plus enough headroom to clear that spread comfortably rather than
  sitting 1 above it. See `LC_MAX_FLAT_DELTA_420_R/G/B` in
  `test_lenscorrect_roundtrip.c` for the exact numbers and their derivation.

Tight numerical guarantees on the map itself stay in `test_lensmap.c`. This
test's job is end-to-end pixel sanity.

### Coverage beyond the figure

- The same round trip on `PF_YUV420P` (assertions only, no dumps). The
  subsampled chroma planes are where a per-plane map is most likely to be
  wrong.
- A second pass at pincushion `k = +0.15` (assertions only, no dumps).

## Outputs

Into `testout/lensclip/`, via the existing `storePPMImage()`:

- `base.ppm`, `distorted_NNN.ppm`, `full_NNN.ppm`, `wobble_NNN.ppm`
- `sheet.ppm` — one contact sheet, base | distorted | Full-corrected
  side by side for frame 0. This is the illustration; it is the only artefact
  that exists for documentation rather than for the test.

The corrected panel is full-bleed, with no border fill. That is a property of
the sign, not an accident: under barrel `D_k` contracts, so every destination
pixel samples strictly inward and the correction crops into the source rather
than running off it — the validity mask reports the entire frame usable on
every frame of this clip. Pincushion is the case that costs field of view,
about 18.5% of the frame at k = +0.15.

PPM because `storePPMImage()` already exists and a PNG encoder would mean a
new zlib dependency. Convert for documents with `pnmtopng` or `convert`.

## Wiring

`generate_lensclip.c` is `#include`d from `tests.c` alongside
`generate_synthetic.c`; the test file likewise, registered as `--testLCR`.
All dumping sits behind its own `--dumpLensClip` flag, following the
`--dumpSynthetic` precedent at `tests/tests.c:238` rather than the `--store`
flag, which is reserved for the raw frame captures at the top of `main`. An
ordinary `make test` therefore writes nothing.

## Out of scope

- Estimating `k` from the clip. `k` is handed to the render path directly, so
  a failure means the render path is wrong and cannot be blamed on detection
  drift. A repeating checkerboard is in any case a poor scene for a block
  matcher.
- Varying `k` across the clip. The render path assumes one `k` per clip.
- PNG output.
