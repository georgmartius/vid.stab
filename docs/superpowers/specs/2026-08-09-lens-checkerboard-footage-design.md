# Checkerboard lens footage and the pixel round trip

Status: design, approved 2026-08-09.
Branch: `feature/lens-checkerboard-footage`, off `feature/lens-distortion-estimation`.

## Why

`test_lensdistortion.c` works entirely on synthetic *point matches* and
`test_lensmap.c` on the map's numbers. Neither ever renders a picture, so
nothing in the suite answers the question a reader actually asks: put a
distorted frame in, does the corrected frame come back straight?

This adds the missing image-level test, and the same generator produces the
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
  centred on the frame centre, with a band period of 60 px in radius. Half the
  frame diagonal is 400 px, so roughly six band boundaries cross the picture.

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
  `transformfloat.c:263`:
  `S_i(x) = z * R(-alpha_i) * (x - c) + c - t_i`.

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
- **Flat pixels** — those where the ideal image's local gradient is below a
  threshold — must satisfy `max |delta| <= 2`.
- **Whole valid mask**: PSNR above a floor, so a gross geometric error in the
  textured regions still fails even though it cannot be caught per pixel.
- **Anti-vacuity control.** The same clip run with `lensCorrection = Off` must
  score dramatically worse against the base. Without this the whole test would
  pass with the lens map stubbed out to a no-op.

The gradient threshold and the PSNR floor are fixed during implementation, to
the tightest values that pass with a clear margin, and the margin actually
measured is recorded in a comment next to each. A threshold loose enough that
the `Off` control would also pass it is a bug in the test.

Tight numerical guarantees on the map itself stay in `test_lensmap.c`. This
test's job is end-to-end pixel sanity.

### Coverage beyond the figure

- The same round trip on `PF_YUV420P` (assertions only, no dumps). The
  subsampled chroma planes are where a per-plane map is most likely to be
  wrong.
- A second pass at pincushion `k = +0.15` (assertions only, no dumps).

## Outputs

Into `testout/lensclip/`, via the existing `storePPMImage()`:

- `base_NNN.ppm`, `distorted_NNN.ppm`, `full_NNN.ppm`, `wobble_NNN.ppm`
- `lensclip_sheet.ppm` — one contact sheet, base | distorted | Full-corrected
  side by side for frame 0. This is the illustration; it is the only artefact
  that exists for documentation rather than for the test.

PPM because `storePPMImage()` already exists and a PNG encoder would mean a
new zlib dependency. Convert for documents with `pnmtopng` or `convert`.

## Wiring

`generate_lensclip.c` is `#include`d from `tests.c` alongside
`generate_synthetic.c`; the test file likewise, registered as `--testLCR`.
All dumping is gated behind the existing `--store` flag, so an ordinary
`make test` writes nothing.

## Out of scope

- Estimating `k` from the clip. `k` is handed to the render path directly, so
  a failure means the render path is wrong and cannot be blamed on detection
  drift. A repeating checkerboard is in any case a poor scene for a block
  matcher.
- Varying `k` across the clip. The render path assumes one `k` per clip.
- PNG output.
