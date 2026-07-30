# Synthetic pixel-format test videos/frames for the stabilizer

## Problem

The stabilizer test suite (`tests/`) has no systematic, per-pixel-format coverage
using clearly interpretable synthetic content. `tests/generate.c` generates noise +
random rectangles in a single fixed format (`PF_YUV420P`), which is good for stress
testing but hard to reason about or visually inspect. We want deterministic,
visually-inspectable test content — a uniform background with circles that move with
a clean, known camera motion, plus a variant with extra squares that move
independently in random (but reproducible) directions as distractors — generated
across a representative set of pixel formats, with test cases that verify
`vsMotionDetection` correctly recovers the injected camera motion.

## Goals

- Generate synthetic frame sequences (uniform background + moving circles) across
  6 representative `VSPixelFormat` values: `PF_GRAY8`, `PF_YUV420P`, `PF_YUV422P`,
  `PF_YUV444P`, `PF_RGB24`, `PF_RGBA`.
- A second sequence variant additionally contains a few squares of a third color that
  move independently, in random-but-reproducible directions, as motion-detection
  distractors.
- Both sequences use a deterministic "clean" global camera motion (translation +
  rotation + zoom), reusing the existing `getTestFrameTransform(i)` pattern from
  `tests/generate.c` / `tests/test_motiondetect.c`.
- New unit tests (following the existing `UNIT()`/`test_bool()` framework) that, for
  each pixel format, run `vsMotionDetection` over the generated sequence and assert
  the recovered transform matches the injected transform within tolerance.
- An optional on-disk dump of generated frames as viewable PPM (P6/RGB) images, for
  manual visual inspection, analogous to the existing `--store` flag in `tests.c`.
- Implementation happens in an isolated git worktree/branch.

## Non-goals

- Exhaustive coverage of all 11 `VSPixelFormat` values (BGR24, YUV410P, YUV411P,
  YUV440P, YUVA420P are skipped — they exercise the same planar-subsampling or
  packed-byte-order code paths as formats already covered).
- Producing muxed video container files (e.g. `.avi`/`.y4m`); frame sequences (raw
  in-memory `VSFrame`s, optionally dumped as PPM) are sufficient.
- Replacing or modifying the existing noise/rectangle generator in `generate.c`.

## Design

### 1. Pixel-format-agnostic pixel helpers (`tests/testutils.h`, `tests/testutils.c`)

New functions, format-aware via `VSFrameInfo`:

- `void setPixelRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y, uint8_t r, uint8_t g, uint8_t b);`
  Converts the given RGB color into the frame's native pixel format and writes it at
  `(x, y)`. For planar YUV formats, converts RGB→YUV and writes luma every pixel;
  chroma only at each format's subsampled chroma-plane position (so writes at
  non-representative subsample positions are no-ops or last-write-wins — acceptable
  since drawn regions are solid-colored blocks larger than one chroma cell). For
  packed formats (`RGB24`, `BGR24`, `RGBA`), writes bytes directly in the correct
  channel order (alpha, if present, is always written opaque).
- `void getPixelRGB(const VSFrame* frame, const VSFrameInfo* fi, int x, int y, uint8_t* r, uint8_t* g, uint8_t* b);`
  Inverse conversion, used only by the PPM dump path.
- `void fillFrameRGB(VSFrame* frame, const VSFrameInfo* fi, uint8_t r, uint8_t g, uint8_t b);`
  Fills the whole frame with one color (uniform background).
- `void paintCircleRGB(VSFrame* frame, const VSFrameInfo* fi, int cx, int cy, int radius, uint8_t r, uint8_t g, uint8_t b);`
  Filled disc via a simple squared-distance test.
- `void paintSquareRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y, int size, uint8_t r, uint8_t g, uint8_t b);`
  Filled axis-aligned square (reuses the geometry of the existing `paintRectangle`,
  but goes through `setPixelRGB` so it works on any format).
- `int storePPMImage(const char* filename, const VSFrame* frame, const VSFrameInfo* fi);`
  Converts the frame to RGB via `getPixelRGB` and writes a binary PPM (P6). Modeled
  on the existing `storePGMImage`.

All new pixel access is coordinate-wise (not the fastest), which is fine — this code
only runs during test-fixture generation, not in any performance-sensitive path.

### 2. Synthetic sequence generators (new `tests/generate_synthetic.c`)

Included from `tests/tests.c` alongside the other `test_*.c`/`generate.c` includes.

```c
#define SYN_WIDTH 320
#define SYN_HEIGHT 240
#define SYN_NUM_FRAMES 6      // frames 0..5, consistent with getTestFrameTransform's range
#define SYN_NUM_CIRCLES 5
#define SYN_NUM_SQUARES 4
#define SYN_SEED 12345

// Frame 0: uniform gray background + SYN_NUM_CIRCLES filled circles of a contrasting
// color at fixed, spread-out positions/radii. Frames 1..N-1: each is produced by
// warping the previous frame with vsTransformPrepare/vsDoTransform/vsTransformFinish
// using getTestFrameTransform(i) -- the same mechanism generateFrames() in generate.c
// already uses, so it is known-correct for every pixel format (vsDoTransform
// dispatches planar vs. packed internally).
void generateCircleFrames(VSFrame* frames, VSFrameInfo* fi, VSPixelFormat pf,
                           int width, int height, int numFrames);

// Same base sequence as generateCircleFrames, but after each frame i (i>=1) is
// warped, SYN_NUM_SQUARES squares of a third color are stamped on top. Each square
// gets one random-but-fixed (seeded) velocity vector at generation time and moves
// linearly frame-to-frame, independent of the camera-shake transform, clamped to
// stay within frame bounds. Represents independently-moving foreground distractors
// the stabilizer should not lock onto.
void generateCircleSquareFrames(VSFrame* frames, VSFrameInfo* fi, VSPixelFormat pf,
                                 int width, int height, int numFrames);

// Writes frames[0..numFrames) as prefix000.ppm, prefix001.ppm, ... via storePPMImage.
void dumpFramesAsPPM(const VSFrame* frames, const VSFrameInfo* fi, int numFrames,
                      const char* prefix);
```

Both generators `vsFrameInfoInit` + `vsFrameAllocate` each frame internally and are
responsible for cleanup being the caller's job (mirrors existing `generateFrames`
convention: caller owns `frames[]` and frees via `vsFrameFree`).

Colors: background mid-gray, circles bright (high luma delta, and distinct hue where
chroma exists), squares a third distinct hue. Exact values tuned during
implementation so contrast is sufficient for `vsMotionDetection`'s block-matching
across all 6 formats, including `PF_GRAY8` (chroma-blind).

### 3. Tests (new `tests/test_synthetic.c`)

```c
void test_synthetic_circles(void);        // --testSYN
void test_synthetic_circles_squares(void); // --testSYNSQ
```

Each iterates the 6 target pixel formats. For each format:

1. Generate the sequence (`SYN_NUM_FRAMES` frames).
2. For each consecutive frame pair `(i-1, i)`, run `vsMotionDetection` and convert the
   resulting `LocalMotions` to a `VSTransform` via `vsSimpleMotionsToTransform`
   (same as `test_motiondetect.c`).
3. Compare against `getTestFrameTransform(i)` using the existing
   `sub_transforms`/`mult_transform_` helpers, asserting
   `fabs(diff.x) < 2 && fabs(diff.y) < 2 && fabs(diff.alpha) < 0.005` via `test_bool`.
4. On failure, print the format name and the diff (mirrors existing diagnostics).

`test_synthetic_circles_squares` uses the same assertion logic against the same
`getTestFrameTransform`, since the squares are distractors and the dominant global
motion (large uniform background + circles) should still win; tolerance may be
loosened during implementation if empirically needed for a particular format, but
starts identical to the circles-only test.

Both are wired into `tests/tests.c`'s `main()` the same way as existing `--testXX`
flags, and included in `--all`.

### 4. Optional PPM dump

A new `--dumpSynthetic` flag in `tests.c`: when present, both generators run once per
format and dump their frames via `dumpFramesAsPPM` to
`tests/testdata/synthetic/<format-name>/{circles,circles_squares}NNN.ppm`. This is
for manual visual inspection only, not part of `--all`, and these directories are
`.gitignore`d (generated, not checked in).

### 5. Build wiring

`tests/CMakeLists.txt`: no changes needed — `tests.c` already `#include`s its test
`.c` files directly rather than compiling them as separate translation units, so
`generate_synthetic.c`/`test_synthetic.c` just need `#include` lines added to
`tests.c`.

### 6. Isolation

Implementation happens in a dedicated git worktree on a new branch
(`feature/synthetic-pixel-tests` or similar), per user request, so it doesn't disturb
the current working tree (which already has unrelated build artifacts/scratch files
present).

## Testing

- Build the `tests` target and run `./tests --testSYN --testSYNSQ` (and `--all`) to
  confirm the new units pass alongside the existing suite.
- Run with `--dumpSynthetic` locally and manually eyeball a few PPM frames per format
  to confirm circles/squares are visible and moving as expected (not part of CI/`--all`,
  just a manual sanity check during implementation).
