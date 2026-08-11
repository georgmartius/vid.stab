# Open issue triage

Snapshot: 2026-08-07, against `origin/master` @ `b85fa83`. **41 open issues.**

**Update 2026-08-07, after the section A pass: 36 open.** #86, #32, #62, #113
and #22 are answered and closed; the documentation landed in `cb87b19`
(README + new `docs/trf-format.md`). See "Section A — done" below.

Each issue below was checked against the code as it stands, not against what the
reporter saw. Where a claim is unverified that is said explicitly.

Closed during this pass: #95 (ORC removed), #71 (L1 implemented), #65 + #34
(OpenMP/pkg-config linking), #52 (not our bug). #165 and #137 were resolved by
the LGPL relicensing.

---

## A. Easy and mechanical — DONE (`cb87b19`)

All five closed. Three corrections to what this section assumed, found while
doing it:

- **The `--enable-gpl` follow-up is done too**, as a short README note
  explaining that it is ffmpeg's packaging decision (libvidstab sits in
  ffmpeg's `EXTERNAL_LIBRARY_GPL_LIST`), not a requirement of this library.
- **#62 is not just "derive a section from `serialize.c`".** Through ffmpeg you
  now always get the *binary* encoding — `vidstabdetect` exposes no option to
  select it and the library default is binary (`motiondetect.c:117`), so every
  ASCII `.trf` in the old issues predates that change. Also, the binary version
  number is written with `%hhu`, i.e. as decimal *text*, so `TRF1` ends in the
  character `'1'`, not the byte `0x01`. Both verified against a real file.
- **#86 has a live remainder.** The doc bug is fixed, but the reporter's actual
  problem — suppressing rotation estimation for monopod footage — is real, and
  `maxangle=0` does not do it (it clamps the output transform after the path is
  computed, so the rotation still contaminates the translation estimate). That
  now lives with **#110**.

Note for the next pass: `Closes #a, #b, #c` in a commit message only closes the
first. Repeat the keyword, or close the rest by hand.

| # | What | Where |
|---|---|---|
| **86** | README still advertises "Brute force algorithm only for translations". The algorithm is gone: `algo` is marked `// deprecated` in `src/motiondetect.h:50` and unused. Delete the claim. | `README.md:23` |
| **32** | Interlaced video is undocumented. One paragraph: deinterlace first, fields are not handled. Confirmed there is no mention anywhere in `README.md` or `docs/`. | `README.md` |
| **62** | The `.trf` format is undocumented. A short section derived from `src/serialize.c` closes it. | `docs/` |
| **113**, **22** | Both are `PKG_CONFIG_PATH` mistakes at ffmpeg-configure time. `README.md:80` already documents the right invocation, and the `Libs.private` fix (#173) removed the leaking flags. Answer and close. | — |

**Related follow-up, not an issue yet:** `README.md:82` still tells people to
build ffmpeg with `--enable-gpl`. Now that the library is LGPL-2.1-or-later that
line wants revisiting — though ffmpeg's own `configure` still gates the filters
behind `--enable-gpl`, so the instruction stays correct until ffmpeg changes.
Worth a note in the README explaining why.

## B. Verify, then close — recent work probably fixed them

These look resolved by work already on master. None has been re-tested against
the reporter's footage; that is the cheapest next step and could close four.

| # | Why it is probably fixed |
|---|---|
| **124**, **102** | Both are "vid.stab is too slow". Since they were filed, master gained native NEON, AVX2 and AVX512 kernels with runtime dispatch (`src/motiondetect_{neon,avx2,avx512}.c`, `src/cpudetect.c`). #124 in particular is Apple Silicon, which now has a real NEON path rather than a scalar fallback. |
| **79**, **103** | Both are border artifacts — grey edges and a flashing green dash. Commit `902ff92` fixed out-of-bounds reads at the frame border in the float interpolators, which is the right shape of cause. |

## C. Valid bugs, need real investigation

No quick win here; each needs the reporter's clip and a debugging session.

- **127** — one 12-second segment produces jerking; the neighbouring segment is fine. Suggests specific content or timestamps defeating detection.
- **88** — jerking near frame edges when smoothing with pan. Reporter worked around it by smoothing the trajectory externally, which points at the camera-path filter rather than detection.
- **81** — only 4 frames detected from a `-f concat` input. Plausibly a frame-count or timestamp issue at the ffmpeg boundary; worth reproducing first, it may not be ours.
- **55** — flashing concert lights defeat detection. Genuine algorithmic weakness: contrast-based field selection follows the lighting. Would need illumination-invariant matching.

## D. Contained features — small in the library, but need ffmpeg plumbing

Each is a modest change to libvidstab. The real cost is that none is reachable
from ffmpeg without a matching patch to `vf_vidstabdetect` / `vf_vidstabtransform`
upstream. Decide on the ffmpeg side before starting.

- **140** — option to disable zoom estimation (force `zoom = 0` in `vsMotionsToTransform`).
- **50** — `crop=average`: a third `VSBorderType` filling with the frame average. Today only `VSKeepBorder` / `VSCropBorder` exist (`src/transform.h:64`).
- **30** — separate `maxshift` for X and Y. `maxShift` is a single `int` in both `src/motiondetect.h:63` and `src/transform.h:99`, and is used both for field placement and for clamping, so this is the largest of the three.

## E. Support questions — answer and close, no code

**120**, **106**, **89**, **82**, **77**, **76**, **67**, **46**, **44**, **40**, **13**

**Correction: #80 does not belong here.** The thread contains an acknowledged
tripod-mode bug — @btzy traced it to `motiondetect.c` (only `virtualTripod` is
consulted, so the first pass still measures relative to the previous frame) and
Georg confirmed "this looks like a bug". Two reporters hit it independently and
both worked around it. It belongs in section C, and there is test footage
attached in the thread.

Mostly "how do I get better results" or "how do I build this". Two worth a
specific answer rather than a generic one:

- **89** — detect at low resolution, transform at high resolution. Does *not* work as-is: shifts are stored in pixels, so the transforms do not carry across scales.
- **13** — jpeg-sequence input, filed 2014 against an ffmpeg from that year. Stale; ask for a retest on a current build or close outright.

## F. Rather not implement

Real requests, but each is a poor fit for a 2D affine stabilizer or a
disproportionate amount of work. Recommend closing with an explanation rather
than leaving them open indefinitely.

- **139** FoV / 3D deshaking, **63** + **131** rolling-shutter compensation — need a per-scanline motion model plus lens and depth awareness. #131 is effectively a duplicate of #63.
- **123** 10+ bit support — invasive. `unsigned char*` pixel access runs through all of motion detection, both interpolator sets and every SIMD kernel; it would need a duplicated or templated pixel path throughout.
- **25** + **112** manual focus areas / reference points — require interactive UI that a CLI filter has no place for.
- **39** GPU acceleration, **40** mobile port — new backends, not fixes. (#40 can simply be answered: it is portable C99 and does build for Android.)
- **54** memory use on multi-hour clips — would need streaming the transform list instead of loading it whole; a real redesign.
- **110** axis-selective smoothing, **138** border inpainting, **24** blur-based frame dropping, **73** Lanczos/Spline interpolators. #73 is easy to add mechanically, but at sub-pixel offsets it buys ringing more than sharpness.

## G. Meta

- **133** — new maintainer wanted. Still open and still true.

---

## H. Found during the lens-correction work, not from the issue list

Two pre-existing bugs surfaced while building and testing lens distortion
correction (branch `feature/lens-distortion-estimation`). Neither is caused
by that feature -- both reproduce with the lens entirely off/untouched -- and
neither was fixed as part of it; each needs its own deliberate change and its
own test-golden update, which is out of scope for a feature branch. Recorded
here so they don't get lost.

- **Fixed-point and float interpolators disagree systematically, even with
  the lens off.** `interpolateBiLinBorder` in `src/transformfixedpoint.c`
  carries a rounding-bias `+ 1` right before the final round-and-shift:

  ```
  fp16 s   = fp16To8(v1*(x - x_f)+v3*(x_c - x))*fp16To8(y - y_f) +
    fp16To8(v2*(x - x_f) + v4*(x_c - x))*fp16To8(y_c - y) + 1;   // src/transformfixedpoint.c:93
  int32_t res = fp16ToIRound(s);                                 // already rounds to nearest
  ```

  The float twin, `_FLT(interpolateBiLinBorder)` in `src/transformfloat.c`
  (line 50), has no such term -- it truncates: `int32_t res = (int32_t)s;`.
  `interpolateBiLin` (the in-bounds bilinear path, not just the border
  fallback) reaches this code for any near-edge but still-in-bounds pixel,
  since its own border check (`transformfixedpoint.c:172`) falls through to
  `interpolateBiLinBorder` there. Measured directly with the existing test
  harness (`test_lensmap_fixed_float_equivalence`, lens forced off so mode
  and k cannot be a factor): mean absolute difference between the fixed and
  float outputs is **3.7477** for `VS_BiLinear` and **3.8977** for
  `VS_BiCubic` on the hard-edged synthetic texture (`ldFillTexture`, 900
  random rectangles) -- both interpolators that route through
  `interpolateBiLinBorder`/`interpolateBiCubBorder`. `VS_Zero` and `VS_Linear`
  don't carry the term and disagree by 0.02-0.03 mean instead.

  Not fixed here because `tests/test_transform_baseline.c` pins the
  byte-exact output of all four warp loops (fixed and float, planar and
  packed) against golden CRCs, and removing the `+1` would change the fixed-
  point goldens by design -- that is a deliberate, separately-reviewed change
  with its own golden regeneration, not something to fold into a lens
  feature. It was invisible before this work because the baseline test pins
  the fixed and float goldens *separately* and never compares them to each
  other; `test_lensmap_fixed_float_equivalence` is, as far as we found, the
  first test in the suite that does.

- **`src/vsvector.c:200` produces a null-pointer UBSan diagnostic.**
  `vs_vector_concat`:

  ```
  memcpy(result.data, V1->data, sizeof(void*)* V1->nelems);
  memcpy(result.data+V1->nelems, V2->data, sizeof(void*)* V2->nelems);   // :200
  ```

  `vs_vector_init` (`vsvector.c:36`) leaves `V->data == NULL` whenever it is
  called with `buffersize <= 0`, independent of `nelems`. `motiondetect.c:223`
  does exactly that -- `vs_vector_init(&motionsfine, 0)` -- and when a frame's
  coarse pass finds too little contrast to run a fine scan (or the fine scan
  matches nothing), `motionsfine` is never appended to and stays `{data=NULL,
  nelems=0}` all the way to `vs_vector_concat(&motionscoarse, &motionsfine)`
  at `motiondetect.c:287`. The second `memcpy` above then runs with a NULL
  source pointer and a length of 0. That is undefined behaviour by the letter
  of the C standard -- passing NULL to `memcpy` is UB regardless of length --
  which is exactly what `-fsanitize=undefined` is built to catch, and it does.
  In practice this is benign: a zero-length copy touches no memory on every
  mainstream implementation (glibc's `memcpy` checks `n` before touching
  `src`/`dst`), and nothing downstream reads through the resulting pointer
  for more than zero elements. The honest fix is a `n > 0` guard before each
  `memcpy` (`vs_vector_init` should arguably not hand out a NULL `data` for
  an empty-but-valid vector at all, but that would be a wider change than
  this one call site), not a behavioural fix -- there is no incorrect output,
  only a diagnostic. Left alone here because it is unrelated to lens
  correction and the fix belongs with a real UBSan-clean pass over
  `vsvector.c`, not a one-line patch buried in this branch.

## Suggested order

1. ~~Section A — five issues, all documentation, no risk.~~ Done, `cb87b19`.
2. Re-test section B against the reporters' clips — potentially four more closures for the cost of running two commands each.
3. Triage section F by closing with explanations, so the open list reflects what is actually intended.
4. Section C is the only real engineering left, and #127 is the best entry point since a reproducer already exists.
