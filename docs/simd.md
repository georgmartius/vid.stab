# SIMD and parallelism in vid.stab

What the library vectorizes today, how a kernel is chosen at run time, how to
build and test the kernels, and what they measure on two very different x86
parts.

This is a description of the current state.

## What is vectorized

Only the motion detection stage has hand-written SIMD, because that is where
the time is. Two kernels dominate it:

| Kernel | What it does | Share of frame time |
|---|---|---|
| `compareSubImg` | SAD of a field between the previous and current frame, called once per candidate offset | ~63% |
| `contrastSubImg1` | min/max contrast of a field, used to pick the measurement fields | ~5% |

`compareSubImg` is called ~594k times per 1080p frame (a full spiral scan over
all fields and offsets), so it is the only thing worth writing four times.

The blur (`src/boxblur.c`) has no intrinsics but is parallelised with OpenMP and
restructured so the compiler can auto-vectorize it. The transform stage has no
intrinsics either; it is parallelised over destination rows and its backward
map is stepped rather than recomputed per pixel — see "transform" below, which
also says what SIMD would be worth there and why it has not been written.

| File | Contents |
|---|---|
| `src/motiondetect_opt.c` | scalar C reference + the SSE2 kernels |
| `src/motiondetect_avx2.c` | AVX2 kernels, compiled with `-mavx2` |
| `src/motiondetect_avx512.c` | AVX-512 (F+BW+VL) kernels |
| `src/motiondetect_neon.c` | NEON kernels for ARM/AArch64 |
| `src/cpudetect.c` | CPUID/XCR0 probe, `VIDSTAB_SIMD` override |
| `src/motiondetect_dispatch.c` | picks the function pointers at run time |

## Runtime dispatch

`vs_simd_init()` sets two function pointers (`compareSubImg`,
`contrastSubImg1`) once, from `vs_cpu_flags()`. Their initial values are the
scalar C versions, so a build with no SIMD at all is still correct.

Order of preference: **AVX2 > NEON > SSE2 > scalar**. AVX-512 is compiled and
tested but never chosen automatically — see below.

Detection is a real CPUID probe including XCR0, so the OS actually saving the
wide registers is verified, not assumed. `motiondetect_dispatch.c` and
`cpudetect.c` are deliberately compiled with baseline flags only: a file built
with `-mavx2` lets the compiler emit AVX2 anywhere in it, including in code
that runs *before* the check.

Override with an environment variable, which caps the detected level:

```
VIDSTAB_SIMD=none|sse2|avx2|avx512|neon
```

This is how the kernels are compared against each other in CI, how the tables
below were produced, and the escape hatch for anyone whose hardware disagrees
with the default. An unrecognised value logs a warning and is ignored.

The selected kernel is printed at info level (`SIMD: AVX2`) and available as
`vs_simd_active_name()`.

### Two properties this buys, and one it costs

* A distribution binary can use AVX2 without being unrunnable on an SSE2-only
  host. Previously SSE2 was selected by the preprocessor and `-msse2` applied
  to the whole library, so a shipped binary could never use anything newer.
* Field size is rounded up to a multiple of 16 **unconditionally**, not only in
  SIMD builds. It has to be: the kernel is picked at run time, so making the
  field geometry depend on the host CPU would make the same input produce
  different `.trf` output on different machines.
* The cost is an indirect call per kernel invocation. At ~594k calls/frame this
  is measurable but small compared to the kernel body itself.

## Build

`CMakeModules/VidstabSimd.cmake` compile-probes each extension and adds the
kernel file with its own `COMPILE_OPTIONS` (`-mavx2`, `-mavx512f -mavx512bw
-mavx512vl`, `/arch:AVX2` under MSVC), plus a `VS_HAVE_*` define for the whole
library. NEON needs no flag on AArch64, where it is mandatory. The configure
step prints what went in:

```
-- SIMD kernels compiled in: SSE2;AVX2;AVX-512
```

A kernel that the compiler cannot build is simply absent; the dispatcher then
never selects it.

## Correctness

All five implementations (C, SSE2, AVX2, AVX-512, NEON) return **bit-identical**
results, and this is enforced rather than hoped for. `tests/test_simd_equivalence.c`
walks every compiled-in kernel over five field sizes, several field positions
and a grid of offsets, comparing against the C reference — 1320 checks on
AArch64, similar counts elsewhere.

Bit-identity constrains the implementation: the early-exit check in
`compareSubImg` stays at every row, even though batching it every 4 rows is
legal under the documented contract, because that would make early-exited calls
diverge between kernels. It was measured at ~1.5% end to end — not worth giving
up an exactly testable property.

`tests/neon_emu.h` is a scalar emulation of the nine NEON intrinsics the kernel
uses, so `motiondetect_neon.c` compiles and runs through the equivalence suite
on x86. It proves the logic, not the code generation; the real check is CI:

| CI job | Covers |
|---|---|
| `ubuntu-latest` x86-64 | scalar / SSE2 / AVX2 dispatch, full suite per level |
| `ubuntu-24.04-arm` | native AArch64 NEON — the real validation of the NEON kernel |
| armv7 under qemu-user | the 32-bit reduction fallbacks (`vs_haddq_u32`, `vs_hminq_u8`, `vs_hmaxq_u8`), which AArch64 never compiles |
| MSVC | Windows build, scalar and SSE2 dispatch |

Both benchmark steps in CI are informational (`continue-on-error`) and assert
nothing: a shared runner is not a measurement environment. What is readable
there is the *ratio* between dispatch levels within one run.

## Measurements

Two machines, five years and two vendors apart. Both use
`bench/bench_motiondetect.c` with synthetic textured frames, GCC `-O3`, best of
three, `VIDSTAB_SIMD` to select the level.

### Reference machine — Intel i7-7800X (Skylake-X, 6c/12t, 3.5 GHz), GCC 13.3

Kernel level, ns/call, 1080p frame:

| size | C | SSE2 | AVX2 | AVX-512 |
|---|---|---|---|---|
| compare 32 | 139.4 | 82.0 | **49.5** | 60.2 |
| compare 48 | 190.3 | 141.7 | **79.1** | 90.2 |
| compare 112 | 755.5 | 610.1 | 455.4 | **444.4** |
| contrast 32 | 808.1 | 53.6 | 40.9 | **40.5** |
| contrast 48 | 1748.9 | 101.8 | **55.5** | 63.0 |
| contrast 112 | 9750.8 | 455.7 | 346.4 | **195.8** |

End to end, ms/frame, before the SIMD work vs. now (9 threads):

| Resolution | Threads | Before | Now | Speedup |
|---|---|---|---|---|
| 1280x720 | 9 | 11.85 | 5.91 | 2.01x |
| 1920x1080 | 9 | 32.34 | 17.86 | 1.81x |
| 3840x2160 | 9 | 305.90 | 202.24 | 1.51x |
| 1920x1080 | 1 | 68.94 | 49.42 | 1.39x |

### Current machine — AMD Ryzen 9 9900X (Zen 5, 12c/24t, up to 5.66 GHz), GCC 15.2, 2026-08-09

Kernel level, ns/call, 1080p frame:

| size | C | SSE2 | AVX2 | AVX-512 |
|---|---|---|---|---|
| compare 32 | 48.4 | 39.3 | **28.9** | 43.8 |
| compare 48 | 86.0 | 86.4 | **39.9** | 42.0 |
| compare 112 | 273.1 | 272.5 | 192.9 | **186.4** |
| contrast 32 | 712.7 | 29.0 | 18.0 | **16.7** |
| contrast 48 | 1598.6 | 62.3 | 39.7 | **26.2** |
| contrast 112 | 6708.7 | 224.7 | 148.8 | **79.0** |

End to end, ms/frame, best of three, 19 threads (the library uses 0.8 x
`omp_get_max_threads()`):

| Resolution | scalar | SSE2 | AVX2 | AVX-512 |
|---|---|---|---|---|
| 1280x720 | 2.69 | 2.10 | **2.04** | 2.16 |
| 1920x1080 | 6.88 | 5.74 | **4.43** | 4.92 |
| 3840x2160 | 59.79 | 54.47 | **44.98** | 48.26 |
| 1920x1080, 1 thread | 28.06 | 27.87 | 23.21 | **21.63** |

1080p motion detection runs at ~225 fps here (~4.4 ms/frame), against 56 fps on
the reference machine.

Things worth noting from the two tables:

* **The scalar C row is not really scalar.** GCC auto-vectorizes `compareSubImg`
  at `-O3`, which is why plain C is within 25% of SSE2 on Zen 5 and beats it at
  size 48. `contrastSubImg` does not auto-vectorize (the min/max reduction over
  bytes defeats it) and stays 25-40x behind — that is the kernel where
  intrinsics buy the most in relative terms, even though it is only ~5% of the
  frame.
* **AVX-512 wins single-threaded on Zen 5 and loses multi-threaded.** 21.6 vs
  23.2 ms/frame on one thread (7% faster), 4.92 vs 4.43 with all cores (11%
  slower, stable over five repeats). With 19 threads on 12 cores the stage is
  bandwidth bound and the extra vector width has nothing left to do, while the
  wider loads compete for the same L2/L3 traffic. This is the opposite shape of
  the Skylake-X result, where AVX-512 lost in both modes for a different reason
  (frequency licensing).
* **Speedups compress as resolution grows** on both machines — 4K is memory
  bound long before it is compute bound.

### AVX-512 is not the default, on purpose

On Skylake-X, AVX-512 is ~18% slower than AVX2 end to end in both threading
modes: these kernels are memory bound, so the extra width buys little, while
issuing 512-bit instructions drops the core clock on parts with AVX-512
frequency licensing — and that penalty applies to the whole frame, including
the scalar code around the kernel.

Zen 5 has no such licensing, and the numbers above show it: AVX-512 is the
fastest option there *when the machine is not already bandwidth saturated*.
That is exactly the problem. The best choice depends on the microarchitecture
**and** on the thread count and resolution, there is no runtime probe for "does
512-bit width cost me frequency here", and a family/model table ages badly. So
the kernels are built and tested and left one environment variable away:

```
VIDSTAB_SIMD=avx512
```

Worth setting if you stabilize single-threaded, or on a machine with more
bandwidth per core than this one.

## boxblur

`boxblur_hori`/`boxblur_vert` were ~10.5 ms/frame at 1080p and entirely serial —
the `#pragma omp parallel for` was commented out with "(no speedup)". True or
not when written, it became the Amdahl limit once the motion search scaled.

Current state: `hori` is parallelised across rows (each row is an independent
accumulator chain). `vert` was rewritten from a column walk — a new cache line
per pixel, one serial dependency chain per column, nothing vectorizable — to a
row-sequential sweep holding all columns' accumulators in an array, then
parallelised over blocks of columns.

Both passes then divide by `size` once per pixel. `size` is a runtime value, so
that is a real integer division — ~20-30 cycles in `hori`'s serial chain, and in
`vert` it blocked the output loop from vectorizing at all, since no SIMD unit
has an integer divide. It is now an exact magic-number reciprocal,
`(acc*mul) >> shift` with `mul = ceil(2^shift/size)`.

The constants are *checked*, not assumed. With `acc <= 255*size` the product
must stay in 32 bits, which caps `shift` at 24; exactness then needs
`255*size*(size-1) < 2^24`, i.e. it holds for every odd size up to 265 and
fails first at 267. So `vs_reciprocal()` searches for a shift satisfying both
conditions and reports failure if there is none, in which case both passes keep
the division. (`boxblurPlanar` only ever passes stepSize-derived sizes, far
inside the range.)

Measured on the Ryzen 9 9900X, 1080p, `size=15` (`bench/bench_boxblur.c`):

| | 1 thread | 24 threads |
|---|---|---|
| `boxblur_hori` | 2.24 -> 1.15 | 0.22 -> 0.11 ms/frame |
| `boxblur_vert` | 2.34 -> 0.44 | 0.88 -> 0.27 ms/frame |

(before -> after the reciprocal; the `vert` output loop now vectorizes, which is
where its 5.3x single-threaded comes from.)

The rewrite is bit-identical to the original including its edge behaviour, and
`tests/test_boxblur.c` checks both passes against a verbatim copy of the old
algorithm over ~1200 geometries, with sizes straddling 265/267 so that the
reciprocal and the division fallback are both exercised. (It used to be a
timing harness that asserted nothing.)

## transform

The warp had never been parallelised — the one stage in the library that still
ran on a single core, even after the motion search and the blur had been
scaled. Every destination row is independent (it writes only its own row and
reads only the source frame and the read-only lens map; `td->src` and
`td->destbuf` can never alias, because `vsTransformPrepare` gives an in-place
caller a private copy), so this was a pragma, not a rewrite.

The other change is arithmetic. Three quantities in the inner loop are
polynomials in x and were being evaluated from scratch at every pixel:

| | why it is a polynomial in x |
|---|---|
| `x_s`, `y_s` | affine on the plain similarity path: `dx` is `x_d1<<16`, whose low bits are zero, so `(zcos_a*dx + zsin_xy*dy)>>16` splits exactly into `zcos_a*x_d1 + (zsin_xy*dy>>16)` |
| `r2u` | wobble's undistort radius — its input is the destination pixel itself, so it is a quadratic |
| `r2d` | the distort radius, a quadratic wherever `x_s`/`y_s` are affine: the plain path, but not behind wobble (non-linear) or fov (projective) |

A quadratic is generated exactly by two running additions, so six 64-bit
multiplies per pixel became six 64-bit adds. All integer, so these are the same
value sequences *to the bit* — `tests/test_transform_incremental.c` checks all
three against verbatim copies of the expressions they replace, over every pixel
of eight geometries including 4:2:2, 4:4:0 and 4:1:1. The asymmetric formats are
the point: on 4:2:0 both axes subsample equally, so a step constant that
confused them would still pass.

The fov path separately lost three of its four per-pixel divisions. It computed
`z*fFov*X/Z/(1<<lsx)` per axis — two divisions each, three of them by
quantities that never change. Only Z varies, so the constants fold into one
factor per axis and a single reciprocal of Z serves both.

### Measurements — Ryzen 9 9900X (Zen 5, 12c/24t), GCC 15.2

YUV420P, bilinear, ms/frame, best of three (`bench/bench_transform.c`).
"before" is the state prior to this work; every frame hash is unchanged
throughout, so all of it is bit-identical.

1080p:

| mode | 1 thread before → after | 24 threads before → after | total |
|---|---|---|---|
| lens=off | 11.3 → 8.9 | 10.5 → 0.96 | **10.9x** |
| lens=full | 20.7 → 16.5 | 20.6 → 1.55 | **13.3x** |
| lens=wobble | 31.3 → 29.3 | 31.0 → 2.54 | **12.2x** |
| fov=90 | 19.8 → 16.2 | 18.9 → 1.62 | **11.7x** |
| fov=90 lens=full | 32.5 → 29.5 | 33.4 → 2.82 | **11.8x** |

After, by thread count — 12 threads is one per physical core, 24 adds SMT:

| mode | 1080p 1 / 12 / 24 | 4K 1 / 12 / 24 |
|---|---|---|
| lens=off | 8.9 / 1.00 / 0.96 | 41.6 / 4.34 / 3.95 |
| lens=full | 16.5 / 1.59 / 1.55 | 69.4 / 6.25 / 6.34 |
| lens=wobble | 29.3 / 2.85 / 2.54 | 119.5 / 10.93 / 10.02 |
| fov=90 | 16.2 / 1.60 / 1.62 | 66.4 / 6.27 / 5.49 |
| fov=90 lens=full | 29.5 / 2.82 / 2.82 | 120.9 / 11.95 / 11.16 |

SMT buys almost nothing: 12 threads is within ~10% of 24 everywhere, and at 4K
`lens=full` and `fov=90` are inside the noise of each other. The stage is
bandwidth bound well before it runs out of cores, so on a 12-core part expect
essentially the 24-thread column.

Relative cost of the optional stages, 1080p / 24 threads: wobble 2.7x the plain
warp, full 1.6x, fov 1.7x. **Wobble costs more than full**, which looks backwards
until you count LUT stages: full undistorts once, wobble undistorts *and*
redistorts. At 4K the ratios are much the same (2.5x / 1.6x / 1.4x).

### What SIMD would be worth, and why it is not written

`VS_Zero` keeps the identical address arithmetic and makes the interpolation
nearly free, so benchmarking a mode against its nearest-neighbour twin splits
the per-pixel cost in two. 1080p, one thread:

| mode | bilinear | nearest | interpolation | addressing |
|---|---|---|---|---|
| lens=off | 9.1 | 4.3 | 4.8 | 4.3 |
| lens=full | 16.5 | 8.7 | 7.8 | 8.7 |
| lens=wobble | 29.2 | 20.3 | 8.9 | **20.3** |

The two modes are not the same problem. With the lens off the split is about
even, so a vectorized bilinear kernel would be the thing to write. With wobble
on, 70% of the time goes on deciding *where* to sample — two dependent LUT
lookups — and vectorizing the interpolator alone would cap out at ~30%.

Both halves are gathers. The bilinear fetch wants `_mm256_i32gather_epi32`
twice (two rows x 8 pixels, each 32-bit word carrying both horizontal
neighbours); the lens stage wants a gather into `gU`/`gD`, which at 1024 x
int32 = 4 KB sits in L1 and should gather well. A 2-3x on the addressing half
looks reachable.

It is not written because the scalar work above landed first and changed the
arithmetic: at 24 threads 1080p now warps in 0.9-2.6 ms/frame, against 4.4 ms
for motion detection on the same machine, so the transform is no longer the
stage that decides throughput. The measurements are here so that the case can
be re-made if it becomes one — at 4K, or on a machine with fewer cores.

### Caching wobble's undistort scale — measured, and not worth it

Wobble's undistort scale `g` depends on the destination pixel and k but **not**
on the per-frame transform, so it can be computed once for a clip and looked up
thereafter: two running adds and a `gU` lookup become one sequential int32
load. This was built behind `VIDSTAB_WOBBLE_CACHE=1` and measured against the
identical frame hash, then removed. Wobble row only, ms/frame:

| | 1 thread | 12 threads | 24 threads |
|---|---|---|---|
| 1080p, no cache | 30.7 | 2.86 | 2.54 |
| 1080p, cached | 25.8 | 2.42 | 2.12 |
| | −16% | −16% | −17% |
| 4K, no cache | 123.8 | 11.74 | 10.11 |
| 4K, cached | 104.0 | 9.95 | 9.25 |
| | −16% | −15% | −8% |

So ~16%, falling to 8% at 4K on all cores where the table stops fitting
anywhere useful. In absolute terms on a multi-core machine that is 0.4 ms at
1080p and 0.9 ms at 4K, and it takes wobble from 2.5x the plain warp to 2.25x.

The price is a resident table of 4 bytes per destination pixel — 12 MB at
1080p, 50 MB at 4K — plus a lifecycle to get wrong: invalidate on k, mode or
geometry change, and per-instance storage, which means a field in a public
struct and an SOVERSION bump. Not worth it for 16%, so it is not in the tree.

Two things to note if this is ever revisited. It scales the wrong way — the
benefit shrinks exactly where the stage is most expensive, because a 50 MB
table streamed per frame is competing for the bandwidth the warp already
wants. And it only addresses the *undistort* half; the distort half depends on
the transform and cannot be cached at all, which is why the ceiling here is far
below the full addressing cost in the table above.

One thing that was tried and rejected: calling `interpolateBiLin` directly
instead of through `td->interpolate`, which the source had recommended for
years ("inlining the interpolation function would bring 10%"). Measured, it is
consistently *slower* — 20.7 → 23.2 ms on lens=full, and slower at both thread
counts — because the indirect call predicts perfectly while the inlined body
costs I-cache and registers in a loop that is already register-hungry. The
comment has been corrected in place.

## Reproducing

```sh
sh bench/build.sh -DVS_HAVE_SSE2 -msse2     # -> bld/bench
./bld/bench 1920 1080 30                    # kernels + end to end
VIDSTAB_SIMD=avx512 ./bld/bench 1920 1080 30
OMP_NUM_THREADS=1 ./bld/bench 1920 1080 20
```

`bench/build.sh` compiles the wide kernels with their own `-m` flags exactly as
CMake does, and builds the NEON kernel against `tests/neon_emu.h` so it can be
run on x86 — its timings are the emulation's, not NEON's, and mean nothing.

The kernel table only times kernels the *CPU* can execute (`vs_cpu_flags()`),
since a build machine's compiler routinely supports more than its processor.

## Known limits

* **No controlled ARM benchmark.** The NEON kernel is correctness-verified on
  real AArch64 hardware, but no speed claim is made for it. CI timings are a
  sanity check that NEON beats scalar, not a figure to quote.
* `boxblur_hori` is still a serial running sum within a row (parallel across
  rows). A prefix-sum formulation would vectorize it.
* `USE_SSE2_ASM`, the historical hand-written assembly variant, is outside the
  runtime dispatch: it ignores `linesize2` and is only correct when both frames
  share a stride. Reachable only through an explicit opt-in build.
* **The next real gain is algorithmic.** `compareSubImg` is within ~2x of memory
  bandwidth for its access pattern, so further speedup is more likely to come
  from *doing fewer comparisons* — a coarse-to-fine pyramid, or early
  termination at the field level — than from making each one faster.
