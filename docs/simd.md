# SIMD and parallelism in vid.stab

What the library vectorizes today, how a kernel is chosen at run time, how to
build and test the kernels, and what they measure on two very different x86
parts.

This is a description of the current state. The story of how it got here — the
ARM shim that never dispatched, the AVX2 tail regression that only showed up on
Zen 4 — is in `docs/superpowers/simd-optimization-report.md`.

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
restructured so the compiler can auto-vectorize it; the transform stage uses
fixed point arithmetic and is not SIMD at all.

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

Measured on the Ryzen 9 9900X, 1080p, `size=15` (`bench/bench_boxblur.c`):

| | 1 thread | 24 threads |
|---|---|---|
| `boxblur_hori` | 2.24 | 0.22 ms/frame |
| `boxblur_vert` | 2.34 | 0.89 ms/frame |

The rewrite is bit-identical to the original including its edge behaviour, and
`tests/test_boxblur.c` checks that against a verbatim copy of the old algorithm
over 500 geometries. (It used to be a timing harness that asserted nothing.)

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
* The `acc[i]/size` division in `boxblur_vert` blocks full auto-vectorization of
  the output loop (runtime divisor). A magic-number reciprocal would fix it,
  provably exact for `size <= 4096` with a 32-bit multiply, which covers every
  geometry the caller can produce.
* `USE_SSE2_ASM`, the historical hand-written assembly variant, is outside the
  runtime dispatch: it ignores `linesize2` and is only correct when both frames
  share a stride. Reachable only through an explicit opt-in build.
* **The next real gain is algorithmic.** `compareSubImg` is within ~2x of memory
  bandwidth for its access pattern, so further speedup is more likely to come
  from *doing fewer comparisons* — a coarse-to-fine pyramid, or early
  termination at the field level — than from making each one faster.
