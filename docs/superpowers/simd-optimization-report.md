# SIMD and parallelism optimization of the motion detection stage

Experimental branch `worktree-simd-opt`. Measurements on an Intel i7-7800X
(Skylake-X, 6c/12t, 3.5 GHz, AVX-512 capable), GCC 13.3, `-O3`, synthetic
textured frames from `bench/bench_motiondetect.c`.

## Summary

| Resolution | Threads | Before (ms/frame) | After (ms/frame) | Speedup |
|---|---|---|---|---|
| 1280x720  | 9 | 11.85 | 5.91 | **2.01x** |
| 1920x1080 | 9 | 32.34 | 17.86 | **1.81x** |
| 3840x2160 | 9 | 305.90 | 202.24 | **1.51x** |
| 1920x1080 | 1 | 68.94 | 49.42 | **1.39x** |

1080p motion detection goes from 31 fps to 56 fps on this machine.

Four changes, in order of how much they bought:

1. **AVX2 kernels with runtime dispatch** for `compareSubImg` and
   `contrastSubImg` (`compareSubImg` alone was 66% of frame time).
2. **boxblur parallelised** across cores — it was ~10 ms/frame of purely serial
   work and had become the Amdahl limit once the search itself scaled.
3. **boxblur vertical pass restructured** from a column walk to a row-sequential
   sweep, so it stops missing cache on every pixel and auto-vectorizes.
4. **ARM/Apple Silicon got SIMD at all** — see below, it had none.

## The ARM finding

This is the one worth reading even if the rest is skipped.

`motiondetect_opt.c` contained:

```c
#ifdef __ARM_NEON__
#include "sse2neon.h"
#define USE_SSE2
#endif
```

That `#define` is local to that one translation unit. `motiondetect.c` — which
is where `compareSubImg` is actually *called*, and where `fieldSize` is rounded
up to a multiple of 16 — never saw it. Verified with the preprocessor:

```
$ gcc -E -Isrc x.c | grep WHICH     # no -DUSE_SSE2, i.e. an ARM build
WHICH: compareSubImg_thr                 <- the scalar C version
```

So on every ARM build, including all Apple Silicon: the shimmed kernels were
compiled and then never called, and **vid.stab ran fully scalar**. There is a
second, independent bug in the same place — `__ARM_NEON__` (with trailing
underscores) is defined by Clang but *not* by GCC on AArch64, so a GCC AArch64
build did not even reach the shim.

Replaced with native NEON kernels (`src/motiondetect_neon.c`) using
`vabdq_u8`/`vpadalq_u8`, wired through the runtime dispatcher, with the
architecture detection in one place (`cpudetect.h`) checking both spellings.

**Correctness is now verified on real hardware.** It was not when this was
first written — no ARM machine, cross-compiler or qemu was available on the
development box — so `tests/neon_emu.h` was added: a scalar emulation of the
nine NEON intrinsics the kernel uses, letting it compile on x86 and run through
the equivalence suite there. CI then closed the gap properly:

- **aarch64**, native `ubuntu-24.04-arm` runner: selects `SIMD: NEON` and
  passes 1320/1320 equivalence checks against the C reference on real silicon.
- **armv7 under qemu-user**: the only place the 32-bit reduction fallbacks
  (`vs_haddq_u32`, `vs_hminq_u8`, `vs_hmaxq_u8`) execute at all, since AArch64
  compiles the `vaddvq_`/`vminvq_` path instead. Also 1320/1320.

Still **not benchmarked** on ARM: no speed claim is made for the NEON kernel,
only correctness. qemu timings are meaningless and the CI runner is not a
measurement environment.

## Runtime dispatch

Previously SSE2 was selected by the preprocessor and `-msse2` was applied to
the whole library, so a distribution binary could never use anything newer.
Now `src/cpudetect.c` probes CPUID (including XCR0, so the OS is actually
saving the wide registers) and `src/motiondetect_dispatch.c` picks kernels at
runtime. Only the AVX2/AVX-512 *kernel files* are compiled with `-mavx2` etc.,
so the binary still runs on an SSE2-only host.

`VIDSTAB_SIMD=none|sse2|avx2|avx512|neon` overrides the choice; that is how the
table below was produced on one machine, how the kernels are tested against
each other in CI, and how the CI benchmark step compares dispatch levels
back to back on one runner.

Field size is now rounded to a multiple of 16 **unconditionally** rather than
only in SSE2 builds. That matters: the kernel is chosen at runtime, so making
the field geometry depend on the host CPU would make the same input produce
different `.trf` output on different machines.

## Kernel measurements

Single call, `field->size` as shown, 1080p frame, ns/call:

| size | C | SSE2 | AVX2 | AVX-512 |
|---|---|---|---|---|
| compare 32  | 139.4 | 82.0 | **49.5** | 60.2 |
| compare 48  | 190.3 | 141.7 | **79.1** | 90.2 |
| compare 112 | 755.5 | 610.1 | 455.4 | **444.4** |
| contrast 32  | 808.1 | 53.6 | 40.9 | **40.5** |
| contrast 48  | 1748.9 | 101.8 | **55.5** | 63.0 |
| contrast 112 | 9750.8 | 455.7 | 346.4 | **195.8** |

All five kernels (C, SSE2, AVX2, AVX-512, NEON) return **bit-identical**
results. The early-exit check cadence was left at every row precisely so that
this holds even for early-exited calls; batching the check every 4 rows is
legal under the documented contract and was measured at only ~1.5% end to end,
which is not worth giving up an exactly-testable property for.

Note the C row for `compare` is not really "scalar" — GCC auto-vectorizes it at
`-O3 -msse2`, which is why the old SSE2 kernel was only ~1.3x faster than it.

## AVX2 lost to SSE2 on the GitHub x86 runner — unexplained

Worth recording because it did not show up on dedicated hardware. The CI
benchmark step (720p, best of three) on `ubuntu-latest`:

```
none    219.82 ms/frame
sse2     57.65 ms/frame
avx2     70.59 ms/frame     <- 22% slower than SSE2
avx512   71.11 ms/frame
```

Reproducible across runs and across best-of-three sampling, so it is not
runner noise — but it is the opposite of every measurement on the i7-7800X,
where AVX2 beats SSE2 at every field size tested.

Not diagnosed. Two plausible causes, neither confirmed:

- At 720p the field size is 80 bytes, so the AVX2 kernel does two 32-byte
  iterations plus a 16-byte tail, and the tail carries widening overhead that
  SSE2's uniform five iterations do not. That is a third of the row on a
  less favourable path.
- 256-bit frequency licensing on the runner's server part, which the
  128-bit SSE2 kernel would avoid entirely.

The dispatch default is set from dedicated-hardware measurement, not from
this, and correctness is unaffected (the kernels are bit-identical and the
equivalence tests pass on that runner). But it does mean **AVX2 should not be
assumed to be a win on every x86 part**, and it is a reason to keep the
`VIDSTAB_SIMD` override available to users. Worth a proper look on a known
server CPU before treating the AVX2 default as settled for server deployments.

## AVX-512 is not enabled by default, on purpose

On this Skylake-X, AVX-512 is consistently **~18% slower** than AVX2 end to
end, single- and multi-threaded (49.7/50.3/50.7 vs 59.3/59.4/61.0 ms/frame over
three runs each). These kernels are memory bound, so the extra width buys
little, while issuing 512-bit instructions drops the core clock on the parts
that implement AVX-512 frequency licensing — and that penalty applies to the
*whole* frame, including the scalar code around the kernel.

Newer cores (Ice Lake+, Zen 4/5) do not throttle this way and would likely come
out ahead, but there is no reliable runtime probe for "does 512-bit width cost
me frequency here", and a family/model table ages badly. So the AVX-512 kernels
are built and tested but reachable only via `VIDSTAB_SIMD=avx512`.

This is worth revisiting on a Zen 4/5 or Ice Lake+ machine — it may well
deserve to be the default there.

## boxblur

`boxblur_hori` + `boxblur_vert` were ~10.5 ms/frame at 1080p and, unusually,
entirely serial: the `#pragma omp parallel for` in `hori` was commented out
with "(no speedup)". That may have been true when written, but once the motion
search scales across 9 cores, 10 ms of serial work dominates everything.

- `hori`: rows are independent accumulator chains — parallelised directly.
- `vert`: rewritten from "one column at a time" (a new cache line per pixel,
  and one serial dependency chain per column, so nothing vectorizes) to a
  row-sequential sweep holding all columns' accumulators in an array, then
  parallelised over blocks of columns.

Result: **10.45 -> 2.59 ms/frame** at 9 threads (1.13x from the restructure
alone, the rest from parallelism).

The rewrite is bit-identical to the original, including its edge behaviour;
`tests/test_boxblur.c` now checks that against a verbatim copy of the old
algorithm over 500 geometries. Previously `test_boxblur` was a timing harness
that asserted nothing (`0/0 checks`).

## Where the time goes now

1080p, single-threaded profile after the changes:

```
63.1%  compareSubImg_thr_avx2
10.8%  boxblur_vert_C
 9.9%  boxblur_hori_C
 5.4%  contrastSubImg
```

`compareSubImg` is still the kernel to beat, but it is now within ~2x of
memory bandwidth for its access pattern. Further meaningful gains are more
likely to come from **doing fewer comparisons** (the search is a full spiral
scan over ~594k calls/frame at 1080p) than from making each one faster —
e.g. a coarse-to-fine pyramid, or early termination on the field level.

## What is not done

- **No controlled ARM benchmark.** The NEON kernel is correctness-verified on
  real aarch64 hardware, and CI prints timings per dispatch level, but a shared
  CI runner is not a measurement environment: treat those numbers as a sanity
  check that NEON beats scalar, not as a figure to quote.
- `boxblur_hori` is still a serial running sum per row (parallel across rows,
  but scalar within one). A prefix-sum formulation would vectorize it.
- The `acc[i]/size` division in `boxblur_vert` blocks full auto-vectorization
  of the output loop (runtime divisor). A magic-number reciprocal would fix it;
  provably exact for `size <= 4096` with a 32-bit multiply, which covers every
  geometry the caller can produce, but it was not needed to hit the numbers
  above.
- `USE_SSE2_ASM` is left out of the runtime dispatch: it ignores `linesize2`
  (pre-existing TODO in `motiondetect_opt.c`) and so is only correct when both
  frames share a stride.
- `src/sse2neon.h` (9199 lines) is now unused and could be deleted.
