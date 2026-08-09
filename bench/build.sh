#!/bin/sh
# Build the benchmark. Extra compiler flags can be passed as arguments.
set -e
cd "$(dirname "$0")/.."
SRCS="src/frameinfo.c src/transformtype.c src/libvidstab.c src/transform.c \
src/transformfixedpoint.c src/motiondetect.c src/motiondetect_opt.c \
src/serialize.c src/localmotion2transform.c src/boxblur.c src/vsvector.c \
src/l1campathoptimization.c src/lpsolver_ipm.c src/cpudetect.c \
src/motiondetect_dispatch.c"

# The wide kernels get their own -m flags, exactly as CMake does per source.
mkdir -p bld
gcc -O3 -std=gnu99 -Isrc -c src/motiondetect_avx2.c   -o bld/mdavx2.o   -DVS_HAVE_AVX2 -mavx2 "$@"
gcc -O3 -std=gnu99 -Isrc -c src/motiondetect_avx512.c -o bld/mdavx512.o -DVS_HAVE_AVX512 \
    -mavx512f -mavx512bw -mavx512vl "$@"
# NEON kernel, built for x86 against the scalar emulation so it can be checked here
gcc -O3 -std=gnu99 -Isrc -Itests -c src/motiondetect_neon.c -o bld/mdneon.o \
    -DVS_HAVE_NEON -DVS_NEON_EMULATION "$@"
OBJS="bld/mdavx2.o bld/mdavx512.o bld/mdneon.o"
EXTRA_DEFS="-DVS_HAVE_AVX2 -DVS_HAVE_AVX512 -DVS_HAVE_NEON"
mkdir -p bld
gcc -O3 -std=gnu99 -DUSE_OMP -fopenmp -DUSE_IPM -DVS_HAVE_LPSOLVER \
    $EXTRA_DEFS -Isrc -Itests -o bld/bench bench/bench_motiondetect.c $SRCS $OBJS -lm "$@"

# The transform benchmark. -DTESTING keeps the float implementation under its
# own name, so both can be timed (and compared) in one binary.
gcc -O3 -std=gnu99 -DUSE_OMP -fopenmp -DUSE_IPM -DVS_HAVE_LPSOLVER -DTESTING \
    $EXTRA_DEFS -Isrc -Itests -o bld/bench_transform bench/bench_transform.c \
    $SRCS src/transformfloat.c $OBJS -lm "$@"
