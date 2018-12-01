#!/usr/bin/env bash
set -x

TEST_FILE="../samples/test001.mp4"
OUT_FILE="stabilised.mp4"
#T_OPTS="=interpol=bicubic"
rm -fv ${OUT_FILE}
rm -fv *.data *.trf
perf record -o detect.data ffmpeg -i ${TEST_FILE} -vf vidstabdetect=${OPTS} -f null -
#perf record -o transform.data ffmpeg -i ${TEST_FILE} -vf vidstabtransform${T_OPTS} ${OUT_FILE}
perf record -o transform.data ffmpeg -i ${TEST_FILE} -vf vidstabtransform${T_OPTS} -f null -   # ~6% speedup
