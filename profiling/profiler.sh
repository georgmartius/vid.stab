#!/usr/bin/env bash

TEST_FILE="../samples/test001.mp4"
OUT_FILE="stabilised.mp4"

rm -fv *.data *.trf *.mp4
perf record -o detect.data ffmpeg -i ${TEST_FILE} -vf vidstabdetect=${OPTS} -f null -
perf record -o transform.data ffmpeg -i ${TEST_FILE} -vf vidstabtransform ${OUT_FILE}
