#!/usr/bin/env bash


TEST_FILE="../samples/test001.mp4"
OUT_FILE="stabilised.mp4"
#OPTS="=interpol=bicubic"
rm -fv ${OUT_FILE}
time ffmpeg -i ${TEST_FILE} -vf vidstabtransform${OPTS} ${OUT_FILE}
