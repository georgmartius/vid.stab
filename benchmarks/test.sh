#!/usr/bin/env bash

TEST_FILE="../samples/test001.mp4"
OPTS="shakiness=10"

time ffmpeg -i ${TEST_FILE} -vf vidstabdetect=${OPTS} -f null -
