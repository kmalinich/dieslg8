#!/usr/bin/env bash

BOARD="arduino:avr:uno"
OUTPUT="dieslg8.hex"
PORT="/dev/${1-ttyUSB0}"

rm -f "${OUTPUT}"

set -e

arduino-cli compile --fqbn "${BOARD}" --output "${OUTPUT}" .
echo
arduino-cli upload --fqbn "${BOARD}" --input "${OUTPUT}" --port "${PORT}"
