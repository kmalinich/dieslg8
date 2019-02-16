#!/usr/bin/env bash

BOARD="arduino:avr:uno"
OUTPUT="dieslg8.hex"

PORT_DEF="$(find /dev/tty.usbmodem* 2> /dev/null | sed 's/\/dev\///g')"
PORT="/dev/${1-$PORT_DEF}"

rm -f "${OUTPUT}"

set -e

arduino-cli compile --fqbn "${BOARD}" --output "${OUTPUT}" .
echo
arduino-cli upload --fqbn "${BOARD}" --input "${OUTPUT}" --port "${PORT}"
