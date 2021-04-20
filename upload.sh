#!/usr/bin/env bash

BOARD="arduino:avr:leonardo"
OUTPUT_DIR="${PWD}/build"

PORT_DEF="$(serialport-list --format json | jq -r '.[] | select(.manufacturer != null) | select(.manufacturer | contains("Arduino")) | .path')"
PORT="${1-$PORT_DEF}"

# Install CAN-BUS Shield library if missing
if ! arduino-cli lib list --format json | jq -r '.[].library.name' | grep -Eq 'CAN\-BUS_Shield'; then
	if ! arduino-cli lib install 'CAN-BUS Shield'; then
		echo "Failed to install missing library, cannot continue"
		exit 1
	fi
fi


rm -rf "${OUTPUT_DIR}"
mkdir -p "${OUTPUT_DIR}"

set -e

arduino-cli compile --fqbn "${BOARD}" --output-dir "${OUTPUT_DIR}" ./
echo

arduino-cli upload --fqbn "${BOARD}" --port "${PORT}" ./
