#!/usr/bin/env bash

PORT="/dev/${1-ttyUSB0}"
SPEED="115200"

stty -F "${PORT}" "${SPEED}" raw -clocal -echo
