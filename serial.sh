#!/usr/bin/env bash

PORT="/dev/${1-tty.usbmodem14201}"
SPEED="115200"

stty -F "${PORT}" "${SPEED}" raw -clocal -echo
