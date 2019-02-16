#!/usr/bin/env bash

PORT_DEF="$(find /dev/tty.usbmodem* 2> /dev/null | sed 's/\/dev\///g')"
PORT="/dev/${1-$PORT_DEF}"

SPEED="115200"

# stty -F "${PORT}" "${SPEED}" raw -clocal -echo

screen "${PORT}" "${SPEED},cs8"
