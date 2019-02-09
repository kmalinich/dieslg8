#!/usr/bin/env bash

FILE_INPUT="${1}"

for x in 0 1 2 3 4 5 6 7 8 9 A B C D E F; do
	sed -i "s,\ ${x}\ ,\ 0${x}\ ,g" "${FILE_INPUT}"
	sed -i "s,\ ${x}$,\ 0${x},g"    "${FILE_INPUT}"
done
