#!/usr/bin/env sh

set -e

# source the venv
. venv/bin/activate

python3 src/pcg/main.py $@
