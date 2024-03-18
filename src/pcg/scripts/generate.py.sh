#!/usr/bin/env sh

# source the venv
. venv/bin/activate

# handle sigint and sigterm; kill python proc
control_c() {
  kill -9 $(ps aux | grep "[p]ython3 src/pcg/main.py" | awk '{print $2}')
  exit
}

trap "control_c" INT TERM

python3 src/pcg/main.py "$@" &
# capture PID of command to kill when this program is killed
PID=$!
wait "$PID"
