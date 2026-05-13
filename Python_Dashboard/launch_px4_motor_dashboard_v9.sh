#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

if ! python3 -c "import pymavlink" >/dev/null 2>&1; then
  echo "pymavlink not found. Installing user-local dependency..."
  python3 -m pip install --user -r px4_motor_dashboard_requirements_v9.txt
fi

# Intended MAVProxy command:
# mavproxy.py --master=/dev/ttyACM0,57600 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14560
exec python3 px4_motor_dashboard_v9.py --connect udpin:0.0.0.0:14560 "$@"
