#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
python3 px4_motor_dashboard_readonly.py "$@"
