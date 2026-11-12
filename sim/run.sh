#!/bin/sh
# Host-side check of the firmware logic against a toy buoyancy model (not measured from the real glider).
# Usage: ./run.sh          summary only
#        ./run.sh spike    also inject a pressure glitch
#        ./run.sh x log    print the CSV log
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -Imocks -DSKETCH='"../firmware/seaglider_depth_pid.ino"' -o sim_bin harness.cpp
./sim_bin "$@"
rm -f sim_bin
