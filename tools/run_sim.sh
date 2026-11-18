#!/bin/bash
# Run one simulated test on an isolated ROS domain (never the real robot's domain).
#   DOMAIN=201 TRACK=u UR=0.3 ./run_sim.sh ccw 50 /path/out_prefix
# Extra env vars are passed to sim_world.py / sim_driver.py (see their headers).
DIR=${1:-ccw}; DUR=${2:-50}; OUT=${3:-/tmp/sim}
HERE=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=${DOMAIN:-201} ROS_LOCALHOST_ONLY=1
if [ "$ROS_DOMAIN_ID" = "107" ]; then echo "refusing to use the robot's domain"; exit 1; fi
T0=$(python3 -c 'import time; print(time.time() + 3)')
python3 "$HERE/sim_world.py" "$T0" "$DUR" "$DIR" "$OUT" > "$OUT.world.log" 2>&1 &
python3 "$HERE/sim_driver.py" "$T0" "$HERE/../line_follower/line_follower.py" "$DUR" \
    2>&1 | grep -v '^\[INFO\]' > "$OUT.driver.log"
wait
cat "$OUT.driver.log"
grep RESULT "$OUT.world.log" || tail -20 "$OUT.world.log"
