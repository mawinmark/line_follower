# Test tools (not installed with the package)

Nothing here talks to the real robot. Simulations run on ROS domain 201+ with
`ROS_LOCALHOST_ONLY=1`; `run_sim.sh` refuses domain 107.

| file | what it does |
|---|---|
| `run_sim.sh` | one simulated run: `DOMAIN=201 TRACK=u UR=0.3 ./run_sim.sh ccw 50 /tmp/out` |
| `sim_world.py` | 2D robot + camera simulator (track `rect` or `u`, camera footprint, delays) |
| `sim_driver.py` | starts `line_follower.py`, presses Start, prints every action change |
| `replay.py` | feeds frames of a recorded bag through the real node, prints its decisions |

Record a real test for analysis:

```bash
ros2 bag record -o ~/test_curveN /image_raw/compressed /cmd_vel /odom
```

Simulator defaults are calibrated to the real robot with the old (high) camera:
image delay 0.35 s, cmd delay 0.15 s, motor time constant 0.35 s, 10 fps,
camera sees the floor from 0.32 m (±4.5 cm) to 1.2 m (±17 cm) ahead.
The camera was lowered in Nov 2026; its new footprint has not been measured yet.
