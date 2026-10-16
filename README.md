# Penn RoboRacer Technical Challenge: LiDAR Wall Follower

A ROS 2 node that drives the AutoDRIVE RoboRacer around the IROS 2026 practice track (Porto) from its LiDAR scan. It comes in two versions that share the same controller:

| Version | Command | Sensors | Fastest lap | Mean flying lap | Wall contacts |
|---|---|---|---|---|---|
| **LiDAR + odometry** | `ros2 run autodrive_roboracer wall_follow_node` | LiDAR for steering, odometry for speed | **6.104 s** | 6.383 s | 0 in 31 laps |
| **LiDAR only** | `ros2 run autodrive_roboracer wall_follow_node_lidar` | LiDAR only (speed from scan matching) | **6.180 s** | 6.390 s | 0 in 31 laps |

All times are from the simulator's own lap timer with the real ROS nodes. Each version drove 31 laps in a row (1 standing start + 30 flying laps). The worst flying lap was 6.984 s with odometry and 7.062 s LiDAR-only. Raw lap lists are in `tools/results/final_*_31laps.txt`.

**Videos (Driver's Eye, unlisted):**
- LiDAR + odometry: https://youtu.be/lLkm8X2-dO0
- LiDAR only: https://youtu.be/S1rFRY-QsEI

## Running it

Follow the challenge setup (Docker container, bridge, simulator connected, Driving Mode set to Autonomous), then run one of:

```bash
ros2 run autodrive_roboracer wall_follow_node         # LiDAR + odometry
ros2 run autodrive_roboracer wall_follow_node_lidar   # LiDAR only, no odometry subscription
# any controller parameter can be overridden, e.g.
ros2 run autodrive_roboracer wall_follow_node --ros-args -p mode:=wall -p v_max:=6.0
```

## How it works

The code is split so the driving logic can be tested without ROS:

- `autodrive_roboracer/wall_follow_core.py` holds the controller. It is plain numpy: a scan goes in, and throttle and steering come out.
- `autodrive_roboracer/wall_follow_node.py` is the ROS 2 wrapper, with one entry point per version. It subscribes to `/lidar` (plus `/odom` for measured speed in the odometry version), runs the controller once per scan, and publishes `throttle_command` and `steering_command`. Every controller setting is a ROS parameter.
- `autodrive_roboracer/lidar_speed.py` estimates the car's speed from the LiDAR alone, for the LiDAR-only version.

### Steering

There are two modes.

- **`wall`** is the classic two-wall follower. For each side it fits a line to the wall from two beams (90° and 90° − θ). It projects the car's distance to that wall forward by a lookahead plus the latency × speed, then uses PD control toward the middle of the corridor.
- **`gap`** (the default and the faster one) is a follow-the-gap controller with a *disparity extender*:
  - Wherever two neighbouring beams differ by more than a threshold, the nearer range is copied over the beams the car's half width would sweep. That way an aim point can never clip a corner.
  - The car aims at the middle of the deepest free window and steers to it with pure-pursuit geometry (κ = 2 sin α / L).
  - A small sideways shift of the aim point and a push-off from close side walls round it out.

Both modes share the same speed planner.

### Speed

The target speed is the smallest of four limits:

| Limit | Formula | Why |
|---|---|---|
| Top speed | `v_max` | |
| Grip | `√(a_lat / κ)` from the steering command | the tyres have finite cornering grip |
| Stopping room | `√(2 a_brake d)`, with *d* the free distance **along the arc being steered**, inside a car-width band | the car must be able to stop before anything on its actual path |
| Corner entry | `√(v_corner² + 2 a_entry d_ahead)`, with *d_ahead* the wall dead ahead | the car must be slow enough for the next corner *before* it starts turning |

Throttle is a feed-forward term (target / 24.9 m/s) plus proportional feedback on speed. Two rules sit on top:

- **Traction control:** wheel speed is capped at `v·(1 + 0.15) + 0.4`, so the tyres don't spin.
- **Hard braking:** once the car is more than 0.4 m/s over its target speed (1.0 m/s in the LiDAR-only version, see below), the throttle drops to 0. In this simulator that locks the wheels, which brakes about 8.5 m/s². A throttle just under the car's speed only drags the motor.

### Speed without odometry (LiDAR-only version)

Both rules above need the car's speed. The LiDAR-only version estimates it from two sources and fuses them:

1. **Scan matching:** the current scan is aligned to one taken about 20 ms earlier with point-to-line ICP (iterative closest point). Points are paired by beam angle, which is cheap and accurate because the car only moves a few centimetres between the two scans. The forward shift divided by the time gap is the speed.
2. **Throttle model:** throttle sets wheel speed, so the car's speed follows the last command with limited acceleration and braking.

The model predicts every scan. The scan-match result corrects it in proportion to how well the walls constrain forward motion. Walls facing the car give a confident correction; two parallel side walls give almost none, and there the model carries the estimate. Against odometry, the estimate is off by 0.17 m/s on average over a lap. The two versions end up with practically the same lap times.

The estimate is noisier than odometry (about ±0.27 m/s on the straights) and reads a little high while braking. With the 0.4 m/s brake threshold, that noise kept crossing the threshold, and the car tapped its brakes on the straights: it braked for 16.7% of the lap, against 12% with odometry. The LiDAR-only version therefore brakes hard only once it is 1.0 m/s over target. That cut braking to about 9% of the lap at the same lap times within run-to-run noise (best 6.180 s against 6.115 s, mean 6.390 s against 6.360 s). The logs are in `tools/results/brake_study/`, and `tools/brake_events.py` counts the brake events in a lap log.

## How it was tuned

Most of the speed came from measuring the simulator rather than guessing:

1. **System identification** (`tools/sysid.py`, `tools/accel_test.py`):
   - Throttle sets *wheel speed*: 0.1 settles at 2.49 m/s, so 1.0 ≈ 24.9 m/s.
   - Throttle 0 locks the wheels and is the hardest brake. It took 4.3 → 1.5 m/s in 0.95 m, about 8.5 m/s².
   - Usable lateral grip is about 7 m/s². Planning with 9+ m/s² made laps slower, and 10+ crashed.
2. **Real-sim lap runner** (`tools/run_laps.py`): it resets the car, drives N laps with the controller and reports lap times and wall contacts.
3. **Parallel search** (`tools/tune.py`): three headless simulators, each with its own container, port and ROS domain, run random and local parameter searches. A config only scores if *every* lap is clean, and the score is its best flying lap. All raw results are in `tools/results/`.

What moved the lap time:

| Step | Result |
|---|---|
| First working version, 4 m/s cap, free distance measured straight ahead | wall 10.57 s / gap 13.35 s (standing laps) |
| Free distance measured along the steering arc, not straight ahead (the car had been braking for walls it was about to turn away from) | gap 13.0 → 10.4 s |
| Random search over 60 gap-follow configs | 6.59 s flying lap, but only 15/60 configs clean |
| Corner-entry speed limit (most crashes were at the hairpin after the long straight) | 6.51 s, and 45/60 configs clean over 3 laps |
| Traction control (0 → 7 m/s took 2.36 s at full throttle and 1.10 s with slip capped at 15%) | 6.28 s, and 61/66 configs clean over 3 laps |
| Wider search plus a small aim-point shift toward the inside of turns | 6.12 s in the runner, 6.140 s with the ROS node |
| Hard braking (throttle 0 once 0.4 m/s over target) | **6.104 s** with the ROS node (LiDAR + odometry), 6.115 s LiDAR-only |
| LiDAR-only: brake threshold raised to 1.0 m/s (see below) | **6.180 s** LiDAR-only, with half the braking time |

Things that didn't work:

- Planning for more grip (`a_lat` ≥ 9) made the car slide wide and lose time.
- Aiming toward the *outside* of corners for a late apex was slower at every offset I tried, and crashed above 0.3 m.
- A braking planner that looked ahead over a window of the scan was slower (6.39 s) and crashed until it was detuned.
- The classic two-wall mode (`mode:=wall`) with every upgrade above managed 7.32 s at best, and only 14 of 60 tuned configs were clean. Follow-the-gap is the default for that reason.

For a sense of the remaining headroom: a forward/backward speed-profile estimate on the track centreline (30.5 m, tightest radius 1.07 m) gives 6.42 s at 7 m/s² of grip. The reactive controller beats the centreline by cutting corners. On the 27.8 m path the car actually drives, the ideal profile is 5.58 s. Closing that last ~0.5 s needs braking points planned from a map, i.e. a racing line, rather than a reactive wall follower.

## Repository layout

```
autodrive_devkit/
  autodrive_roboracer/wall_follow_core.py   controller (no ROS)
  autodrive_roboracer/wall_follow_node.py   ROS 2 node (both versions)
  autodrive_roboracer/lidar_speed.py        LiDAR-only speed estimate
  setup.py                                  adds wall_follow_node and wall_follow_node_lidar
  tools/                                    sysid, lap runner, tuner, lap monitor, results
```

`autodrive_bridge.py` is unchanged from the AutoDRIVE repository.
