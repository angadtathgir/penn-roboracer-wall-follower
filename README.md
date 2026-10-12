# Penn RoboRacer Technical Challenge: LiDAR Wall Follower

A ROS 2 node that drives the AutoDRIVE RoboRacer around the IROS 2026 practice track (Porto) from its LiDAR scan.

**Fastest lap: 6.140 s.** This is the simulator's own lap timer, running `ros2 run autodrive_roboracer wall_follow_node`.

**Video (Driver's Eye, unlisted):** _link added after upload_

Consistency check with the real node: 33 laps across three runs. The car never touched a wall while driving. Over 30 flying laps, the best was 6.140 s, the mean 6.311 s and the worst 6.466 s. The standing-start first lap takes about 7.6–7.9 s.

## Running it

Follow the challenge setup (Docker container, bridge, simulator connected, Driving Mode set to Autonomous), then:

```bash
ros2 run autodrive_roboracer wall_follow_node
# any controller parameter can be overridden, e.g.
ros2 run autodrive_roboracer wall_follow_node --ros-args -p mode:=wall -p v_max:=6.0
```

## How it works

The code is split so the driving logic can be tested without ROS:

- `autodrive_roboracer/wall_follow_core.py` holds the controller. It is plain numpy: a scan goes in, and throttle and steering come out.
- `autodrive_roboracer/wall_follow_node.py` is the ROS 2 wrapper. It subscribes to `/lidar` (plus `/odom` for measured speed), runs the controller once per scan, and publishes `throttle_command` and `steering_command`. Every controller setting is a ROS parameter.

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

Throttle is a feed-forward term (target / 24.9 m/s) plus proportional feedback on measured speed. On top of that sits **traction control**: wheel speed is capped at `v·(1 + 0.15) + 0.4`.

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
| Wider search plus a small aim-point shift toward the inside of turns | **6.12 s** in the runner, **6.140 s** with the ROS node |

Things that didn't work:

- Planning for more grip (`a_lat` ≥ 9) made the car slide wide and lose time.
- Aiming toward the *outside* of corners for a late apex was slower at every offset I tried, and crashed above 0.3 m.

For a sense of the remaining headroom: a forward/backward speed-profile estimate on the track centreline (30.5 m, tightest radius 1.07 m) gives 6.42 s at 7 m/s² of grip. The reactive controller beats the centreline by cutting corners.

## Repository layout

```
autodrive_devkit/
  autodrive_roboracer/wall_follow_core.py   controller (no ROS)
  autodrive_roboracer/wall_follow_node.py   ROS 2 node
  setup.py                                  adds the wall_follow_node entry point
  tools/                                    sysid, lap runner, tuner, lap monitor, results
```

`autodrive_bridge.py` is unchanged from the AutoDRIVE repository.
