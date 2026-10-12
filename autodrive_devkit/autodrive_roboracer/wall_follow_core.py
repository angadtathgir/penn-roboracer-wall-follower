"""LiDAR-only reactive racing controller (no ROS imports, so it can be tested offline).

Two steering modes share one speed planner:

* ``wall``: classic two-wall follower. Fits a line to each wall from two beams, projects
  the car forward by a lookahead distance, and steers with PD control toward the centre
  of the corridor.
* ``gap``: disparity-extender follow-the-gap. Inflates every range discontinuity by the
  car's half width so that the chosen heading never clips a corner, then aims at the
  deepest free beam with pure-pursuit geometry.

Speed comes from two physical limits:
  - lateral grip:   v <= sqrt(a_lat / |curvature|)
  - stopping room:  v <= sqrt(2 * a_brake * (free distance ahead - margin))
and a throttle loop: feed-forward (v / v_full_throttle) plus P feedback on measured speed.
"""
import math

import numpy as np

try:
    from autodrive_roboracer.lidar_speed import LidarSpeed
except ImportError:  # loaded by file path (offline harness)
    from lidar_speed import LidarSpeed

MAX_STEER = 0.5236   # rad at steering command 1.0
WHEELBASE = 0.324    # m
RANGE_MAX = 10.0     # m


class Controller:
    def __init__(self):
        self.params = {
            'mode': 'gap',
            'speed_source': 'odom',    # 'odom' = measured speed, 'lidar' = LiDAR-only estimate
            # gap follower
            'fov_deg': 63.474,           # only aim at beams within +/- this angle
            'half_width': 0.231,        # car half width + margin used to inflate edges (m)
            'disparity': 0.399,         # range jump that counts as an edge (m)
            'lookahead_max': 1.415,      # cap on the pure-pursuit lookahead (m)
            'depth_tol': 0.017,         # beams within this of the max depth are candidates
            'apex_offset': -0.1,        # aim-point shift toward the outside of turns (m); <0 = inside
            'apex_alpha': 0.2,         # aim angle (rad) at which that shift is ~76% applied
            # wall follower
            'wall_theta_deg': 50.0,    # angle between the two beams used per wall
            'wall_lookahead': 0.8,     # project the car this far ahead (m)
            'kp_wall': 1.2,
            'kd_wall': 0.08,
            # both modes
            'steer_gain': 0.8,
            'side_clear': 0.285,        # push away from a side wall closer than this (m)
            'k_side': 0.607,
            # speed planner
            'v_min': 1.5,              # m/s
            'v_max': 10.482,              # m/s
            'a_lat': 7.254,              # usable lateral acceleration (m/s^2)
            'a_brake': 4.707,            # usable braking deceleration (m/s^2)
            'stop_margin': 0.5,       # stopping distance kept in reserve (m)
            'front_half_width': 0.157,  # corridor width used for "free distance ahead" (m)
            'latency': 0.072,           # actuation + bridge delay to plan around (s)
            'v_corner': 4.232,          # speed to be down to when reaching the wall dead ahead
            'a_brake_entry': 7.961,      # braking used for that corner-entry limit (m/s^2)
            # throttle loop
            'v_full_throttle': 24.9,   # measured: throttle 0.1 settles at 2.49 m/s
            'kp_speed': 0.2,
            'throttle_min': 0.0,
            'brake_hyst': 0.4,         # full brake once speed exceeds target by this (m/s)
            # traction control: throttle sets wheel speed, so asking for far more than the
            # car is doing just spins the tyres. Cap wheel speed at v * (1 + slip) + offset.
            # Measured 0 -> 7 m/s: 2.36 s at full throttle, 1.10 s with slip 0.15.
            'tc_slip': 0.154,
            'tc_offset': 0.415,          # m/s, lets the car pull away from standstill
            'steer_smooth': 0.171,       # 0 = no filtering, 0.9 = heavy low-pass
        }
        self.reset()

    def reset(self):
        self.prev_err = 0.0
        self.prev_steer = 0.0
        self.prev_throttle = 0.0
        self.speed_est = LidarSpeed()
        self.last = {}

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def clean(ranges):
        r = np.asarray(ranges, dtype=float).copy()
        r[~np.isfinite(r)] = RANGE_MAX   # no return
        r[r <= 0.0] = RANGE_MAX
        return np.minimum(r, RANGE_MAX)

    @staticmethod
    def beam(r, ang, deg):
        i = int(np.argmin(np.abs(ang - math.radians(deg))))
        return r[i]

    def extend_disparities(self, r, inc):
        p = self.params
        out = r.copy()
        n = len(r)
        jumps = np.nonzero(np.abs(np.diff(r)) > p['disparity'])[0]
        for i in jumps:
            if r[i] < r[i + 1]:
                near, lo_dir = i, +1
            else:
                near, lo_dir = i + 1, -1
            d = max(r[near], 0.05)
            k = int(math.ceil(math.atan2(p['half_width'], d) / inc))
            if lo_dir > 0:
                sl = slice(near, min(n, near + k + 1))
            else:
                sl = slice(max(0, near - k), near + 1)
            out[sl] = np.minimum(out[sl], d)
        return out

    # ----------------------------------------------------------------- steering
    def steer_gap(self, r, ang, inc):
        p = self.params
        ext = self.extend_disparities(r, inc)
        # the car itself is ~half_width wide; anything this close at the side is a wall
        fov = math.radians(p['fov_deg'])
        idx = np.nonzero(np.abs(ang) <= fov)[0]
        depth = ext[idx]
        best = depth.max()
        cand = idx[depth >= best - p['depth_tol']]
        # centre of the deepest contiguous window that contains the max beam
        i_max = idx[np.argmax(depth)]
        split = np.split(cand, np.nonzero(np.diff(cand) != 1)[0] + 1)
        window = next(w for w in split if i_max in w)
        tgt = window[len(window) // 2]
        alpha = ang[tgt]
        ld = min(ext[tgt], p['lookahead_max'])
        # late apex: slide the aim point sideways toward the outside of the turn, so the car
        # stays wide on entry and turns in later on a larger radius (tanh keeps it smooth
        # through alpha = 0 instead of flipping sides)
        off = p['apex_offset'] * math.tanh(alpha / p['apex_alpha'])
        if off:
            px = ld * math.cos(alpha) + off * math.sin(alpha)
            py = ld * math.sin(alpha) - off * math.cos(alpha)
            alpha, ld = math.atan2(py, px), math.hypot(px, py)
        # pure pursuit from the rear axle: kappa = 2 sin(alpha) / ld
        return math.atan(WHEELBASE * 2.0 * math.sin(alpha) / max(ld, 0.3)), ext

    def wall_offset(self, r, ang, side, v):
        """Signed distance-to-wall error for one side, projected ahead (left = +90 deg)."""
        p = self.params
        th = math.radians(p['wall_theta_deg'])
        s = 1.0 if side == 'left' else -1.0
        b = self.beam(r, ang, s * 90.0)
        a = self.beam(r, ang, s * (90.0 - math.degrees(th)))
        alpha = math.atan2(a * math.cos(th) - b, a * math.sin(th))
        d_now = b * math.cos(alpha)
        # a*cos(th) > b means the wall diverges ahead, so the gap grows as the car moves
        return d_now + (p['wall_lookahead'] + v * p['latency']) * math.sin(alpha), alpha

    def steer_wall(self, r, ang, v, dt):
        p = self.params
        dl, al = self.wall_offset(r, ang, 'left', v)
        dr, ar = self.wall_offset(r, ang, 'right', v)
        # positive error = car is right of centre (left wall further away) -> steer left
        err = (dl - dr) / 2.0
        derr = (err - self.prev_err) / max(dt, 1e-3)
        self.prev_err = err
        return p['kp_wall'] * err + p['kd_wall'] * derr

    # -------------------------------------------------------------------- speed
    def target_speed(self, r, ang, delta, v):
        p = self.params
        # free distance along the arc we are actually steering, inside a car-width band
        # (measuring straight ahead brakes for every wall the car is about to turn away from)
        x, y = r * np.cos(ang), r * np.sin(ang)
        kappa_s = math.tan(delta) / WHEELBASE
        if abs(kappa_s) < 1e-3:
            on_path = (np.abs(y) < p['front_half_width']) & (x > 0)
            s = x[on_path]
        else:
            R = 1.0 / kappa_s
            on_path = np.abs(np.hypot(x, y - R) - abs(R)) < p['front_half_width']
            # arc length from the car to each point, measured around the turn centre
            phi = np.arctan2(x, np.sign(R) * (R - y))
            on_path &= phi > 0
            s = abs(R) * phi[on_path]
        d_free = float(s.min()) if s.size else RANGE_MAX
        d_free -= v * p['latency']
        v_brake = math.sqrt(max(0.0, 2.0 * p['a_brake'] * (d_free - p['stop_margin'])))
        kappa = abs(math.tan(delta)) / WHEELBASE
        v_grip = math.sqrt(p['a_lat'] / kappa) if kappa > 1e-3 else p['v_max']
        # corner entry: the wall dead ahead is where the next corner is. Be able to brake
        # down to v_corner before reaching it, even while the gap target already points
        # around the bend (otherwise the car only brakes once it is already turning)
        lateral = np.abs(y)
        dead_ahead = (x > 0) & (lateral < p['front_half_width'])
        d_ahead = float(x[dead_ahead].min()) if dead_ahead.any() else RANGE_MAX
        d_ahead -= v * p['latency'] + p['stop_margin']
        v_entry = math.sqrt(p['v_corner'] ** 2 + 2.0 * p['a_brake_entry'] * max(0.0, d_ahead))
        v_t = min(p['v_max'], v_brake, v_grip, v_entry)
        self.last.update(d_free=d_free, v_brake=v_brake, v_grip=v_grip, v_entry=v_entry)
        return max(p['v_min'], v_t)

    # --------------------------------------------------------------------- step
    def step(self, ranges, angle_min, angle_inc, speed, dt):
        p = self.params
        r = self.clean(ranges)
        ang = angle_min + angle_inc * np.arange(len(r))
        if p['speed_source'] == 'lidar' or speed is None:
            speed = self.speed_est.update(ranges, angle_min, angle_inc, dt, self.prev_throttle)
        v = max(0.0, float(speed))

        if p['mode'] == 'wall':
            delta = self.steer_wall(r, ang, v, dt)
        else:
            delta, _ = self.steer_gap(r, ang, angle_inc)
        delta *= p['steer_gain']

        # keep off the side walls (beams 60-120 deg each side measure lateral clearance)
        side = (np.abs(ang) > math.radians(60)) & (np.abs(ang) < math.radians(120))
        left = float(np.min(r[side & (ang > 0)] * np.abs(np.sin(ang[side & (ang > 0)]))))
        right = float(np.min(r[side & (ang < 0)] * np.abs(np.sin(ang[side & (ang < 0)]))))
        if left < p['side_clear']:
            delta -= p['k_side'] * (p['side_clear'] - left)
        if right < p['side_clear']:
            delta += p['k_side'] * (p['side_clear'] - right)

        delta = max(-MAX_STEER, min(MAX_STEER, delta))
        a = p['steer_smooth']
        delta = a * self.prev_steer + (1.0 - a) * delta
        self.prev_steer = delta

        v_t = self.target_speed(r, ang, delta, v)
        throttle = v_t / p['v_full_throttle'] + p['kp_speed'] * (v_t - v)
        # brake hard, not softly: a throttle a little under the car's speed only drags the
        # motor, while 0 locks the wheels (~8.5 m/s^2 measured)
        if v > v_t + p['brake_hyst']:
            throttle = 0.0
        if p['tc_slip'] > 0:
            throttle = min(throttle, (v * (1.0 + p['tc_slip']) + p['tc_offset'])
                           / p['v_full_throttle'])
        throttle = max(p['throttle_min'], min(1.0, throttle))
        self.last.update(v_target=v_t, delta=delta, left=left, right=right)
        self.prev_throttle = throttle
        self.last['v_used'] = v
        return throttle, delta / MAX_STEER
