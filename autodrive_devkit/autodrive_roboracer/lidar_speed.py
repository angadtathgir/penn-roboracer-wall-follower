"""Speed estimate from the LiDAR alone (no odometry), for the LiDAR-only controller.

Two sources, fused:
  * scan matching: point-to-line ICP between the current scan and one taken ~20 ms
    earlier. Correspondences are found by beam angle (projective association), which is
    cheap and accurate because the car moves only a few centimetres between the scans.
  * a throttle model: throttle sets wheel speed (1.0 ~ 24.9 m/s), and the car follows it
    with limited acceleration and braking. It carries the estimate through stretches
    where the walls do not constrain forward motion (e.g. two parallel walls).
"""
import math

import numpy as np

V_FULL = 24.9


class LidarSpeed:
    def __init__(self, gap_s=0.02, a_acc=6.5, a_brake=8.0, gain=0.35, stride=3):
        self.gap_s = gap_s          # time between the two scans that get matched
        self.a_acc = a_acc          # model acceleration limit (traction control on)
        self.a_brake = a_brake      # model braking (throttle 0 locks the wheels)
        self.gain = gain            # how strongly a confident ICP result corrects the model
        self.stride = stride        # use every n-th beam of the current scan
        self.v = 0.0
        self.t = 0.0
        self.ref = None             # (time, points, normals, valid) of the reference scan
        self.v_icp = None

    @staticmethod
    def _points(r, ang):
        valid = (r > 0.08) & (r < 9.5)
        pts = np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)
        # normals from neighbouring beams; drop edges where the surface jumps
        d = np.zeros_like(pts)
        d[1:-1] = pts[2:] - pts[:-2]
        gap = np.hypot(d[:, 0], d[:, 1])
        ok = valid & (gap > 1e-4) & (gap < 0.12)
        ok[[0, -1]] = False
        n = np.stack([-d[:, 1], d[:, 0]], axis=1) / np.maximum(gap, 1e-9)[:, None]
        return pts, n, ok

    def _icp(self, ref, cur_pts, cur_ok, angle_min, inc, guess):
        _, P, N, ok_ref = ref
        q = cur_pts[::self.stride][cur_ok[::self.stride]]
        x = np.array(guess, dtype=float)          # (tx, ty, phi): current -> reference
        H = None
        for _ in range(4):
            c, s = math.cos(x[2]), math.sin(x[2])
            qx = c * q[:, 0] - s * q[:, 1] + x[0]
            qy = s * q[:, 0] + c * q[:, 1] + x[1]
            idx = np.rint((np.arctan2(qy, qx) - angle_min) / inc).astype(int)
            inb = (idx >= 0) & (idx < len(P))
            idx = np.where(inb, idx, 0)
            use = inb & ok_ref[idx]
            p, n = P[idx[use]], N[idx[use]]
            ex = qx[use] - p[:, 0]
            ey = qy[use] - p[:, 1]
            e = n[:, 0] * ex + n[:, 1] * ey
            keep = np.abs(e) < 0.15
            if keep.sum() < 30:
                return None, None
            n, e = n[keep], e[keep]
            qq = q[use][keep]
            dqx = -s * qq[:, 0] - c * qq[:, 1]
            dqy = c * qq[:, 0] - s * qq[:, 1]
            J = np.stack([n[:, 0], n[:, 1], n[:, 0] * dqx + n[:, 1] * dqy], axis=1)
            H = J.T @ J
            dx = np.linalg.solve(H + 1e-6 * np.eye(3), -J.T @ e)
            x += dx
            if abs(dx[0]) + abs(dx[1]) < 1e-5:
                break
        return x, H

    def update(self, ranges, angle_min, inc, dt, throttle_prev):
        """Feed one scan; returns the fused forward speed (m/s)."""
        self.t += dt
        # model prediction
        w = throttle_prev * V_FULL
        a = -self.a_brake if throttle_prev <= 0 else max(-self.a_brake,
                                                          min(self.a_acc, (w - self.v) / 0.1))
        self.v = max(0.0, self.v + a * dt)

        r = np.nan_to_num(np.asarray(ranges, dtype=float), nan=0.0, posinf=0.0)
        ang = angle_min + inc * np.arange(len(r))
        pts, nrm, ok = self._points(r, ang)
        if self.ref is None:
            self.ref = (self.t, pts, nrm, ok)
            return self.v
        span = self.t - self.ref[0]
        if span < self.gap_s:
            return self.v
        x, H = self._icp(self.ref, pts, ok, angle_min, inc, (self.v * span, 0.0, 0.0))
        self.ref = (self.t, pts, nrm, ok)
        if x is None:
            return self.v
        # forward motion is only trustworthy if surfaces facing the car constrain it
        info_x = H[0, 0]
        conf = info_x / (info_x + 20.0)
        self.v_icp = max(0.0, x[0] / span)
        self.v += self.gain * conf * (self.v_icp - self.v)
        return self.v
