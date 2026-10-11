#!/usr/bin/env python3
"""Random-search controller parameters in the real AutoDRIVE sim, spread over several
sim + bridge containers (run on the host, not inside a container).

    python3 tune.py --mode gap --n 90 --out results_gap.jsonl
    python3 tune.py --around best.json --n 60 --scale 0.15 --out refine.jsonl

Each config drives --laps laps; the score is the best flying lap (lap >= 2) of a run with
no wall contact. Results are appended as JSON lines.
"""
import argparse
import json
import os
import random
import subprocess
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
CONTAINERS = ['penn-challenge', 'pc2', 'pc3']
IN_CONTAINER = '/home/autodrive_devkit/src/autodrive_devkit/tools'

SPACE = {
    'common': {
        'v_max': (6.0, 14.0), 'a_lat': (5.0, 12.0), 'a_brake': (4.0, 10.0),
        'v_corner': (2.0, 6.0), 'a_brake_entry': (3.0, 10.0),
        'tc_slip': (0.05, 0.3), 'tc_offset': (0.2, 1.0),
        'stop_margin': (0.1, 0.9), 'latency': (0.0, 0.15), 'kp_speed': (0.03, 0.5),
        'steer_gain': (0.5, 1.5), 'side_clear': (0.15, 0.35), 'k_side': (0.0, 2.0),
        'front_half_width': (0.13, 0.22), 'steer_smooth': (0.0, 0.6),
    },
    'gap': {
        'fov_deg': (35.0, 110.0), 'half_width': (0.18, 0.34), 'disparity': (0.15, 0.6),
        'lookahead_max': (0.6, 5.0), 'depth_tol': (0.0, 0.5),
        'apex_offset': (-0.3, 0.3), 'apex_alpha': (0.1, 0.8),
    },
    'wall': {
        'wall_theta_deg': (30.0, 70.0), 'wall_lookahead': (0.3, 1.5),
        'kp_wall': (0.5, 3.0), 'kd_wall': (0.0, 0.3),
    },
}


def sample(mode, center=None, scale=None):
    p = {'mode': mode}
    for k, (lo, hi) in {**SPACE['common'], **SPACE[mode]}.items():
        if center is None:
            p[k] = round(random.uniform(lo, hi), 3)
        else:
            v = center[k] + random.gauss(0, scale * (hi - lo))
            p[k] = round(min(hi, max(lo, v)), 3)
    return p


def worker(container, configs, laps, out, lock):
    path = os.path.join(HERE, f'_batch_{container}.json')
    with open(path, 'w') as f:
        json.dump(configs, f)
    cmd = ['docker', 'exec', container, 'bash', '-lc',
           f'source /opt/ros/humble/setup.bash && cd {IN_CONTAINER} && '
           f'python3 run_laps.py --laps {laps} --timeout {12 * laps + 15} '
           f'--configs @{os.path.basename(path)}']
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
    for line in proc.stdout:
        try:
            r = json.loads(line)
        except ValueError:
            continue
        flying = r['laps'][1:]
        r['score'] = min(flying) if r['result'] == 'ok' and flying else None
        with lock:
            with open(out, 'a') as f:
                f.write(json.dumps(r) + '\n')
            s = f"{r['score']:.3f}" if r['score'] else r['result']
            print(f'[{container}] {s:>9}  laps={r["laps"]}', flush=True)
    proc.wait()
    os.remove(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='gap')
    ap.add_argument('--n', type=int, default=30)
    ap.add_argument('--laps', type=int, default=2)
    ap.add_argument('--around', help='JSON file with a param dict to sample around')
    ap.add_argument('--scale', type=float, default=0.15)
    ap.add_argument('--configs', help='JSON file with an explicit list of configs')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    if a.configs:
        configs = json.load(open(a.configs))
    elif a.around:
        c = json.load(open(a.around))
        configs = [dict(c)] + [sample(c['mode'], c, a.scale) for _ in range(a.n - 1)]
    else:
        configs = [sample(a.mode) for _ in range(a.n)]

    chunks = [configs[i::len(CONTAINERS)] for i in range(len(CONTAINERS))]
    lock = threading.Lock()
    ts = [threading.Thread(target=worker, args=(c, ch, a.laps, a.out, lock))
          for c, ch in zip(CONTAINERS, chunks) if ch]
    for t in ts:
        t.start()
    for t in ts:
        t.join()

    rows = [json.loads(line) for line in open(a.out)]
    good = sorted((r for r in rows if r.get('score')), key=lambda r: r['score'])
    print(f'\n{len(good)}/{len(rows)} clean. Top 5:')
    for r in good[:5]:
        print(f"  {r['score']:.3f}  {json.dumps(r['params'])}")


if __name__ == '__main__':
    main()
