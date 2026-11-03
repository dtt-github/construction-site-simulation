"""Record demonstrations from the built-in operator model for imitation learning: observations, the levers
it used and the rewards, one .npz per episode (arrays stacked over control steps; terrain as float16).

    ../phase0/.venv-newton/bin/python record_demos.py --soil loam --episodes 3 --seconds 60 --out demos/
"""

import argparse
import os
import time

import numpy as np

from excavator_env import ExcavatorEnv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--soil", default="dry-sand")
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--control-hz", type=float, default=10.0)
    ap.add_argument("--no-terrain", action="store_true")
    ap.add_argument("--out", default="demos")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    env = ExcavatorEnv(soil=args.soil, autopilot=True, episode_s=args.seconds, control_hz=args.control_hz, terrain=not args.no_terrain)
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=ep)
        rows = {k: [v] for k, v in obs.items()}
        actions, rewards, stages = [], [], []
        t0, done = time.perf_counter(), False
        while not done:
            obs, r, term, trunc, info = env.step(None)
            for k, v in obs.items():
                rows[k].append(v)
            actions.append(info["action"])
            rewards.append(r)
            stages.append(info["stage"])
            done = term or trunc
        data = {f"obs_{k}": np.stack(v) for k, v in rows.items()}  # one more than actions: obs before each step and the last
        if "obs_terrain" in data:
            data["obs_terrain"] = data["obs_terrain"].astype(np.float16)
        data.update(actions=np.stack(actions), rewards=np.array(rewards, dtype=np.float32), stages=np.array(stages),
                    control_dt=env.control_dt, soil=args.soil)
        path = os.path.join(args.out, f"{args.soil}_ep{ep:03d}.npz")
        np.savez_compressed(path, **data)
        print(f"{path}: {len(actions)} steps, {sum(rewards):.2f} t to the truck, {time.perf_counter() - t0:.0f} s wall", flush=True)


if __name__ == "__main__":
    main()
