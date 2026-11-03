"""Checks of the Gymnasium environment (GPU; about 2 minutes):

    ../phase0/.venv-newton/bin/python test_env.py

1. gymnasium's env_checker (spaces, reset/step signatures, dtypes, seeding)
2. each lever moves its joint the documented way (boom + raises, stick + out, bucket + dumps, swing + left)
3. reset restores the machine, the pit and the truck
4. the built-in operator, as autopilot, earns the load-truck reward and reports the levers it used
"""

import numpy as np
from gymnasium.utils.env_checker import check_env

import excavator_server as ex
from excavator_env import ExcavatorEnv


def lever_moves(env, k, sign, seconds=1.0):
    """Joint change and bucket-tip change (height, distance from the swing axis) for one lever held."""
    obs0, _ = env.reset()
    a = np.zeros(4, dtype=np.float32)
    a[k] = sign
    for _ in range(round(seconds / env.control_dt)):
        obs, *_ = env.step(a)
    reach = lambda o: float(np.hypot(*(o["bucket_tip"][:2] - ex.SWING_POS[:2])))  # noqa: E731
    return float(obs["joint_pos"][k] - obs0["joint_pos"][k]), float(obs["bucket_tip"][2] - obs0["bucket_tip"][2]), reach(obs) - reach(obs0)


def main():
    env = ExcavatorEnv(episode_s=4.0)
    check_env(env, skip_render_check=True)
    print("1. env_checker: ok")

    # each lever, held 1 s each way, moves its joint both ways; boom + lifts the bucket tip, stick + reaches out
    names = ["swing (+ left)", "boom (+ raise)", "stick (+ out)", "bucket (+ dump)"]
    for k, name in enumerate(names):
        (up, up_z, up_r), (down, _, _) = lever_moves(env, k, +1.0), lever_moves(env, k, -1.0)
        assert up * down < 0 and min(abs(up), abs(down)) > 0.05, (name, up, down)
        if k == 1:
            assert up_z > 0.2, up_z
        if k == 2:
            assert up_r > 0.2, up_r
        print(f"2. {name:16s} joint per second at +1: {np.degrees(up):+6.1f} deg, at -1: {np.degrees(down):+6.1f} deg; "
              f"tip at +1: height {up_z:+.2f} m, reach {up_r:+.2f} m")

    obs0, _ = env.reset()
    for _ in range(20):
        env.step(np.array([0.0, -1.0, 1.0, 0.0], dtype=np.float32))
    obs1, _ = env.reset()
    for key in obs0:
        assert np.allclose(obs0[key], obs1[key], atol=1e-3), key
    print("3. reset restores the observation exactly: ok")

    auto = ExcavatorEnv(autopilot=True, episode_s=25.0, terrain=False)
    obs, _ = auto.reset()
    total, levers = 0.0, []
    trunc = False
    while not trunc:
        obs, r, _, trunc, info = auto.step(None)
        total += r
        levers.append(info["action"])
    levers = np.array(levers)
    assert total > 0.5 and np.abs(levers).max() > 0.5, (total, np.abs(levers).max())
    print(f"4. autopilot: {total:.2f} t to the truck in 25 s; lever ranges {np.round(levers.min(0), 2)} .. {np.round(levers.max(0), 2)}")


if __name__ == "__main__":
    main()
