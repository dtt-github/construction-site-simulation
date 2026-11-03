"""Gymnasium environment for the excavator: the same simulator as the browser demo (ZX200-class machine,
hydraulic cylinders, MPM soil), driven through the operator's four levers, for reinforcement learning,
imitation learning and controller research.

    import excavator_env
    env = excavator_env.ExcavatorEnv(soil="loam")
    obs, info = env.reset()
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())

Action: the four spring-return levers, each in [-1, 1], held for one control step:
    [swing (+ left), boom (+ raise), stick (+ out), bucket (+ dump)]
Each lever opens its valve in proportion; the hydraulics (flow, relief pressure) and the soil decide how fast
the joint actually moves, as on the machine.

Observation (dict of float32 arrays, SI units, zone frame: x across the pit, z up):
    joint_pos (4)          swing, boom, stick, bucket joint angles (rad): swinging left increases swing; raising
                           the boom, reaching the stick out and dumping the bucket decrease their angles
    joint_vel (4)          their rates (rad/s)
    pressure (4)           swing motor and boom, stick, bucket cylinder pressures (Pa); signed: + head side
                           (swing: + turning left)
    bucket_tip (3)         cutting edge position (m)
    bucket_force (3)       soil force on the bucket (N)
    bucket_load (1)        soil inside the bucket shell (kg)
    truck_load (1)         soil in the truck bed (kg)
    terrain (86, 96)       surface height of the pit (m) on a 0.1 m grid, rows along y, from x, y = -3, -2.5
                           (only with terrain=True)

Tasks (reward per control step; pass reward_fn to define your own):
    "load-truck"   tonnes of soil added to the truck
    "fill-bucket"  tonnes of soil added to the bucket (negative when it spills)

autopilot=True lets the built-in operator model (the browser's "auto" cycle) drive and ignores the actions;
info["action"] holds the levers it used, in the action convention: demonstrations for imitation learning.

Episodes are truncated after episode_s simulated seconds; nothing terminates them. One simulator is built
per environment (about 10 s, GPU); reset() restores the pit without rebuilding unless the soil changes
(reset(options={"soil": "clay"})). env.sim is the ExcavatorSim, for anything the observation leaves out
(stats(), particles, the truck heightmap).
"""

import gymnasium as gym
import numpy as np
from gymnasium import spaces

import excavator_server as ex
import hydraulics as hyd
import soils

TASKS = ("load-truck", "fill-bucket")


class ExcavatorEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, soil=soils.DEFAULT_SOIL, task="load-truck", control_hz=10.0, episode_s=60.0,
                 autopilot=False, terrain=True, reward_fn=None, sim_kwargs=None):
        if task not in TASKS and reward_fn is None:
            raise ValueError(f"task must be one of {TASKS}, or pass reward_fn")
        self.task, self.reward_fn = task, reward_fn
        self.episode_s, self.autopilot, self.with_terrain = episode_s, autopilot, terrain
        self.sim_kwargs = dict(sim_kwargs or {})
        self.sim = ex.ExcavatorSim(soil=soil, **self.sim_kwargs)
        self.frame_skip = max(1, round(1.0 / (control_hz * self.sim.frame_dt)))
        self.control_dt = self.frame_skip * self.sim.frame_dt

        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)
        lo = np.array([ex.LIMITS[j][0] for j in ex.JOINTS], dtype=np.float32)
        hi = np.array([ex.LIMITS[j][1] for j in ex.JOINTS], dtype=np.float32)
        obs = {
            "joint_pos": spaces.Box(lo, hi, dtype=np.float32),
            "joint_vel": spaces.Box(-np.inf, np.inf, shape=(4,), dtype=np.float32),
            "pressure": spaces.Box(-np.inf, np.inf, shape=(4,), dtype=np.float32),
            "bucket_tip": spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32),
            "bucket_force": spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32),
            "bucket_load": spaces.Box(0.0, np.inf, shape=(1,), dtype=np.float32),
            "truck_load": spaces.Box(0.0, np.inf, shape=(1,), dtype=np.float32),
        }
        if terrain:
            obs["terrain"] = spaces.Box(-np.inf, np.inf, shape=(ex.HM_NY, ex.HM_NX), dtype=np.float32)
        self.observation_space = spaces.Dict(obs)
        self._obs = None

    # ------------------------------------------------------------------ gymnasium API
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        soil = options.get("soil", self.sim.soil)
        if soil != self.sim.soil:
            self.sim = ex.ExcavatorSim(soil=soil, **self.sim_kwargs)
        else:
            self.sim.reset()
        self.sim.auto = self.autopilot
        self.sim.measure()
        self._obs = self._observe()
        return self._obs, self._info(np.zeros(4, dtype=np.float32))

    def step(self, action):
        sim = self.sim
        if self.autopilot:
            sim.auto = True
            used = np.zeros(4)
            for _ in range(self.frame_skip):
                sim.step()
                used += sim.levers
            used /= self.frame_skip
        else:
            a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
            sim.set_levers(*a)
            for _ in range(self.frame_skip):
                sim.step()
            used = a
        sim.measure()
        prev, obs = self._obs, self._observe()
        self._obs = obs
        info = self._info(used.astype(np.float32))
        if self.reward_fn is not None:
            reward = float(self.reward_fn(prev, obs, info))
        elif self.task == "load-truck":
            reward = float(obs["truck_load"][0] - prev["truck_load"][0]) / 1000.0
        else:
            reward = float(obs["bucket_load"][0] - prev["bucket_load"][0]) / 1000.0
        truncated = sim.sim_time >= self.episode_s - 1e-6
        return obs, reward, False, truncated, info

    # ------------------------------------------------------------------ helpers
    def _observe(self):
        sim = self.sim
        cyl = sim.cyl_p * np.where(sim.cyl_head, 1.0, -1.0)
        swing = sim.tau[0] / hyd.SWING_DISP
        obs = {
            "joint_pos": sim.q.astype(np.float32),
            "joint_vel": sim.qd.astype(np.float32),
            "pressure": np.array([swing, *cyl], dtype=np.float32),
            "bucket_tip": np.asarray(ex.fk(sim.q)[2], dtype=np.float32),
            "bucket_force": sim.force.astype(np.float32),
            "bucket_load": np.array([sim.fill_kg], dtype=np.float32),
            "truck_load": np.array([sim.truck_kg], dtype=np.float32),
        }
        if self.with_terrain:
            obs["terrain"] = sim.hm_np.reshape(ex.HM_NY, ex.HM_NX).astype(np.float32)
        return obs

    def _info(self, action):
        sim = self.sim
        return {
            "sim_time": sim.sim_time,
            "stage": sim.stage_name,
            "action": action,
            "cut_depth": sim.cut_depth,
            "mckyes_force": sim.fee_ref,
            "passes": sim.passes,
        }


if __name__ == "__main__":
    import time

    env = ExcavatorEnv(autopilot=True, episode_s=30.0)
    obs, info = env.reset(seed=0)
    total, t0, n = 0.0, time.perf_counter(), 0
    while True:
        obs, r, term, trunc, info = env.step(None)
        total += r
        n += 1
        if trunc or term:
            break
    wall = time.perf_counter() - t0
    print(f"{n} steps, {info['sim_time']:.1f} s simulated in {wall:.1f} s ({info['sim_time'] / wall:.2f}x real time), "
          f"reward {total:.2f} t, truck {obs['truck_load'][0]:.0f} kg, passes {info['passes']}")
