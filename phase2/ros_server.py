"""Simulator side of the ROS 2 bridge: runs the excavator, its soil and its sensors in real time (or as fast
as possible with --fast) and serves them over TCP to the bridge node in the ROS 2 container
(ros2_bridge/excavator_bridge.py), which publishes them as ROS topics. No ROS on this side.

    ../phase0/.venv-newton/bin/python ros_server.py --soil loam            # listens on 127.0.0.1:7777

Protocol (both directions): messages of a 4-byte little-endian length, then a JSON header line, then the raw
bytes of the arrays the header lists ({"name", "dtype", "shape"} in order).
    server -> client  hello (once): joint names, sensor mounts and intrinsics, rates
                      state (STATE_HZ): sim time, joints, pressures, base and body poses, loads, soil force
                      imu (IMU_HZ), lidar (LIDAR_HZ: points in the lidar frame and labels), camera
                      (CAMERA_HZ: depth and label images)
    client -> server  {"type": "levers", "levers": [swing, boom, stick, bucket]}   each in [-1, 1]
                      {"type": "dispatch"}   send the carrier off with its load
                      {"type": "autopilot", "on": true | false}
                      {"type": "reset"}
Levers return to neutral when no lever message arrives for COMMAND_TIMEOUT s (a dead-man switch).
"""

import argparse
import asyncio
import json
import math
import struct
import time

import numpy as np

import excavator_server as ex
import hydraulics as hyd
import sensors
import soils

STATE_HZ, IMU_HZ, LIDAR_HZ, CAMERA_HZ = 30.0, 60.0, 10.0, 10.0
COMMAND_TIMEOUT = 0.5  # s
URDF_JOINTS = ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]  # PWRI OPERA zx200_ros2 (Apache-2.0)


def pack(header, arrays=()):
    header = dict(header, arrays=[{"name": n, "dtype": str(a.dtype), "shape": list(a.shape)} for n, a in arrays])
    body = json.dumps(header).encode() + b"\n" + b"".join(np.ascontiguousarray(a).tobytes() for _, a in arrays)
    return struct.pack("<I", len(body)) + body


def base_pose(sim):
    """base_link (= track_link of the URDF) in the zone frame: the house pose (URDF body_link) with the swing
    joint undone, so the URDF chain base_link -> body_link -> ... reproduces the simulator's links."""
    p, R = sensors.house_pose(sim)
    c, s = math.cos(sim.q[0]), math.sin(sim.q[0])
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    Rb = R @ Rz.T
    return p - Rb @ ex.CRAWLER_SWING, Rb


class Server:
    def __init__(self, sim, rig, fast=False):
        self.sim, self.rig, self.fast = sim, rig, fast
        self.clients = set()
        self.last_cmd = -1e9
        self.next = {k: 0.0 for k in ("state", "imu", "lidar", "camera")}

    def hello(self):
        rig = self.rig
        return pack({"type": "hello", "joints": URDF_JOINTS, "soil": self.sim.soil, "frame_dt": self.sim.frame_dt,
                     "rates": {"state": STATE_HZ, "imu": IMU_HZ, "lidar": LIDAR_HZ, "camera": CAMERA_HZ},
                     "mounts": {"lidar": sensors.LIDAR_MOUNT.tolist(), "camera": sensors.CAMERA_MOUNT.tolist(),
                                "camera_q": ex.quat_from_matrix(rig.R_house_cam).tolist(), "imu": sensors.IMU_MOUNT.tolist()},
                     "camera": {"K": rig.camera_K.tolist(), "height": rig.camera_shape[0], "width": rig.camera_shape[1], "range": rig.camera_range},
                     "lidar": {"channels": rig.lidar_shape[0], "columns": rig.lidar_shape[1], "range": rig.lidar_range},
                     "labels": sensors.LABELS})

    def command(self, msg):
        sim = self.sim
        if msg.get("type") == "levers":
            sim.set_levers(*[float(v) for v in msg["levers"][:4]])
            self.last_cmd = time.monotonic()
        elif msg.get("type") == "autopilot":
            sim.auto = bool(msg.get("on"))
        elif msg.get("type") == "dispatch":
            sim.dispatch()
        elif msg.get("type") == "reset":
            sim.reset()
            self.rig.imu_prev = None
            self.next = {k: 0.0 for k in self.next}

    def messages(self):
        """What is due at the current sim time."""
        sim, rig, t, out = self.sim, self.rig, self.sim.sim_time, []
        if t >= self.next["state"]:
            self.next["state"] = t + 1.0 / STATE_HZ
            sim.measure()
            bp, bR = base_pose(sim)
            hp, hR = rig.house_pose()
            cyl = sim.cyl_p * np.where(sim.cyl_head, 1.0, -1.0)
            out.append(pack({"type": "state", "t": t, "stage": sim.stage_name, "auto": bool(sim.auto), "carrier_stages": [c.stage for c in sim.fleet], "dozer_stage": sim.dozer.stage}, [
                ("q", sim.q.astype(np.float64)), ("qd", sim.qd.astype(np.float64)),
                ("pressure", np.array([sim.tau[0] / hyd.SWING_DISP, *cyl], dtype=np.float64)),
                ("levers", sim.levers.astype(np.float64)),
                ("base_p", bp.astype(np.float64)), ("base_q", ex.quat_from_matrix(bR).astype(np.float64)),
                ("body_p", hp.astype(np.float64)), ("body_q", ex.quat_from_matrix(hR).astype(np.float64)),
                ("bucket_tip", np.asarray(ex.fk(sim.q)[2], dtype=np.float64)), ("soil_force", sim.force.astype(np.float64)),
                ("loads", np.array([sim.fill_kg, sim.truck_kg], dtype=np.float64)),
                ("carriers", np.array([[c.base[0], c.base[1], ex.GROUND_Z, c.yaw, c.tip, c.payload_kg, c.delivered_kg] for c in sim.fleet], dtype=np.float64)),
                ("dozer", np.array([*sim.dozer.base_pos(), *sim.dozer.base_quat(), sim.dozer.lift, sim.dozer.force], dtype=np.float64))]))
        if t >= self.next["imu"]:
            self.next["imu"] = t + 1.0 / IMU_HZ
            imu = rig.imu()
            out.append(pack({"type": "imu", "t": t}, [("angular_velocity", imu["angular_velocity"].astype(np.float64)),
                                                      ("linear_acceleration", imu["linear_acceleration"].astype(np.float64))]))
        if t >= self.next["lidar"]:
            self.next["lidar"] = t + 1.0 / LIDAR_HZ
            scan = rig.lidar()
            out.append(pack({"type": "lidar", "t": t}, [("points", scan["points"]), ("labels", scan["point_labels"])]))
        if t >= self.next["camera"]:
            self.next["camera"] = t + 1.0 / CAMERA_HZ
            image = rig.depth_camera()
            out.append(pack({"type": "camera", "t": t}, [("depth", image["depth"]), ("labels", image["labels"])]))
        return out

    async def handle(self, reader, writer):
        self.clients.add(writer)
        writer.write(self.hello())
        print(f"client connected: {writer.get_extra_info('peername')}", flush=True)
        try:
            while True:
                head = await reader.readexactly(4)
                body = await reader.readexactly(struct.unpack("<I", head)[0])
                self.command(json.loads(body.split(b"\n", 1)[0]))
        except (asyncio.IncompleteReadError, ConnectionError):
            pass
        finally:
            self.clients.discard(writer)
            print("client disconnected", flush=True)

    async def run(self):
        sim = self.sim
        t_wall0, t_sim0 = time.monotonic(), sim.sim_time
        while True:
            if not sim.auto and time.monotonic() - self.last_cmd > COMMAND_TIMEOUT and np.any(sim.levers):
                sim.set_levers(0.0, 0.0, 0.0, 0.0)  # dead-man: no recent command
            sim.step()
            if sim.sim_time < t_sim0:  # reset
                t_wall0, t_sim0 = time.monotonic(), sim.sim_time
            for m in self.messages() if self.clients else ():
                for w in list(self.clients):
                    w.write(m)
            for w in list(self.clients):
                if w.transport.get_write_buffer_size() > 64 << 20:  # a stuck client: drop it
                    w.close()
                    self.clients.discard(w)
                else:
                    await w.drain()
            ahead = (sim.sim_time - t_sim0) - (time.monotonic() - t_wall0)
            await asyncio.sleep(ahead if ahead > 0 and not self.fast else 0)


async def main_async(args):
    sim = ex.ExcavatorSim(soil=args.soil, carriers=args.carriers)
    sim.auto = args.autopilot
    server = Server(sim, sensors.SensorRig(sim), fast=args.fast)
    srv = await asyncio.start_server(server.handle, args.host, args.port)
    print(f"excavator simulator for ROS 2 on {args.host}:{args.port} (soil {args.soil})", flush=True)
    async with srv:
        await server.run()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=7777)
    ap.add_argument("--soil", default=soils.DEFAULT_SOIL, choices=list(soils.SOILS))
    ap.add_argument("--carriers", type=int, default=1, choices=(1, 2), help="crawler carriers in the haul cycle")
    ap.add_argument("--autopilot", action="store_true", help="start with the built-in operator driving")
    ap.add_argument("--fast", action="store_true", help="run as fast as possible instead of in real time")
    asyncio.run(main_async(ap.parse_args()))


if __name__ == "__main__":
    main()
