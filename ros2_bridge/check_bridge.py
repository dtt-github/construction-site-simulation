"""End-to-end check of the bridge, run inside the image with phase2/ros_server.py and the bridge running:

    docker run --rm --network host -v $PWD/ros2_bridge:/check construction-sim-ros2 python3 /check/check_bridge.py

Listens for 6 s and reports topic rates; decodes a point cloud, a depth image and the static transforms;
then raises the boom through /excavator/levers for 2 s and checks /joint_states follows; then lets the
autopilot drive for 6 s and checks the bucket fills.
"""

import collections
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from tf2_msgs.msg import TFMessage

TOPICS = {"/clock": Clock, "/joint_states": JointState, "/tf": TFMessage, "/lidar/points": PointCloud2,
          "/camera/depth/image_raw": Image, "/camera/depth/camera_info": CameraInfo, "/camera/labels/image_raw": Image,
          "/imu/data": Imu, "/excavator/bucket_load": Float64, "/carrier/stage": String, "/carrier/delivered": Float64,
          "/carrier/joint_states": JointState, "/dozer/stage": String, "/dozer/joint_states": JointState}
LABELS = ["none", "ground", "soil", "machine", "truck"]


class Check(Node):
    def __init__(self):
        super().__init__("bridge_check")
        best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.count, self.last = collections.Counter(), {}
        for name, typ in TOPICS.items():
            self.create_subscription(typ, name, lambda m, n=name: self.got(n, m), best)
        self.static = []
        self.create_subscription(TFMessage, "/tf_static", lambda m: self.static.extend(m.transforms),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.levers = self.create_publisher(Float64MultiArray, "/excavator/levers", 10)
        self.autopilot = self.create_publisher(Bool, "/excavator/autopilot", 10)

    def got(self, name, msg):
        self.count[name] += 1
        self.last[name] = msg


def spin_for(node, seconds, every=None):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.02)
        if every:
            every()


def main():
    rclpy.init()
    n = Check()
    spin_for(n, 1.0)
    n.count.clear()
    spin_for(n, 6.0)
    print("rates (per wall second):", {k: round(v / 6.0, 1) for k, v in sorted(n.count.items())})
    missing = [k for k in TOPICS if n.count[k] == 0]
    assert not missing, f"no messages on {missing}"

    pc = n.last["/lidar/points"]
    rec = np.frombuffer(bytes(pc.data), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("label", "u1"), ("pad", "u1", 3)])
    print(f"lidar: {pc.width} points in {pc.header.frame_id}, labels", {LABELS[k]: int(v) for k, v in zip(*np.unique(rec["label"], return_counts=True))},
          f"range {np.linalg.norm(np.stack([rec['x'], rec['y'], rec['z']], 1), axis=1).max():.1f} m max")
    img = n.last["/camera/depth/image_raw"]
    depth = np.frombuffer(bytes(img.data), dtype=np.float32).reshape(img.height, img.width)
    print(f"depth: {img.width}x{img.height} {img.encoding} in {img.header.frame_id}, {np.mean(depth > 0):.0%} pixels with a return, "
          f"{depth[depth > 0].min():.2f}..{depth.max():.2f} m; camera_info fx {n.last['/camera/depth/camera_info'].k[0]:.1f}")
    frames = sorted(f"{t.header.frame_id}->{t.child_frame_id}" for t in n.static)
    print("tf_static:", frames, "; tf:", sorted(f"{t.header.frame_id}->{t.child_frame_id}" for t in n.last["/tf"].transforms))
    assert {"body_link->lidar_link", "body_link->camera_optical_frame", "body_link->imu_link"} <= set(frames)
    js = n.last["/joint_states"]
    print("joint_states:", list(js.name), np.round(np.degrees(js.position), 1).tolist(), "sim time", n.last["/clock"].clock.sec)

    boom0 = js.position[1]
    spin_for(n, 2.0, lambda: n.levers.publish(Float64MultiArray(data=[0.0, 1.0, 0.0, 0.0])))
    spin_for(n, 0.3)
    moved = np.degrees(n.last["/joint_states"].position[1] - boom0)
    print(f"boom lever +1 for 2 s: boom_joint moved {moved:+.1f} deg (raising decreases it)")
    assert moved < -10.0, moved

    n.autopilot.publish(Bool(data=True))
    peak = 0.0
    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(n, timeout_sec=0.02)
        if "/excavator/bucket_load" in n.last:
            peak = max(peak, n.last["/excavator/bucket_load"].data)
    n.autopilot.publish(Bool(data=False))
    spin_for(n, 0.2)
    print(f"autopilot for 6 s: bucket load peaked at {peak:.0f} kg")
    assert peak > 100.0, peak
    st = n.last.get("/carrier/stage")
    print(f"carrier: stage {st.data if st else None}, delivered {n.last['/carrier/delivered'].data:.0f} kg, vessel_pin_joint {n.last['/carrier/joint_states'].position[0]:+.2f} rad")
    print(f"dozer: stage {n.last['/dozer/stage'].data}, lift_joint {n.last['/dozer/joint_states'].position[0]:+.2f} rad")
    print("bridge check: ok")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
