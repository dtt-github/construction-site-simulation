"""ROS 2 (Jazzy) node for the excavator simulator: connects to phase2/ros_server.py over TCP and publishes the
machine, its soil and its sensors as standard ROS topics; lever commands go back to the simulator.

Runs in the ros2_bridge Docker image (see README.md there); needs rclpy, the standard message packages and
numpy, nothing from this repository.

Published (stamps in simulation time; run your nodes with use_sim_time:=true):
    /clock                              rosgraph_msgs/Clock
    /joint_states                       sensor_msgs/JointState   swing_joint, boom_joint, arm_joint, bucket_joint
                                        (the names and angles of PWRI OPERA's zx200_ros2 URDF); effort: hydraulic
                                        pressure on each joint's motor or cylinder (Pa, + head side)
    /tf                                 world -> base_link (the URDF's base; robot_state_publisher with the
                                        zx200 description then gives every link), world -> body_link
                                        (the upper structure, for users without the URDF)
    /tf_static                          body_link -> lidar_link, camera_link, camera_optical_frame, imu_link
    /lidar/points                       sensor_msgs/PointCloud2  x, y, z (float32), label (uint8), lidar_link
    /camera/depth/image_raw             sensor_msgs/Image 32FC1  depth along the optical axis (m), 0: no return
    /camera/depth/camera_info           sensor_msgs/CameraInfo
    /camera/labels/image_raw            sensor_msgs/Image mono8  semantic label per pixel
    /imu/data                           sensor_msgs/Imu          angular velocity, specific force (no orientation)
    /excavator/levers_state             std_msgs/Float64MultiArray  levers in use [swing, boom, stick, bucket]
    /excavator/bucket_tip               geometry_msgs/PointStamped   cutting edge, world frame
    /excavator/soil_force               geometry_msgs/Vector3Stamped soil force on the bucket (N), world frame
    /excavator/bucket_load              std_msgs/Float64  soil in the bucket (kg)
    /excavator/truck_load               std_msgs/Float64  soil in the truck (kg)
    /excavator/stage                    std_msgs/String   operator stage ("manual" or the autopilot's stage)
Subscribed:
    /excavator/levers     std_msgs/Float64MultiArray  [swing (+ left), boom (+ raise), stick (+ out), bucket (+ dump)],
                          each in [-1, 1]; publish at 10 Hz or more: levers return to neutral 0.5 s after the last
    /excavator/autopilot  std_msgs/Bool   the built-in operator drives (any lever message takes over again)
    /excavator/reset      std_msgs/Empty  restore the machine, the pit and the truck
Labels: 0 none, 1 ground, 2 soil, 3 machine, 4 truck.

    python3 excavator_bridge.py --ros-args -p host:=127.0.0.1 -p port:=7777
"""

import json
import socket
import struct
import threading

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, TransformStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2, PointField
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, String
from tf2_msgs.msg import TFMessage


def stamp(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def transform(parent, child, t, p, q):
    m = TransformStamped()
    m.header.stamp, m.header.frame_id, m.child_frame_id = stamp(t), parent, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (float(v) for v in p)
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = (float(v) for v in q)
    return m


class Bridge(Node):
    def __init__(self):
        super().__init__("excavator_bridge")
        self.host = self.declare_parameter("host", "127.0.0.1").value
        self.port = int(self.declare_parameter("port", 7777).value)
        sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = {
            "clock": self.create_publisher(Clock, "/clock", 10),
            "joints": self.create_publisher(JointState, "/joint_states", 10),
            "tf": self.create_publisher(TFMessage, "/tf", 10),
            "tf_static": self.create_publisher(TFMessage, "/tf_static", latched),
            "points": self.create_publisher(PointCloud2, "/lidar/points", sensor_qos),
            "depth": self.create_publisher(Image, "/camera/depth/image_raw", sensor_qos),
            "info": self.create_publisher(CameraInfo, "/camera/depth/camera_info", sensor_qos),
            "labels": self.create_publisher(Image, "/camera/labels/image_raw", sensor_qos),
            "imu": self.create_publisher(Imu, "/imu/data", sensor_qos),
            "levers": self.create_publisher(Float64MultiArray, "/excavator/levers_state", 10),
            "tip": self.create_publisher(PointStamped, "/excavator/bucket_tip", 10),
            "force": self.create_publisher(Vector3Stamped, "/excavator/soil_force", 10),
            "bucket": self.create_publisher(Float64, "/excavator/bucket_load", 10),
            "truck": self.create_publisher(Float64, "/excavator/truck_load", 10),
            "stage": self.create_publisher(String, "/excavator/stage", 10),
        }
        self.create_subscription(Float64MultiArray, "/excavator/levers", lambda m: self.send({"type": "levers", "levers": list(m.data)[:4]}), 10)
        self.create_subscription(Bool, "/excavator/autopilot", lambda m: self.send({"type": "autopilot", "on": bool(m.data)}), 10)
        self.create_subscription(Empty, "/excavator/reset", lambda m: self.send({"type": "reset"}), 10)
        self.sock, self.lock, self.hello = None, threading.Lock(), None
        threading.Thread(target=self.receive_loop, daemon=True).start()

    # ------------------------------------------------------------------ connection
    def send(self, msg):
        body = json.dumps(msg).encode() + b"\n"
        with self.lock:
            if self.sock is not None:
                try:
                    self.sock.sendall(struct.pack("<I", len(body)) + body)
                except OSError:
                    pass

    def recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("simulator closed the connection")
            buf += chunk
        return bytes(buf)

    def read(self):
        n = struct.unpack("<I", self.recv_exact(4))[0]
        head, raw = self.recv_exact(n).split(b"\n", 1)
        header, arrays, off = json.loads(head), {}, 0
        for a in header["arrays"]:
            dt = np.dtype(a["dtype"])
            count = int(np.prod(a["shape"])) if a["shape"] else 1
            arrays[a["name"]] = np.frombuffer(raw, dt, count, off).reshape(a["shape"])
            off += count * dt.itemsize
        return header, arrays

    def receive_loop(self):
        while rclpy.ok():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=5.0)
                sock.settimeout(None)
                with self.lock:
                    self.sock = sock
                self.get_logger().info(f"connected to the simulator at {self.host}:{self.port}")
                while rclpy.ok():
                    header, arrays = self.read()
                    getattr(self, "on_" + header["type"])(header, arrays)
            except (OSError, ConnectionError) as e:
                with self.lock:
                    self.sock = None
                self.get_logger().warn(f"simulator not reachable ({e}); retrying", throttle_duration_sec=10.0)
                threading.Event().wait(1.0)

    # ------------------------------------------------------------------ messages
    def on_hello(self, h, _):
        self.hello = h
        m = h["mounts"]
        static = [transform("body_link", "lidar_link", 0.0, m["lidar"], (0, 0, 0, 1)),
                  transform("body_link", "camera_link", 0.0, m["camera"], (0, 0, 0, 1)),
                  transform("body_link", "camera_optical_frame", 0.0, m["camera"], m["camera_q"]),
                  transform("body_link", "imu_link", 0.0, m["imu"], (0, 0, 0, 1))]
        self.pub["tf_static"].publish(TFMessage(transforms=static))
        self.get_logger().info(f"simulator: soil {h['soil']}, rates {h['rates']}")

    def on_state(self, h, a):
        t = h["t"]
        self.pub["clock"].publish(Clock(clock=stamp(t)))
        js = JointState()
        js.header.stamp = stamp(t)
        js.name = list(self.hello["joints"]) if self.hello else ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
        js.position, js.velocity, js.effort = a["q"].tolist(), a["qd"].tolist(), a["pressure"].tolist()
        self.pub["joints"].publish(js)
        self.pub["tf"].publish(TFMessage(transforms=[transform("world", "base_link", t, a["base_p"], a["base_q"]),
                                                     transform("world", "body_link", t, a["body_p"], a["body_q"])]))
        self.pub["levers"].publish(Float64MultiArray(data=a["levers"].tolist()))
        tip = PointStamped()
        tip.header.stamp, tip.header.frame_id = stamp(t), "world"
        tip.point.x, tip.point.y, tip.point.z = (float(v) for v in a["bucket_tip"])
        self.pub["tip"].publish(tip)
        f = Vector3Stamped()
        f.header.stamp, f.header.frame_id = stamp(t), "world"
        f.vector.x, f.vector.y, f.vector.z = (float(v) for v in a["soil_force"])
        self.pub["force"].publish(f)
        self.pub["bucket"].publish(Float64(data=float(a["loads"][0])))
        self.pub["truck"].publish(Float64(data=float(a["loads"][1])))
        self.pub["stage"].publish(String(data=h["stage"]))

    def on_imu(self, h, a):
        m = Imu()
        m.header.stamp, m.header.frame_id = stamp(h["t"]), "imu_link"
        m.orientation_covariance[0] = -1.0  # no orientation estimate
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (float(v) for v in a["angular_velocity"])
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (float(v) for v in a["linear_acceleration"])
        self.pub["imu"].publish(m)

    def on_lidar(self, h, a):
        pts, labels = a["points"].astype(np.float32), a["labels"].astype(np.uint8)
        rec = np.zeros(len(pts), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("label", "u1"), ("pad", "u1", 3)])
        rec["x"], rec["y"], rec["z"], rec["label"] = pts[:, 0], pts[:, 1], pts[:, 2], labels
        m = PointCloud2()
        m.header.stamp, m.header.frame_id = stamp(h["t"]), "lidar_link"
        m.height, m.width = 1, len(pts)
        m.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name="label", offset=12, datatype=PointField.UINT8, count=1)]
        m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 16, 16 * len(pts), True
        m.data = rec.tobytes()
        self.pub["points"].publish(m)

    def on_camera(self, h, a):
        depth, labels = a["depth"].astype(np.float32), a["labels"].astype(np.uint8)
        hgt, wid = depth.shape
        st = stamp(h["t"])
        for key, img, enc, step in (("depth", depth, "32FC1", wid * 4), ("labels", labels, "mono8", wid)):
            m = Image()
            m.header.stamp, m.header.frame_id = st, "camera_optical_frame"
            m.height, m.width, m.encoding, m.is_bigendian, m.step = hgt, wid, enc, 0, step
            m.data = img.tobytes()
            self.pub[key].publish(m)
        K = np.array(self.hello["camera"]["K"]) if self.hello else np.eye(3)
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = st, "camera_optical_frame"
        info.height, info.width, info.distortion_model = hgt, wid, "plumb_bob"
        info.d = [0.0] * 5
        info.k = K.ravel().tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub["info"].publish(info)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
