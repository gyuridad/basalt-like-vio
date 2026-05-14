#!/usr/bin/env python3

import json
import math
import os
import socket
import time
import traceback
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any, Deque, Dict, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


DEFAULT_DEBUG_BASE_DIR = os.path.expanduser(
    "~/isaac_ros2_ws/IsaacSim-ros_workspaces/humble_ws/src/carter_project/debug_runs"
)


class ReasonCode:
    class COMMON:
        NODE_STARTED = "COMMON.NODE_STARTED"
        NODE_STOPPED = "COMMON.NODE_STOPPED"
        EXCEPTION = "COMMON.EXCEPTION"
        CONFIG_SNAPSHOT_WRITTEN = "COMMON.CONFIG_SNAPSHOT_WRITTEN"
        HEARTBEAT = "COMMON.HEARTBEAT"

    class INTERFACE:
        TOPIC_NOT_FOUND = "INTERFACE.TOPIC_NOT_FOUND"
        TOPIC_NO_PUBLISHER = "INTERFACE.TOPIC_NO_PUBLISHER"
        MESSAGE_TYPE_MISMATCH = "INTERFACE.MESSAGE_TYPE_MISMATCH"
        INTERFACE_OK = "INTERFACE.INTERFACE_OK"

    class IMU:
        SAMPLE_RECEIVED = "IMU.SAMPLE_RECEIVED"
        WINDOW_SUMMARY = "IMU.WINDOW_SUMMARY"
        GRAVITY_RAW_ACCEL_SUSPECTED = "IMU.GRAVITY_RAW_ACCEL_SUSPECTED"
        LINEAR_ACCEL_SUSPECTED = "IMU.LINEAR_ACCEL_SUSPECTED"
        TIMESTAMP_NON_MONOTONIC = "IMU.TIMESTAMP_NON_MONOTONIC"
        LOW_SAMPLE_COUNT = "IMU.LOW_SAMPLE_COUNT"
        ODOM_LOW_SAMPLE_COUNT = "IMU.ODOM_LOW_SAMPLE_COUNT"


def now_sec() -> float:
    return time.time()


def stamp_to_sec(msg: Imu) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def vec3(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return (float(x), float(y), float(z))


def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_mul(a, s: float):
    return (a[0] * s, a[1] * s, a[2] * s)


def v_dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def v_norm(a) -> float:
    return math.sqrt(v_dot(a, a))


def q_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_normalize(q):
    n = math.sqrt(sum(x * x for x in q))
    if n <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(x / n for x in q)


def q_exp_so3(phi):
    theta = v_norm(phi)
    if theta < 1e-12:
        return q_normalize((1.0, 0.5 * phi[0], 0.5 * phi[1], 0.5 * phi[2]))
    axis = v_mul(phi, 1.0 / theta)
    half = 0.5 * theta
    s = math.sin(half)
    return q_normalize((math.cos(half), axis[0] * s, axis[1] * s, axis[2] * s))


def q_rotate(q, v):
    w, x, y, z = q
    qv = (x, y, z)
    t = v_mul(v_cross(qv, v), 2.0)
    return v_add(v_add(v, v_mul(t, w)), v_cross(qv, t))


def q_to_yaw(q) -> float:
    w, x, y, z = q_normalize(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def stddev(values: List[float]) -> float:
    if len(values) < 2:
        return 0.0
    m = mean(values)
    return math.sqrt(sum((x - m) * (x - m) for x in values) / (len(values) - 1))


def axis_stats(samples: List[Tuple[float, float, float]]) -> Dict[str, List[float]]:
    if not samples:
        return {"mean": [0.0, 0.0, 0.0], "std": [0.0, 0.0, 0.0], "min": [0.0, 0.0, 0.0], "max": [0.0, 0.0, 0.0]}
    axes = [[s[i] for s in samples] for i in range(3)]
    return {
        "mean": [mean(a) for a in axes],
        "std": [stddev(a) for a in axes],
        "min": [min(a) for a in axes],
        "max": [max(a) for a in axes],
    }


def jsonable(obj: Any) -> Any:
    if obj is None or isinstance(obj, (str, int, float, bool)):
        return obj
    if isinstance(obj, tuple):
        return [jsonable(x) for x in obj]
    if isinstance(obj, list):
        return [jsonable(x) for x in obj]
    if isinstance(obj, dict):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if hasattr(obj, "__dataclass_fields__"):
        return jsonable(asdict(obj))
    return str(obj)


@dataclass
class ImuSample:
    stamp_sec: float
    recv_sec: float
    frame_id: str
    accel: Tuple[float, float, float]
    gyro: Tuple[float, float, float]


@dataclass
class OdomSample:
    stamp_sec: float
    recv_sec: float
    frame_id: str
    child_frame_id: str
    position: Tuple[float, float, float]
    orientation_q: Tuple[float, float, float, float]
    linear_velocity: Tuple[float, float, float]
    angular_velocity: Tuple[float, float, float]


class LDADebugger:
    def __init__(
        self,
        node_name: str,
        robot_name: str = "carter",
        namespace: str = "/",
        base_dir: str = DEFAULT_DEBUG_BASE_DIR,
        also_print: bool = True,
    ):
        self.node_name = node_name
        self.robot_name = robot_name
        self.namespace = namespace or "/"
        self.base_dir = os.path.expanduser(base_dir)
        self.also_print = also_print
        self.run_id = time.strftime("%Y%m%d_%H%M%S")
        self.hostname = socket.gethostname()
        self.pid = os.getpid()
        self.start_time = now_sec()

        self.run_dir = os.path.join(self.base_dir, self.run_id)
        self.log_dir = os.path.join(self.run_dir, "logs")
        self.snapshot_dir = os.path.join(self.run_dir, "snapshots")
        self.interface_dir = os.path.join(self.run_dir, "interfaces")
        self.meta_dir = os.path.join(self.run_dir, "meta")
        for path in [self.log_dir, self.snapshot_dir, self.interface_dir, self.meta_dir]:
            os.makedirs(path, exist_ok=True)

        self.log_path = os.path.join(self.log_dir, f"{node_name}.jsonl")
        self.sample_path = os.path.join(self.log_dir, f"{node_name}_samples.jsonl")
        self.odom_sample_path = os.path.join(self.log_dir, f"{node_name}_odom_samples.jsonl")
        self.snapshot_path = os.path.join(self.snapshot_dir, f"{node_name}.json")
        self.interface_path = os.path.join(self.interface_dir, f"{node_name}.json")
        self.meta_path = os.path.join(self.meta_dir, f"{node_name}.json")
        self.write_json(
            self.meta_path,
            {
                "node_name": node_name,
                "robot_name": robot_name,
                "namespace": self.namespace,
                "hostname": self.hostname,
                "pid": self.pid,
                "run_id": self.run_id,
                "start_time": self.start_time,
                "base_dir": self.base_dir,
            },
        )

    def append_jsonl(self, path: str, record: Dict[str, Any]):
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(jsonable(record), ensure_ascii=False) + "\n")

    def write_json(self, path: str, payload: Dict[str, Any]):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(jsonable(payload), f, ensure_ascii=False, indent=2)

    def emit_event(self, level: str, event: str, reason_code: str, data: Optional[Dict[str, Any]] = None):
        record = {
            "ts": now_sec(),
            "node": self.node_name,
            "robot": self.robot_name,
            "namespace": self.namespace,
            "run_id": self.run_id,
            "level": level,
            "event": event,
            "reason_code": reason_code,
            "data": data or {},
        }
        self.append_jsonl(self.log_path, record)
        if self.also_print:
            print(f"[{level}] [{self.node_name}] event={event} reason={reason_code} data={record['data']}")

    def emit_sample(self, sample: ImuSample, index: int):
        self.append_jsonl(
            self.sample_path,
            {
                "ts": now_sec(),
                "index": index,
                "stamp_sec": sample.stamp_sec,
                "recv_sec": sample.recv_sec,
                "frame_id": sample.frame_id,
                "accel": sample.accel,
                "gyro": sample.gyro,
                "accel_norm": v_norm(sample.accel),
                "gyro_norm": v_norm(sample.gyro),
            },
        )

    def emit_odom_sample(self, sample: OdomSample, index: int):
        self.append_jsonl(
            self.odom_sample_path,
            {
                "ts": now_sec(),
                "index": index,
                "stamp_sec": sample.stamp_sec,
                "recv_sec": sample.recv_sec,
                "frame_id": sample.frame_id,
                "child_frame_id": sample.child_frame_id,
                "position": sample.position,
                "orientation_q": sample.orientation_q,
                "yaw_rad": q_to_yaw(sample.orientation_q),
                "linear_velocity": sample.linear_velocity,
                "linear_velocity_w_from_body": q_rotate(sample.orientation_q, sample.linear_velocity),
                "angular_velocity": sample.angular_velocity,
            },
        )

    def write_snapshot(self, snapshot: Dict[str, Any]):
        self.write_json(
            self.snapshot_path,
            {
                "ts": now_sec(),
                "uptime_sec": now_sec() - self.start_time,
                "run_id": self.run_id,
                "snapshot": snapshot,
            },
        )

    def write_interface_snapshot(self, snapshot: Dict[str, Any]):
        self.write_json(
            self.interface_path,
            {
                "ts": now_sec(),
                "run_id": self.run_id,
                "interface_snapshot": snapshot,
            },
        )
        self.emit_event(
            "INFO",
            "interface_snapshot_written",
            ReasonCode.COMMON.CONFIG_SNAPSHOT_WRITTEN,
            {"interface_path": self.interface_path},
        )

    def emit_exception(self, where: str, exc: Exception):
        self.emit_event(
            "ERROR",
            "exception",
            ReasonCode.COMMON.EXCEPTION,
            {
                "where": where,
                "exc_type": type(exc).__name__,
                "exc_msg": str(exc),
                "traceback": traceback.format_exc(),
            },
        )


class IsaacImuLdaDebugNode(Node):
    def __init__(self):
        super().__init__("isaac_imu_lda_debug_node")

        self.declare_parameter("imu_topic", "/chassis/imu")
        self.declare_parameter("odom_topic", "/chassis/odom")
        self.declare_parameter("window_sec", 1.0)
        self.declare_parameter("summary_every_sec", 1.0)
        self.declare_parameter("gravity_z", -9.81)
        self.declare_parameter("lda_debug_dir", DEFAULT_DEBUG_BASE_DIR)
        self.declare_parameter("write_samples", False)
        self.declare_parameter("sample_stride", 1)
        self.declare_parameter("also_print", True)

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.window_sec = float(self.get_parameter("window_sec").value)
        self.summary_every_sec = float(self.get_parameter("summary_every_sec").value)
        self.gravity = (0.0, 0.0, float(self.get_parameter("gravity_z").value))
        self.write_samples = bool(self.get_parameter("write_samples").value)
        self.sample_stride = max(1, int(self.get_parameter("sample_stride").value))

        self.debugger = LDADebugger(
            node_name=self.get_name(),
            robot_name="carter",
            namespace=self.get_namespace(),
            base_dir=str(self.get_parameter("lda_debug_dir").value),
            also_print=bool(self.get_parameter("also_print").value),
        )

        self.samples: Deque[ImuSample] = deque()
        self.odom_samples: Deque[OdomSample] = deque()
        self.total_samples = 0
        self.total_odom_samples = 0
        self.last_summary_wall_sec = 0.0
        self.last_stamp_sec: Optional[float] = None
        self.last_odom_stamp_sec: Optional[float] = None

        self.sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.on_imu,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.on_odom,
            qos_profile_sensor_data,
        )

        self.debugger.emit_event(
            "INFO",
            "node_started",
            ReasonCode.COMMON.NODE_STARTED,
            {
                "imu_topic": self.imu_topic,
                "odom_topic": self.odom_topic,
                "window_sec": self.window_sec,
                "summary_every_sec": self.summary_every_sec,
                "gravity": self.gravity,
                "write_samples": self.write_samples,
                "sample_stride": self.sample_stride,
                "run_dir": self.debugger.run_dir,
            },
        )
        self.write_interface_snapshot()
        self.create_timer(2.0, self.check_interface)

    def write_interface_snapshot(self):
        topics = {name: types for name, types in self.get_topic_names_and_types()}
        self.debugger.write_interface_snapshot(
            {
                "node_name": self.get_name(),
                "namespace": self.get_namespace(),
                "subscribes": [
                    {
                        "name": self.imu_topic,
                        "msg_type": "sensor_msgs/msg/Imu",
                        "required": True,
                        "qos": "sensor_data",
                    },
                    {
                        "name": self.odom_topic,
                        "msg_type": "nav_msgs/msg/Odometry",
                        "required": True,
                        "qos": "sensor_data",
                    }
                ],
                "params": {
                    "imu_topic": self.imu_topic,
                    "odom_topic": self.odom_topic,
                    "window_sec": self.window_sec,
                    "summary_every_sec": self.summary_every_sec,
                    "gravity": self.gravity,
                },
                "observed_topics": topics,
            }
        )

    def check_interface(self):
        topics = {name: types for name, types in self.get_topic_names_and_types()}
        self.check_one_topic(self.imu_topic, "sensor_msgs/msg/Imu")
        self.check_one_topic(self.odom_topic, "nav_msgs/msg/Odometry")

    def check_one_topic(self, topic: str, expected_type: str):
        topics = {name: types for name, types in self.get_topic_names_and_types()}
        actual_types = topics.get(topic)
        if actual_types is None:
            self.debugger.emit_event(
                "WARN",
                "interface_check",
                ReasonCode.INTERFACE.TOPIC_NOT_FOUND,
                {"expected_topic": topic, "expected_type": expected_type, "observed_topic_count": len(topics)},
            )
            return
        if expected_type not in actual_types:
            self.debugger.emit_event(
                "ERROR",
                "interface_check",
                ReasonCode.INTERFACE.MESSAGE_TYPE_MISMATCH,
                {"topic": topic, "actual_types": actual_types, "expected_type": expected_type},
            )
            return
        publisher_count = self.count_publishers(topic)
        level = "INFO" if publisher_count > 0 else "WARN"
        reason = ReasonCode.INTERFACE.INTERFACE_OK if publisher_count > 0 else ReasonCode.INTERFACE.TOPIC_NO_PUBLISHER
        self.debugger.emit_event(
            level,
            "interface_check",
            reason,
            {"topic": topic, "actual_types": actual_types, "publisher_count": publisher_count},
        )

    def on_imu(self, msg: Imu):
        try:
            recv_sec = self.get_clock().now().nanoseconds * 1e-9
            sample = ImuSample(
                stamp_sec=stamp_to_sec(msg),
                recv_sec=recv_sec,
                frame_id=str(msg.header.frame_id),
                accel=vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
                gyro=vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            )

            if self.last_stamp_sec is not None and sample.stamp_sec <= self.last_stamp_sec:
                self.debugger.emit_event(
                    "WARN",
                    "timestamp_non_monotonic",
                    ReasonCode.IMU.TIMESTAMP_NON_MONOTONIC,
                    {"last_stamp_sec": self.last_stamp_sec, "current_stamp_sec": sample.stamp_sec},
                )
            self.last_stamp_sec = sample.stamp_sec

            self.samples.append(sample)
            self.total_samples += 1

            while self.samples and sample.stamp_sec - self.samples[0].stamp_sec > self.window_sec:
                self.samples.popleft()

            if self.write_samples and self.total_samples % self.sample_stride == 0:
                self.debugger.emit_sample(sample, self.total_samples)

            now = now_sec()
            if now - self.last_summary_wall_sec >= self.summary_every_sec:
                self.last_summary_wall_sec = now
                self.emit_window_summary()
        except Exception as exc:
            self.debugger.emit_exception("on_imu", exc)

    def on_odom(self, msg: Odometry):
        try:
            recv_sec = self.get_clock().now().nanoseconds * 1e-9
            q = msg.pose.pose.orientation
            sample = OdomSample(
                stamp_sec=stamp_to_sec(msg),
                recv_sec=recv_sec,
                frame_id=str(msg.header.frame_id),
                child_frame_id=str(msg.child_frame_id),
                position=vec3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                orientation_q=(float(q.w), float(q.x), float(q.y), float(q.z)),
                linear_velocity=vec3(
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ),
                angular_velocity=vec3(
                    msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z,
                ),
            )

            if self.last_odom_stamp_sec is not None and sample.stamp_sec <= self.last_odom_stamp_sec:
                self.debugger.emit_event(
                    "WARN",
                    "odom_timestamp_non_monotonic",
                    ReasonCode.IMU.TIMESTAMP_NON_MONOTONIC,
                    {"last_stamp_sec": self.last_odom_stamp_sec, "current_stamp_sec": sample.stamp_sec},
                )
            self.last_odom_stamp_sec = sample.stamp_sec

            self.odom_samples.append(sample)
            self.total_odom_samples += 1

            latest_ref_sec = self.samples[-1].stamp_sec if self.samples else sample.stamp_sec
            while self.odom_samples and latest_ref_sec - self.odom_samples[0].stamp_sec > self.window_sec:
                self.odom_samples.popleft()

            if self.write_samples and self.total_odom_samples % self.sample_stride == 0:
                self.debugger.emit_odom_sample(sample, self.total_odom_samples)
        except Exception as exc:
            self.debugger.emit_exception("on_odom", exc)

    def integrate_window(self, samples: List[ImuSample]) -> Dict[str, Any]:
        q = (1.0, 0.0, 0.0, 0.0)
        raw_delta_v = (0.0, 0.0, 0.0)
        raw_delta_p = (0.0, 0.0, 0.0)
        lda_delta_v = (0.0, 0.0, 0.0)
        lda_delta_p = (0.0, 0.0, 0.0)
        lda_accels = []
        dts = []

        for s0, s1 in zip(samples[:-1], samples[1:]):
            dt = s1.stamp_sec - s0.stamp_sec
            if dt <= 0.0:
                continue
            gyro = v_mul(v_add(s0.gyro, s1.gyro), 0.5)
            accel_b = v_mul(v_add(s0.accel, s1.accel), 0.5)
            accel_w = q_rotate(q, accel_b)
            lda_accel_w = v_add(accel_w, self.gravity)

            raw_delta_p = v_add(raw_delta_p, v_add(v_mul(raw_delta_v, dt), v_mul(accel_w, 0.5 * dt * dt)))
            raw_delta_v = v_add(raw_delta_v, v_mul(accel_w, dt))
            lda_delta_p = v_add(lda_delta_p, v_add(v_mul(lda_delta_v, dt), v_mul(lda_accel_w, 0.5 * dt * dt)))
            lda_delta_v = v_add(lda_delta_v, v_mul(lda_accel_w, dt))

            q = q_normalize(q_mul(q, q_exp_so3(v_mul(gyro, dt))))
            lda_accels.append(lda_accel_w)
            dts.append(dt)

        return {
            "dt": {
                "count": len(dts),
                "mean": mean(dts),
                "std": stddev(dts),
                "min": min(dts) if dts else 0.0,
                "max": max(dts) if dts else 0.0,
                "total": sum(dts),
            },
            "raw_delta_velocity_w": raw_delta_v,
            "raw_delta_position_w": raw_delta_p,
            "lda_delta_velocity_w": lda_delta_v,
            "lda_delta_position_w": lda_delta_p,
            "lda_accel_w_stats": axis_stats(lda_accels),
            "delta_rotation_q": q,
        }

    def odom_window_summary(self, start_sec: float, end_sec: float, integration: Dict[str, Any]) -> Dict[str, Any]:
        odom_samples = [s for s in self.odom_samples if start_sec <= s.stamp_sec <= end_sec]
        if len(odom_samples) < 2:
            return {
                "valid": False,
                "reason_code": ReasonCode.IMU.ODOM_LOW_SAMPLE_COUNT,
                "odom_sample_count": len(odom_samples),
                "total_odom_samples": self.total_odom_samples,
            }

        first = odom_samples[0]
        last = odom_samples[-1]
        dt = last.stamp_sec - first.stamp_sec
        odom_delta_p = v_sub(last.position, first.position)
        first_linear_velocity_w = q_rotate(first.orientation_q, first.linear_velocity)
        last_linear_velocity_w = q_rotate(last.orientation_q, last.linear_velocity)
        odom_delta_v_w = v_sub(last_linear_velocity_w, first_linear_velocity_w)
        odom_delta_yaw = wrap_angle(q_to_yaw(last.orientation_q) - q_to_yaw(first.orientation_q))
        q_delta = q_mul(q_conj(first.orientation_q), last.orientation_q)

        # IMU preintegration delta_p does not include the initial velocity term.
        odom_delta_p_minus_v0_dt = v_sub(odom_delta_p, v_mul(first_linear_velocity_w, dt))

        imu_delta_p_local = tuple(integration["lda_delta_position_w"])
        imu_delta_v_local = tuple(integration["lda_delta_velocity_w"])
        imu_delta_p_w = q_rotate(first.orientation_q, imu_delta_p_local)
        imu_delta_v_w = q_rotate(first.orientation_q, imu_delta_v_local)
        imu_delta_yaw = q_to_yaw(tuple(integration["delta_rotation_q"]))

        return {
            "valid": True,
            "odom_sample_count": len(odom_samples),
            "total_odom_samples": self.total_odom_samples,
            "frame_id": first.frame_id,
            "child_frame_id": first.child_frame_id,
            "start_stamp_sec": first.stamp_sec,
            "end_stamp_sec": last.stamp_sec,
            "dt": dt,
            "start_position_w": first.position,
            "end_position_w": last.position,
            "odom_delta_position_w": odom_delta_p,
            "odom_delta_position_minus_v0_dt_w": odom_delta_p_minus_v0_dt,
            "odom_delta_velocity_w": odom_delta_v_w,
            "odom_start_linear_velocity_body": first.linear_velocity,
            "odom_end_linear_velocity_body": last.linear_velocity,
            "odom_start_linear_velocity_w": first_linear_velocity_w,
            "odom_end_linear_velocity_w": last_linear_velocity_w,
            "odom_twist_linear_frame_assumption": "child_frame_id/body; rotated into frame_id/world with pose.orientation",
            "odom_start_angular_velocity": first.angular_velocity,
            "odom_end_angular_velocity": last.angular_velocity,
            "odom_delta_yaw_rad": odom_delta_yaw,
            "odom_delta_yaw_deg": odom_delta_yaw * 180.0 / math.pi,
            "odom_delta_rotation_q": q_delta,
            "imu_lda_delta_position_local": imu_delta_p_local,
            "imu_lda_delta_velocity_local": imu_delta_v_local,
            "imu_lda_delta_position_w": imu_delta_p_w,
            "imu_lda_delta_velocity_w": imu_delta_v_w,
            "imu_lda_delta_frame_assumption": "window_start_imu/body; rotated into frame_id/world with odom start orientation",
            "imu_delta_yaw_rad": imu_delta_yaw,
            "imu_delta_yaw_deg": imu_delta_yaw * 180.0 / math.pi,
            "compare_delta_position_minus_v0_dt_error_w": v_sub(imu_delta_p_w, odom_delta_p_minus_v0_dt),
            "compare_delta_velocity_error_w": v_sub(imu_delta_v_w, odom_delta_v_w),
            "compare_delta_yaw_error_rad": wrap_angle(imu_delta_yaw - odom_delta_yaw),
        }

    def classify_accel_semantics(self, accel_mean, lda_mean) -> Tuple[str, str]:
        accel_norm = v_norm(accel_mean)
        lda_norm = v_norm(lda_mean)
        if abs(accel_norm - abs(self.gravity[2])) < 1.0 and lda_norm < 1.0:
            return ("raw_accel_includes_gravity", ReasonCode.IMU.GRAVITY_RAW_ACCEL_SUSPECTED)
        if accel_norm < 1.0:
            return ("linear_accel_already_gravity_compensated", ReasonCode.IMU.LINEAR_ACCEL_SUSPECTED)
        return ("unknown_or_moving", ReasonCode.IMU.WINDOW_SUMMARY)

    def emit_window_summary(self):
        samples = list(self.samples)
        if len(samples) < 2:
            self.debugger.emit_event(
                "WARN",
                "imu_window_summary",
                ReasonCode.IMU.LOW_SAMPLE_COUNT,
                {"sample_count": len(samples), "total_samples": self.total_samples},
            )
            return

        accels = [s.accel for s in samples]
        gyros = [s.gyro for s in samples]
        accel_stats = axis_stats(accels)
        gyro_stats = axis_stats(gyros)
        accel_mean = tuple(accel_stats["mean"])
        integration = self.integrate_window(samples)
        odom_compare = self.odom_window_summary(samples[0].stamp_sec, samples[-1].stamp_sec, integration)
        lda_mean = tuple(integration["lda_accel_w_stats"]["mean"])
        semantic, semantic_reason = self.classify_accel_semantics(accel_mean, lda_mean)

        summary = {
            "sample_count": len(samples),
            "total_samples": self.total_samples,
            "odom_sample_count": odom_compare.get("odom_sample_count", 0),
            "total_odom_samples": self.total_odom_samples,
            "window_start_stamp_sec": samples[0].stamp_sec,
            "window_end_stamp_sec": samples[-1].stamp_sec,
            "frame_ids": sorted(set(s.frame_id for s in samples)),
            "accel_b_stats": accel_stats,
            "gyro_b_stats": gyro_stats,
            "accel_norm_mean": mean([v_norm(a) for a in accels]),
            "gyro_norm_mean": mean([v_norm(g) for g in gyros]),
            "gravity": self.gravity,
            "semantic_guess": semantic,
            "integration": integration,
            "odom_compare": odom_compare,
            "debug_paths": {
                "run_dir": self.debugger.run_dir,
                "log_path": self.debugger.log_path,
                "sample_path": self.debugger.sample_path if self.write_samples else None,
                "odom_sample_path": self.debugger.odom_sample_path if self.write_samples else None,
                "snapshot_path": self.debugger.snapshot_path,
            },
        }

        level = "INFO" if semantic_reason != ReasonCode.IMU.WINDOW_SUMMARY else "WARN"
        self.debugger.emit_event(level, "imu_window_summary", semantic_reason, summary)
        self.debugger.write_snapshot(summary)


def main(args=None):
    rclpy.init(args=args)
    node = IsaacImuLdaDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.debugger.emit_exception("main", exc)
        raise
    finally:
        node.debugger.emit_event("INFO", "node_stopped", ReasonCode.COMMON.NODE_STOPPED, {})
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# 실행예시
# ros2 run basalt_like_vio isaac_imu_lda_debug_node.py

# ros2 run basalt_like_vio isaac_imu_lda_debug_node.py --ros-args \
#   -p write_samples:=true \
#   -p sample_stride:=1




