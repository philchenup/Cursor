#!/usr/bin/env python3
"""JAKA eye-in-hand 6D tracking with servo_j.

Vision thread: 30 Hz, updates the object pose in the base frame.
Control thread: 125 Hz / 8 ms, SE(3) interpolation -> IK -> servo_j(ABS).

Do not send the 30 Hz IK solution directly to servo_j. The controller does
not interpolate servo commands.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import jkrc

ABS = 0
INCR = 1
DT = 0.008
DEG2RAD = math.pi / 180.0


def _quat_mul(a, b):
    return (
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    )


def _quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def _quat_norm(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(c / n for c in q)


def _slerp(a, b, t):
    a = _quat_norm(a)
    b = _quat_norm(b)
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-c for c in b)
        dot = -dot
    if dot > 0.9995:
        return _quat_norm(tuple(x + t * (y - x) for x, y in zip(a, b)))
    theta = math.acos(min(1.0, max(-1.0, dot)))
    s = math.sin(theta)
    w1 = math.sin((1.0 - t) * theta) / s
    w2 = math.sin(t * theta) / s
    return _quat_norm(tuple(w1 * x + w2 * y for x, y in zip(a, b)))


def _rotate(q, v):
    qv = (0.0, v[0], v[1], v[2])
    r = _quat_mul(_quat_mul(q, qv), _quat_conj(q))
    return (r[1], r[2], r[3])


def _pose_mul(a, b):
    pa, qa = a
    pb, qb = b
    return (tuple(pa[i] + _rotate(qa, pb)[i] for i in range(3)), _quat_mul(qa, qb))


def _pose_inv(p):
    qinv = _quat_conj(p[1])
    t = _rotate(qinv, p[0])
    return (tuple(-c for c in t), qinv)


def _rpy_to_quat(rpy):
    rx, ry, rz = rpy
    cx, sx = math.cos(rx * 0.5), math.sin(rx * 0.5)
    cy, sy = math.cos(ry * 0.5), math.sin(ry * 0.5)
    cz, sz = math.cos(rz * 0.5), math.sin(rz * 0.5)
    # Fallback ZYX-style composition; prefer robot.rpy conversion in C++.
    return _quat_norm((
        cx * cy * cz + sx * sy * sz,
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz,
    ))


def _quat_to_rpy(q):
    w, x, y, z = _quat_norm(q)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        ry = math.copysign(math.pi / 2.0, sinp)
    else:
        ry = math.asin(sinp)
    rx = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    rz = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (rx, ry, rz)


def _cart_to_pose(cart):
    return (tuple(cart[0:3]), _rpy_to_quat(tuple(cart[3:6])))


def _pose_to_cart(pose):
    return list(pose[0]) + list(_quat_to_rpy(pose[1]))


def _step_toward(cur, tgt, max_dp, max_dth):
    dp = tuple(tgt[0][i] - cur[0][i] for i in range(3))
    dist = math.sqrt(sum(c * c for c in dp))
    dot = abs(sum(a * b for a, b in zip(_quat_norm(cur[1]), _quat_norm(tgt[1]))))
    ang = 2.0 * math.acos(min(1.0, max(-1.0, dot)))
    t = 1.0
    if dist > max_dp and dist > 1e-9:
        t = min(t, max_dp / dist)
    if ang > max_dth and ang > 1e-9:
        t = min(t, max_dth / ang)
    p = tuple(cur[0][i] + t * (tgt[0][i] - cur[0][i]) for i in range(3))
    return (p, _slerp(cur[1], tgt[1], t))


def _integrate(pose, v, w, dt):
    p = tuple(pose[0][i] + v[i] * dt for i in range(3))
    wn = math.sqrt(sum(c * c for c in w))
    q = pose[1]
    if wn > 1e-9:
        half = 0.5 * wn * dt
        s = math.sin(half) / wn
        dq = _quat_norm((math.cos(half), w[0] * s, w[1] * s, w[2] * s))
        q = _quat_mul(dq, q)
    return (p, q)


@dataclass
class Config:
    robot_ip: str = "192.168.137.101"
    t_tcp_cam: list = field(default_factory=lambda: [0.0, 0.0, 80.0, 0.0, 0.0, 0.0])
    vision_delay_s: float = 0.040
    vision_timeout_s: float = 0.150
    pose_alpha: float = 0.35
    twist_alpha: float = 0.25
    max_tcp_step_mm: float = 1.2
    max_tcp_step_rad: float = 40.0 * DEG2RAD / 125.0
    max_joint_step_rad: float = 80.0 * DEG2RAD / 125.0
    ik_jump_limit_rad: float = 15.0 * DEG2RAD
    nlf_vr: float = 80.0
    nlf_ar: float = 400.0
    nlf_jr: float = 2000.0


class Jaka6dServoJTracker:
    def __init__(self, robot: jkrc.RC, cfg: Config):
        self.robot = robot
        self.cfg = cfg
        self.T_tcp_cam = _cart_to_pose(cfg.t_tcp_cam)
        self.T_obj_tcp = None
        self.obj = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = None

    def on_vision(self, t_cam_obj, t_base_tcp=None, stamp=None):
        """Call at ~30 Hz. t_cam_obj is [x,y,z,rx,ry,rz] in the camera frame."""
        if t_base_tcp is None:
            ret = self.robot.get_tcp_position()
            if ret[0] != 0:
                return
            t_base_tcp = ret[1]
        stamp = time.monotonic() if stamp is None else stamp
        T_base_tcp = _cart_to_pose(t_base_tcp)
        T_cam_obj = _cart_to_pose(t_cam_obj)
        T_base_obj = _pose_mul(_pose_mul(T_base_tcp, self.T_tcp_cam), T_cam_obj)

        with self.lock:
            if self.obj is None:
                self.obj = {
                    "stamp": stamp,
                    "pose": T_base_obj,
                    "v": (0.0, 0.0, 0.0),
                    "w": (0.0, 0.0, 0.0),
                }
            else:
                dt = max(1e-4, stamp - self.obj["stamp"])
                v = tuple((T_base_obj[0][i] - self.obj["pose"][0][i]) / dt for i in range(3))
                self.obj["pose"] = (
                    tuple((1.0 - self.cfg.pose_alpha) * self.obj["pose"][0][i]
                          + self.cfg.pose_alpha * T_base_obj[0][i] for i in range(3)),
                    _slerp(self.obj["pose"][1], T_base_obj[1], self.cfg.pose_alpha),
                )
                self.obj["v"] = tuple(
                    (1.0 - self.cfg.twist_alpha) * self.obj["v"][i] + self.cfg.twist_alpha * v[i]
                    for i in range(3)
                )
                self.obj["stamp"] = stamp
            if self.T_obj_tcp is None:
                self.T_obj_tcp = _pose_inv(self.obj["pose"])
                self.T_obj_tcp = _pose_mul(self.T_obj_tcp, T_base_tcp)

    def start(self):
        # Filters must be set before servo_move_enable.
        self.robot.servo_move_use_joint_NLF(self.cfg.nlf_vr, self.cfg.nlf_ar, self.cfg.nlf_jr)
        ret = self.robot.servo_move_enable(True)
        if ret[0] != 0:
            raise RuntimeError(f"servo_move_enable failed: {ret}")
        self.stop.clear()
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()

    def stop_tracker(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=1.0)
        self.robot.servo_move_enable(False)

    def _control_loop(self):
        ret = self.robot.get_joint_position()
        q_cmd = list(ret[1])
        ret = self.robot.get_tcp_position()
        T_cmd = _cart_to_pose(ret[1])
        next_t = time.perf_counter()

        while not self.stop.is_set():
            next_t += DT
            now = time.monotonic()
            with self.lock:
                snap = None if self.obj is None else dict(self.obj)
                offset = self.T_obj_tcp

            T_des = T_cmd
            if snap is not None and offset is not None:
                if now - snap["stamp"] <= self.cfg.vision_timeout_s:
                    dt = now - snap["stamp"] + self.cfg.vision_delay_s
                    T_obj = _integrate(snap["pose"], snap["v"], snap["w"], dt)
                    T_des = _pose_mul(T_obj, offset)
                else:
                    with self.lock:
                        if self.obj is not None:
                            self.obj["v"] = tuple(0.85 * c for c in self.obj["v"])
                            self.obj["w"] = tuple(0.85 * c for c in self.obj["w"])

            T_cmd = _step_toward(T_cmd, T_des, self.cfg.max_tcp_step_mm, self.cfg.max_tcp_step_rad)
            cart = _pose_to_cart(T_cmd)
            ik = self.robot.kine_inverse(q_cmd, cart)
            if ik[0] == 0:
                q_ik = list(ik[1])
                ok = True
                q_lim = q_cmd[:]
                for i in range(6):
                    dq = q_ik[i] - q_cmd[i]
                    if abs(dq) > self.cfg.ik_jump_limit_rad:
                        ok = False
                        break
                    q_lim[i] = q_cmd[i] + max(
                        -self.cfg.max_joint_step_rad, min(self.cfg.max_joint_step_rad, dq)
                    )
                if ok:
                    q_cmd = q_lim

            ret = self.robot.servo_j(q_cmd, ABS, 1)
            if ret[0] != 0:
                print("servo_j error", ret)
                break

            remain = next_t - time.perf_counter()
            if remain > 0:
                time.sleep(remain)


def main():
    cfg = Config()
    robot = jkrc.RC(cfg.robot_ip)
    robot.login()
    robot.power_on()
    robot.enable_robot()
    robot.set_rapidrate(1.0)
    robot.joint_move([0.0, -0.6, 1.4, -0.8, 1.57, 0.0], ABS, True, 0.4)

    tracker = Jaka6dServoJTracker(robot, cfg)
    tracker.start()
    print("servo_j 6D tracking running. Ctrl+C to stop.")
    try:
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            # Replace this block with your 30 Hz 6D estimator (T_cam_obj).
            obj = [
                400.0 + 40.0 * math.cos(0.4 * t),
                40.0 * math.sin(0.4 * t),
                150.0,
                0.0,
                0.0,
                0.15 * math.sin(0.3 * t),
            ]
            ret = robot.get_tcp_position()
            if ret[0] == 0:
                # Demo only: invert the simulated base pose into the camera frame.
                T_base_tcp = _cart_to_pose(ret[1])
                T_base_obj = _cart_to_pose(obj)
                T_cam_obj = _pose_mul(_pose_inv(_pose_mul(T_base_tcp, tracker.T_tcp_cam)), T_base_obj)
                tracker.on_vision(_pose_to_cart(T_cam_obj), ret[1])
            time.sleep(1.0 / 30.0)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.stop_tracker()
        robot.disable_robot()
        robot.logout()


if __name__ == "__main__":
    main()
