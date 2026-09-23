#!/usr/bin/env python3
"""UR eye-in-hand 6D tracking with servoJ (ur_rtde).

Same pipeline as the JAKA servo_j tracker:
  vision 30 Hz  -> object pose in the base frame
  control 125/500 Hz -> SE(3) interpolation -> IK -> servoJ

Do not use urx. It replaces the URScript program on every call and cannot
hold a native-rate servo stream.

Do not call rtde_c.getInverseKinematics() inside the servo loop. That RPC
waits for the controller and breaks 125/500 Hz timing. This file uses a
local damped-least-squares IK on UR DH + the active TCP offset.

UR pose units: [x, y, z, rx, ry, rz] with metres and rotation vector (rad).
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np

try:
    import rtde_control
    import rtde_receive
except ImportError:  # kinematics helpers still import without the robot SDK
    rtde_control = None
    rtde_receive = None

# ---------------------------------------------------------------------------
# UR DH (standard UR convention). Tool offset is measured at startup so the
# local FK matches getActualTCPPose(), including the TCP set on the teach pendant.
# ---------------------------------------------------------------------------
UR_DH = {
    "ur10": {
        "a": [0.0, -0.612, -0.5723, 0.0, 0.0, 0.0],
        "d": [0.1273, 0.0, 0.0, 0.163941, 0.1157, 0.0922],
        "alpha": [math.pi / 2.0, 0.0, 0.0, math.pi / 2.0, -math.pi / 2.0, 0.0],
    },
    "ur10e": {
        "a": [0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0],
        "d": [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655],
        "alpha": [math.pi / 2.0, 0.0, 0.0, math.pi / 2.0, -math.pi / 2.0, 0.0],
    },
    "ur5": {
        "a": [0.0, -0.425, -0.39225, 0.0, 0.0, 0.0],
        "d": [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823],
        "alpha": [math.pi / 2.0, 0.0, 0.0, math.pi / 2.0, -math.pi / 2.0, 0.0],
    },
    "ur5e": {
        "a": [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0],
        "d": [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996],
        "alpha": [math.pi / 2.0, 0.0, 0.0, math.pi / 2.0, -math.pi / 2.0, 0.0],
    },
}


def _dh(a, alpha, d, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def _quat_mul(a, b):
    return np.array(
        [
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
        ]
    )


def _quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_norm(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def _slerp(a, b, t):
    a = _quat_norm(a)
    b = _quat_norm(b)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    if dot > 0.9995:
        return _quat_norm(a + t * (b - a))
    theta = math.acos(min(1.0, max(-1.0, dot)))
    s = math.sin(theta)
    return _quat_norm(math.sin((1.0 - t) * theta) / s * a + math.sin(t * theta) / s * b)


def _rotate(q, v):
    qv = np.array([0.0, v[0], v[1], v[2]])
    r = _quat_mul(_quat_mul(q, qv), _quat_conj(q))
    return r[1:]


def rotvec_to_quat(rv):
    rv = np.asarray(rv, dtype=float)
    theta = float(np.linalg.norm(rv))
    if theta < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rv / theta
    s = math.sin(0.5 * theta)
    return np.array([math.cos(0.5 * theta), axis[0] * s, axis[1] * s, axis[2] * s])


def quat_to_rotvec(q):
    q = _quat_norm(q)
    if q[0] < 0.0:
        q = -q
    w = min(1.0, max(-1.0, float(q[0])))
    theta = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-8:
        return np.zeros(3)
    return theta * q[1:] / s


def pose_to_pq(pose):
    """UR [x,y,z,rx,ry,rz] -> (p metres, quat wxyz)."""
    pose = np.asarray(pose, dtype=float)
    return pose[:3].copy(), rotvec_to_quat(pose[3:])


def pq_to_pose(p, q):
    return np.concatenate([np.asarray(p, dtype=float), quat_to_rotvec(q)]).tolist()


def pq_mul(a, b):
    pa, qa = a
    pb, qb = b
    return pa + _rotate(qa, pb), _quat_mul(qa, qb)


def pq_inv(p, q):
    qi = _quat_conj(q)
    return -_rotate(qi, p), qi


def mat_to_pq(t):
    r = t[:3, :3]
    p = t[:3, 3]
    tr = float(np.trace(r))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = np.array([0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s])
    else:
        i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            q = np.array([(r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s])
        elif i == 1:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            q = np.array([(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s])
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            q = np.array([(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s])
    return p.copy(), _quat_norm(q)


def pq_to_mat(p, q):
    q = _quat_norm(q)
    w, x, y, z = q
    t = np.eye(4)
    t[0, 0] = 1.0 - 2.0 * (y * y + z * z)
    t[0, 1] = 2.0 * (x * y - z * w)
    t[0, 2] = 2.0 * (x * z + y * w)
    t[1, 0] = 2.0 * (x * y + z * w)
    t[1, 1] = 1.0 - 2.0 * (x * x + z * z)
    t[1, 2] = 2.0 * (y * z - x * w)
    t[2, 0] = 2.0 * (x * z - y * w)
    t[2, 1] = 2.0 * (y * z + x * w)
    t[2, 2] = 1.0 - 2.0 * (x * x + y * y)
    t[:3, 3] = p
    return t


def pose_error(p_des, q_des, p_cur, q_cur):
    """Cartesian error in the current TCP frame, size 6: [dp, rotvec]."""
    dp_base = p_des - p_cur
    dp = _rotate(_quat_conj(q_cur), dp_base)
    q_err = _quat_mul(q_des, _quat_conj(q_cur))
    if q_err[0] < 0.0:
        q_err = -q_err
    return np.concatenate([dp, quat_to_rotvec(q_err)])


def step_toward(p0, q0, p1, q1, max_dp, max_dth):
    dp = p1 - p0
    dist = float(np.linalg.norm(dp))
    dot = abs(float(np.dot(_quat_norm(q0), _quat_norm(q1))))
    ang = 2.0 * math.acos(min(1.0, max(-1.0, dot)))
    t = 1.0
    if dist > max_dp and dist > 1e-12:
        t = min(t, max_dp / dist)
    if ang > max_dth and ang > 1e-12:
        t = min(t, max_dth / ang)
    return p0 + t * dp, _slerp(q0, q1, t)


def integrate_pose(p, q, v, w, dt):
    p_n = p + v * dt
    wn = float(np.linalg.norm(w))
    if wn > 1e-9:
        half = 0.5 * wn * dt
        s = math.sin(half) / wn
        dq = _quat_norm(np.array([math.cos(half), w[0] * s, w[1] * s, w[2] * s]))
        q = _quat_mul(dq, q)
    return p_n, q


class UrKinematics:
    def __init__(self, model="ur10"):
        if model not in UR_DH:
            raise ValueError(f"unknown UR model {model}, expected one of {list(UR_DH)}")
        self.dh = UR_DH[model]
        self.t_flange_tcp = np.eye(4)

    def calibrate_tool(self, q, tcp_pose):
        """Make local FK match the controller TCP (active tool included)."""
        flange = self.fk_flange(q)
        tcp = pq_to_mat(*pose_to_pq(tcp_pose))
        self.t_flange_tcp = np.linalg.inv(flange) @ tcp

    def fk_flange(self, q):
        t = np.eye(4)
        for i in range(6):
            t = t @ _dh(self.dh["a"][i], self.dh["alpha"][i], self.dh["d"][i], q[i])
        return t

    def fk(self, q):
        return mat_to_pq(self.fk_flange(q) @ self.t_flange_tcp)

    def jacobian(self, q, h=1e-5):
        p0, q0 = self.fk(q)
        j = np.zeros((6, 6))
        q = np.asarray(q, dtype=float)
        for i in range(6):
            dq = q.copy()
            dq[i] += h
            p1, q1 = self.fk(dq)
            j[:, i] = pose_error(p1, q1, p0, q0) / h
        return j

    def ik(self, p_des, q_des, q_seed, iters=4, damping=0.04):
        q = np.asarray(q_seed, dtype=float).copy()
        for _ in range(iters):
            p, quat = self.fk(q)
            err = pose_error(p_des, q_des, p, quat)
            if float(np.linalg.norm(err)) < 1e-5:
                break
            jac = self.jacobian(q)
            jjt = jac @ jac.T + (damping ** 2) * np.eye(6)
            q = q + jac.T @ np.linalg.solve(jjt, err)
        return q.tolist()


@dataclass
class Config:
    robot_ip: str = "192.168.0.10"
    # cb3: 125 Hz / 8 ms (UR10). e: 500 Hz / 2 ms (UR10e / UR Series).
    series: str = "cb3"
    model: str = "ur10"
    # "servoJ" matches the JAKA servo_j design. "servoL" lets the controller IK.
    servo_mode: str = "servoJ"
    # Camera in TCP, UR pose: metres + rotation vector.
    t_tcp_cam: list = field(default_factory=lambda: [0.0, 0.0, 0.08, 0.0, 0.0, 0.0])
    lock_relative_pose_on_first_see: bool = True
    t_obj_tcp_desired: list = field(default_factory=lambda: [0.0, 0.0, -0.25, 0.0, math.pi, 0.0])
    vision_delay_s: float = 0.040
    vision_timeout_s: float = 0.150
    pose_alpha: float = 0.35
    twist_alpha: float = 0.25
    max_tcp_speed_m: float = 0.15
    max_ori_speed_rad: float = 40.0 * math.pi / 180.0
    max_joint_speed_rad: float = 80.0 * math.pi / 180.0
    ik_jump_limit_rad: float = 15.0 * math.pi / 180.0
    lookahead_time: float = 0.10
    gain: float = 300.0
    home_q: list = field(default_factory=lambda: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])

    @property
    def dt(self) -> float:
        return 0.002 if self.series.lower() in ("e", "eseries", "e-series") else 0.008


class Ur6dServoJTracker:
    def __init__(self, rtde_c, rtde_r, cfg: Config):
        self.c = rtde_c
        self.r = rtde_r
        self.cfg = cfg
        self.kin = UrKinematics(cfg.model)
        self.T_tcp_cam = pose_to_pq(cfg.t_tcp_cam)
        self.T_obj_tcp = None if cfg.lock_relative_pose_on_first_see else pose_to_pq(cfg.t_obj_tcp_desired)
        self.obj = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = None
        self.ready = False

    def on_vision(self, t_cam_obj, t_base_tcp=None, stamp=None):
        """Call at ~30 Hz. t_cam_obj is object-in-camera, UR pose list."""
        if not self.ready:
            return
        if t_base_tcp is None:
            t_base_tcp = self.r.getActualTCPPose()
        stamp = time.monotonic() if stamp is None else stamp
        p_tcp, q_tcp = pose_to_pq(t_base_tcp)
        p_co, q_co = pose_to_pq(t_cam_obj)
        p_obj, q_obj = pq_mul(pq_mul((p_tcp, q_tcp), self.T_tcp_cam), (p_co, q_co))

        with self.lock:
            if self.obj is None:
                self.obj = {
                    "stamp": stamp,
                    "p": p_obj,
                    "q": q_obj,
                    "v": np.zeros(3),
                    "w": np.zeros(3),
                }
            else:
                dt = max(1e-4, stamp - self.obj["stamp"])
                v_m = (p_obj - self.obj["p"]) / dt
                q_rel = _quat_mul(q_obj, _quat_conj(self.obj["q"]))
                w_m = quat_to_rotvec(q_rel) / dt
                a, b = self.cfg.pose_alpha, self.cfg.twist_alpha
                self.obj["p"] = (1.0 - a) * self.obj["p"] + a * p_obj
                self.obj["q"] = _slerp(self.obj["q"], q_obj, a)
                self.obj["v"] = (1.0 - b) * self.obj["v"] + b * v_m
                self.obj["w"] = (1.0 - b) * self.obj["w"] + b * w_m
                self.obj["stamp"] = stamp
            if self.T_obj_tcp is None:
                self.T_obj_tcp = pq_mul(pq_inv(self.obj["p"], self.obj["q"]), (p_tcp, q_tcp))

    def start(self):
        q0 = self.r.getActualQ()
        tcp0 = self.r.getActualTCPPose()
        self.kin.calibrate_tool(q0, tcp0)
        self.ready = True
        self.stop.clear()
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()

    def stop_tracker(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=1.0)
        try:
            self.c.servoStop()
        except Exception:
            pass
        self.ready = False

    def _control_loop(self):
        dt = self.cfg.dt
        max_dp = self.cfg.max_tcp_speed_m * dt
        max_dth = self.cfg.max_ori_speed_rad * dt
        max_dq = self.cfg.max_joint_speed_rad * dt
        q_cmd = list(self.r.getActualQ())
        p_cmd, q_ori = pose_to_pq(self.r.getActualTCPPose())

        while not self.stop.is_set():
            t_start = self.c.initPeriod()
            now = time.monotonic()
            with self.lock:
                snap = None if self.obj is None else {k: (v.copy() if hasattr(v, "copy") else v) for k, v in self.obj.items()}
                offset = self.T_obj_tcp

            p_des, q_des = p_cmd, q_ori
            if snap is not None and offset is not None:
                if now - snap["stamp"] <= self.cfg.vision_timeout_s:
                    pred_dt = now - snap["stamp"] + self.cfg.vision_delay_s
                    p_obj, q_obj = integrate_pose(snap["p"], snap["q"], snap["v"], snap["w"], pred_dt)
                    p_des, q_des = pq_mul((p_obj, q_obj), offset)
                else:
                    with self.lock:
                        if self.obj is not None:
                            self.obj["v"] *= 0.85
                            self.obj["w"] *= 0.85

            p_cmd, q_ori = step_toward(p_cmd, q_ori, p_des, q_des, max_dp, max_dth)
            pose_cmd = pq_to_pose(p_cmd, q_ori)

            if self.cfg.servo_mode.lower() == "servol":
                self.c.servoL(pose_cmd, 0.0, 0.0, dt, self.cfg.lookahead_time, self.cfg.gain)
            else:
                q_ik = self.kin.ik(p_cmd, q_ori, q_cmd)
                ok = True
                q_lim = list(q_cmd)
                for i in range(6):
                    dq = q_ik[i] - q_cmd[i]
                    if abs(dq) > self.cfg.ik_jump_limit_rad:
                        ok = False
                        break
                    q_lim[i] = q_cmd[i] + max(-max_dq, min(max_dq, dq))
                if ok:
                    q_cmd = q_lim
                self.c.servoJ(q_cmd, 0.0, 0.0, dt, self.cfg.lookahead_time, self.cfg.gain)

            self.c.waitPeriod(t_start)


def simulated_object_base(t_s):
    yaw = 0.15 * math.sin(0.3 * t_s)
    p = np.array([0.40 + 0.04 * math.cos(0.4 * t_s), 0.04 * math.sin(0.4 * t_s), 0.15])
    q = np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])
    return p, q


def main():
    import argparse

    parser = argparse.ArgumentParser(description="UR 6D eye-in-hand servoJ tracking")
    parser.add_argument("--ip", default="192.168.0.10")
    parser.add_argument("--series", default="cb3", choices=["cb3", "e"])
    parser.add_argument("--model", default="ur10", choices=list(UR_DH))
    parser.add_argument("--mode", default="servoJ", choices=["servoJ", "servoL"])
    args = parser.parse_args()

    if rtde_control is None or rtde_receive is None:
        raise SystemExit("ur_rtde is not installed. pip install ur_rtde")

    cfg = Config(robot_ip=args.ip, series=args.series, model=args.model, servo_mode=args.mode)
    rtde_c = rtde_control.RTDEControlInterface(cfg.robot_ip)
    rtde_r = rtde_receive.RTDEReceiveInterface(cfg.robot_ip)

    rtde_c.moveJ(cfg.home_q, 0.5, 0.5)

    tracker = Ur6dServoJTracker(rtde_c, rtde_r, cfg)
    tracker.start()
    print(
        f"UR {cfg.model} {cfg.servo_mode} tracking: vision 30 Hz, control {1.0 / cfg.dt:.0f} Hz. "
        "Ctrl+C to stop."
    )
    try:
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            p_obj, q_obj = simulated_object_base(t)
            tcp = rtde_r.getActualTCPPose()
            p_tcp, q_tcp = pose_to_pq(tcp)
            p_cam, q_cam = pq_mul((p_tcp, q_tcp), tracker.T_tcp_cam)
            p_co, q_co = pq_mul(pq_inv(p_cam, q_cam), (p_obj, q_obj))
            tracker.on_vision(pq_to_pose(p_co, q_co), tcp)
            time.sleep(1.0 / 30.0)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.stop_tracker()
        rtde_c.stopScript()


if __name__ == "__main__":
    main()
