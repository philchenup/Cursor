#!/usr/bin/env python3
"""
live_track.py  --  DeepAC 引导式初始化 + 全画面跟踪 (无 ROS)

复刻作者演示视频的交互:
  INIT  画面中央显示黄色网格线框(位姿固定不动) -> 你把工件移过去对齐
        对齐瞬间优化收敛, 自动锁定(或按空格手动锁定)
  TRACK 线框变绿 + 显示物体坐标系, 跟着工件走
        失锁自动退回 INIT

位姿每帧打印到 stdout (可用 --output 同时写入文件), 不依赖 ROS2。

放在 DeepAC 仓库根目录运行。

  # 先用已有图像序列验证 (推荐, 可反复跑)
  python live_track.py --source data/mydata/img --obj calib --data-dir data/mydata

  # 接相机
  python live_track.py --source realsense --obj calib --data-dir data/mydata
  python live_track.py --source cam:0     --obj calib --data-dir data/mydata

按键:
  INIT   j l  偏航      i k  俯仰      u o  滚转   (大写 = 0.2 度)
         - =  远近      方向键 平移（或 z/x 左右、v/b 上下）
         空格 手动锁定   a 切换自动锁定
  INIT   s 保存当前初始姿态(下次自动载入)
  两态   w 线框开关  n 网状/特征边切换  c 坐标系  p 暂停  r 回 INIT  q 退出

阈值需要你自己标定: 先跑一次, 盯着画面左上角三个数, 看对齐前后怎么变,
再用 --lock-dt / --lock-dr / --lock-conf 填进去。
"""

import argparse
import glob
import time
import os
import pickle
import sys
import warnings

import numpy as np

# ---------------------------------------------------------------------------
# 纯 numpy 工具: 不依赖 torch / cv2 / ROS, 便于单测
# ---------------------------------------------------------------------------

STATE_CODE = {'INIT': 0, 'TRACK': 1, 'HOLD': 2, 'LOST': 3}


def quat_from_R(R):
    """旋转矩阵 -> 四元数 (x, y, z, w), Shepperd 分支法。"""
    m = np.asarray(R, dtype=np.float64)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return float(x), float(y), float(z), float(w)


def format_pose_line(frame_index, state, valid, R, t, met, stamp_sec=None):
    """一行位姿样本, 空格分隔, 方便重定向/解析。

    stamp_sec frame state valid conf cs dt_mm dr_deg tx ty tz qx qy qz qw
    state 为 INIT/TRACK/HOLD/LOST; valid 为 0/1。
    """
    if stamp_sec is None:
        stamp_sec = time.time()
    if R is None or t is None:
        qx = qy = qz = qw = float('nan')
        xyz = [float('nan')] * 3
    else:
        qx, qy, qz, qw = quat_from_R(R)
        xyz = [float(v) for v in t]
    conf = met.get('conf', float('nan'))
    cs = met.get('cs', float('nan'))
    dt = met.get('dt', float('nan'))
    dr = met.get('dr', float('nan'))
    return (
        '{stamp:.6f} {frame:d} {state} {valid:d} '
        '{conf:.6g} {cs:.6g} {dt:.6g} {dr:.6g} '
        '{tx:.8f} {ty:.8f} {tz:.8f} {qx:.8f} {qy:.8f} {qz:.8f} {qw:.8f}'
    ).format(
        stamp=stamp_sec, frame=int(frame_index), state=state,
        valid=int(bool(valid)), conf=conf, cs=cs, dt=dt, dr=dr,
        tx=xyz[0], ty=xyz[1], tz=xyz[2], qx=qx, qy=qy, qz=qz, qw=qw,
    )


def rot_axis(axis, deg):
    a = np.asarray(axis, np.float64)
    a /= np.linalg.norm(a)
    th = np.deg2rad(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def euler_to_R(rx, ry, rz):
    """XYZ 欧拉角(度) -> R = Rz @ Ry @ Rx"""
    return rot_axis([0, 0, 1], rz) @ rot_axis([0, 1, 0], ry) @ rot_axis([1, 0, 0], rx)


class PoseOutput:
    """把每帧跟踪样本打印到 stdout, 可选写入文本文件。不依赖 ROS。"""

    HEADER = ('# stamp_sec frame state valid conf cs dt_mm dr_deg '
              'tx ty tz qx qy qz qw')

    def __init__(self, a):
        self.print_stdout = not getattr(a, 'no_print', False)
        self.print_init = getattr(a, 'print_init', False)
        self.fp = None
        out = getattr(a, 'output', '') or ''
        if out:
            self.fp = open(out, 'w', encoding='utf-8')
            self.fp.write(self.HEADER + '\n')
            print(f'[pose ] 写入 {out}')
        if self.print_stdout:
            print('[pose ] stdout: ' + self.HEADER)

    def send(self, R, t, frame_index, state, valid, met):
        if state == 'INIT' and not self.print_init and not valid:
            line = None
        else:
            line = format_pose_line(frame_index, state, valid, R, t, met)
        if line is None:
            return
        if self.print_stdout:
            print(line, flush=True)
        if self.fp is not None:
            self.fp.write(line + '\n')
            self.fp.flush()

    def close(self):
        if self.fp is not None:
            self.fp.close()
            self.fp = None


# 六个正视图: 让物体的某个轴指向相机 (相机沿 +Z 看)
PRESETS = {
    '1': ('+Z 面 (默认)', np.eye(3)),
    '2': ('-Z 面', None),
    '3': ('+X 面', None),
    '4': ('-X 面', None),
    '5': ('+Y 面', None),
    '6': ('-Y 面', None),
}


def _build_presets():
    PRESETS['2'] = ('-Z 面', rot_axis([0, 1, 0], 180))
    PRESETS['3'] = ('+X 面', rot_axis([0, 1, 0], 90))
    PRESETS['4'] = ('-X 面', rot_axis([0, 1, 0], -90))
    PRESETS['5'] = ('+Y 面', rot_axis([1, 0, 0], -90))
    PRESETS['6'] = ('-Y 面', rot_axis([1, 0, 0], 90))


# ---------------------------------------------------------------------------
# 以下需要 DeepAC / OpenCV / torch, 仅在实际跟踪时导入
# ---------------------------------------------------------------------------

YELLOW, GREEN, RED, CYAN = (0, 220, 255), (0, 255, 0), (0, 0, 255), (255, 220, 0)


class Wire:
    """两种线框: full = 抽稀后的完整网格(像演示视频), feature = 锐边+轮廓边"""

    def __init__(self, path, unit, sharp_deg=10.0, net_faces=700):
        import trimesh
        m = trimesh.load(path, force='mesh')
        m.merge_vertices()
        self.V = np.asarray(m.vertices, np.float64) * unit
        self.F = np.asarray(m.faces, np.int64)
        self.fn = np.asarray(m.face_normals, np.float64)
        self.fc = self.V[self.F].mean(axis=1)
        self.adj = np.asarray(m.face_adjacency, np.int64)
        self.adj_e = np.asarray(m.face_adjacency_edges, np.int64)
        ang = np.asarray(m.face_adjacency_angles, np.float64)
        self.sharp = ang > np.deg2rad(sharp_deg)
        self.diag = float(np.linalg.norm(self.V.max(0) - self.V.min(0)))

        self.NV, self.NE = self._make_net(path, unit, net_faces)
        c = (self.V.max(0) + self.V.min(0)) / 2 * 1000
        print(f'[mesh] {path}  verts {len(self.V)} faces {len(self.F)} '
              f'sharp {int(self.sharp.sum())}  net-edges {len(self.NE)}')
        print(f'       center(mm) {c.round(2)} '
              f'{"OK" if np.abs(c).max() < 1 else "<-- NOT CENTERED"}')

    def _make_net(self, path, unit, target):
        """抽稀出一个低面数网格, 用它的全部边画网状线框"""
        try:
            import pymeshlab
            ms = pymeshlab.MeshSet()
            ms.load_new_mesh(str(path))
            ms.meshing_remove_duplicate_vertices()
            if ms.current_mesh().face_number() > target:
                ms.meshing_decimation_quadric_edge_collapse(
                    targetfacenum=target, preserveboundary=True,
                    preservetopology=True, planarquadric=True)
            mm = ms.current_mesh()
            V = mm.vertex_matrix().astype(np.float64) * unit
            F = mm.face_matrix().astype(np.int64)
            e = np.vstack([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
            E = np.unique(np.sort(e, axis=1), axis=0)
            return V, E
        except Exception as ex:
            print(f'[warn] pymeshlab 抽稀失败({ex}), 改用抽样边')
            e = np.vstack([self.F[:, [0, 1]], self.F[:, [1, 2]], self.F[:, [2, 0]]])
            E = np.unique(np.sort(e, axis=1), axis=0)
            if len(E) > 1500:
                E = E[np.linspace(0, len(E) - 1, 1500).astype(int)]
            return self.V, E

    @staticmethod
    def _uv(P, R, t, K):
        Xc = P @ R.T + t
        ok = Xc[:, 2] > 1e-6
        uv = np.full((len(P), 2), np.nan)
        if ok.any():
            p = Xc[ok] @ K.T
            uv[ok] = p[:, :2] / p[:, 2:3]
        return uv, ok

    def _seg(self, img, uv, ok, pairs, color, th):
        import cv2
        h, w = img.shape[:2]
        for a, b in pairs:
            if not (ok[a] and ok[b]):
                continue
            pa, pb = uv[a], uv[b]
            if (max(pa[0], pb[0]) < -w or min(pa[0], pb[0]) > 2 * w or
                    max(pa[1], pb[1]) < -h or min(pa[1], pb[1]) > 2 * h):
                continue
            cv2.line(img, (int(pa[0]), int(pa[1])),
                     (int(pb[0]), int(pb[1])), color, th, cv2.LINE_AA)

    def draw_net(self, img, R, t, K, color, thick=1):
        """完整网状线框, 不剔除背面 —— 与作者演示视频一致"""
        uv, ok = self._uv(self.NV, R, t, K)
        if not ok.any():
            return
        self._seg(img, uv, ok, self.NE, color, thick)

    def draw(self, img, R, t, K, color, thick=1):
        """锐边 + 轮廓边, 带背面剔除 —— 精确对齐用"""
        import cv2
        uv, ok = self._uv(self.V, R, t, K)
        if not ok.any():
            return
        n = self.fn @ R.T
        cc = self.fc @ R.T + t
        fr = np.einsum('ij,ij->i', n, cc) < 0
        f0, f1 = fr[self.adj[:, 0]], fr[self.adj[:, 1]]
        sil = f0 ^ f1
        vis = self.sharp & (f0 | f1) & ~sil
        self._seg(img, uv, ok, self.adj_e[vis], color, thick)
        self._seg(img, uv, ok, self.adj_e[sil], color, max(thick, 2))

    def draw_axes(self, img, R, t, K):
        import cv2
        L = self.diag * 0.6
        P = np.array([[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, L]])
        Xc = P @ R.T + t
        if (Xc[:, 2] <= 1e-6).any():
            return
        p = Xc @ K.T
        uv = (p[:, :2] / p[:, 2:3]).astype(int)
        o = tuple(uv[0])
        for i, col in [(1, (0, 0, 255)), (2, (0, 255, 0)), (3, (255, 0, 0))]:
            cv2.arrowedLine(img, o, tuple(uv[i]), col, 3, tipLength=0.25)


def auto_front_view(V_mm):
    """把六个正视图各正交投影一次, 选投影面积最大的作为初始朝向。"""
    import cv2
    best_k, best_R, best_a = '1', np.eye(3), -1.0
    for k in sorted(PRESETS):
        R = PRESETS[k][1]
        p = (V_mm @ R.T)[:, :2].astype(np.float32)
        area = float(cv2.contourArea(cv2.convexHull(p.reshape(-1, 1, 2))))
        if area > best_a:
            best_k, best_R, best_a = k, R.copy(), area
    q = (V_mm @ best_R.T)[:, :2]
    ext = q.max(0) - q.min(0)
    if ext[1] > ext[0]:
        best_R = rot_axis([0, 0, 1], 90) @ best_R
    print(f'[init ] 自动选正面: {PRESETS[best_k][0]}  '
          f'投影面积 {best_a/100:.1f} cm2  (--init-view {best_k} 可锁定)')
    return best_R


def orthonormalize(p):
    """把 R 投影回 SO(3), 消除数值漂移。"""
    import torch
    from src_open.utils.geometry.wrappers import Pose
    R = p.R.double()
    U, _, Vh = torch.linalg.svd(R)
    Rn = U @ Vh
    if torch.linalg.det(Rn) < 0:
        U = U.clone()
        U[:, -1] *= -1
        Rn = U @ Vh
    return Pose.from_Rt(Rn.float(), p.t)


def rot_health(p):
    """返回 (det(R), 正交性最大误差)"""
    import torch
    R = p.R.double()
    I = torch.eye(3, dtype=torch.float64)
    return float(torch.linalg.det(R)), float((R @ R.T - I).abs().max())


def pose_delta(p_a, p_b):
    """两个 Pose 之间的平移距离(米)和旋转角(度)"""
    import torch
    dt = float((p_b.t - p_a.t).norm())
    Rr = p_b.R @ p_a.R.transpose(-1, -2)
    cos = (torch.diagonal(Rr, dim1=-2, dim2=-1).sum(-1) - 1) / 2
    return dt, float(torch.rad2deg(torch.arccos(cos.clamp(-1, 1))))


class Tracker:
    def __init__(self, a):
        import torch
        from omegaconf import OmegaConf
        from src_open.models import get_model
        from src_open.utils.lightening_utils import (MyLightningLogger,
                                                     convert_old_model,
                                                     load_model_weight)

        self.a = a
        train_cfg = OmegaConf.load(a.load_cfg)
        self.dc = train_cfg.data

        logger = MyLightningLogger('live', a.save_dir)
        model = get_model(train_cfg.models.name)(train_cfg.models)
        ckpt = torch.load(a.load_model, map_location='cpu')
        if 'pytorch-lightning_version' not in ckpt:
            warnings.warn('old ckpt format, converting')
            ckpt = convert_old_model(ckpt)
        load_model_weight(model, ckpt, logger)
        self.model = model.cuda().eval()

        self.cap = {}
        orig = self.model.boundary_predictor.forward

        def hooked(*args, **kw):
            out = orig(*args, **kw)
            self.cap['unc'] = out[2]
            return out
        self.model.boundary_predictor.forward = hooked

        tpl = os.path.join(a.data_dir, a.obj, 'pre_render', f'{a.obj}.pkl')
        with open(tpl, 'rb') as f:
            d = pickle.load(f)
        self.n_pts = d['head']['num_sample_contour_point']
        self.tviews = torch.from_numpy(d['template_view']).float()
        self.orients = torch.from_numpy(d['orientation_in_body']).float()
        print(f'[tpl ] {tpl}  {len(self.orients)} views, {self.n_pts} pts/view')

        self.K = np.loadtxt(os.path.join(a.data_dir, 'K.txt')).reshape(3, 3)
        self.wire = Wire(os.path.join(a.data_dir, f'{a.obj}.obj'),
                         a.geometry_unit_in_meter, a.sharp_deg, a.net_faces)

        _build_presets()
        self.init_file = a.init_pose or os.path.join(a.data_dir, 'init_pose.txt')
        self.R0 = np.eye(3)
        if a.init_view == 'auto':
            self.R0 = auto_front_view(self.wire.V * 1000.0)
        elif a.init_view in PRESETS:
            self.R0 = PRESETS[a.init_view][1].copy()
            print(f'[init ] --init-view {a.init_view} = {PRESETS[a.init_view][0]}')
        if a.init_rpy != [0.0, 0.0, 0.0]:
            self.R0 = euler_to_R(*a.init_rpy) @ self.R0
            print(f'[init ] 叠加 --init-rpy {a.init_rpy}')
        if a.init_pose and os.path.exists(self.init_file):
            r = np.loadtxt(self.init_file).reshape(-1)
            self.R0 = r[:9].reshape(3, 3)
            if r.size >= 12:
                a.init_z_mm = float(r[11])
            print(f'[init ] 载入 {self.init_file}  z={a.init_z_mm:.0f}mm')
        self.R0_home = self.R0.copy()

        self.t0 = np.array([0.0, 0.0, a.init_z_mm / 1000.0])
        self.reset()
        self.pub = PoseOutput(a)
        self.csv = None
        if a.log_pose:
            self.csv = open(a.log_pose, 'w')
            self.csv.write('frame,t_sec,tx_mm,ty_mm,tz_mm,'
                           'r00,r01,r02,r10,r11,r12,r20,r21,r22,conf\n')
            print(f'[log  ] 位姿记录 -> {a.log_pose}')
        self.nframe = 0
        self.show = {'wire': True, 'pts': True, 'ax': True,
                     'net': not a.feature_wire}
        self.centered = False
        self.scaled = False
        self.last_good = None
        self.auto = a.auto

    def reset(self):
        self.R0 = self.R0_home.copy()
        self.state = 'INIT'
        self.pose = None
        self.hist = None
        self.good = 0
        self.bad = 0
        self.conf_s = None

    def _prep(self, img_rgb, bbox2d, camera):
        from src_open.dataset.utils import resize, numpy_image_to_torch, crop, zero_pad
        bbox2d[2:] += self.dc.crop_border * 2
        img, camera, _ = crop(img_rgb, bbox2d, camera=camera, return_bbox=True)
        scales = (1, 1)
        if isinstance(self.dc.resize, int):
            if self.dc.resize_by == 'max':
                img, scales = resize(img, self.dc.resize, fn=max)
            elif (self.dc.resize_by == 'min' or
                  (self.dc.resize_by == 'min_if' and
                   min(*img.shape[:2]) < self.dc.resize)):
                img, scales = resize(img, self.dc.resize, fn=min)
        elif len(self.dc.resize) == 2:
            img, scales = resize(img, list(self.dc.resize))
        if scales != (1, 1):
            camera = camera.scale(scales)
        img, = zero_pad(self.dc.pad, img)
        return numpy_image_to_torch(img.astype(np.float32)), camera

    def _pass(self, img_rgb, pose_in, hist):
        """单次 DeepAC 精化。返回 (新位姿, 裁剪图, 裁剪相机, conf)"""
        import torch
        from src_open.utils.geometry.wrappers import Camera
        from src_open.utils.utils import (project_correspondences_line,
                                          get_closest_k_template_view_index,
                                          get_bbox_from_p2d)
        from src_open.models.deep_ac import calculate_basic_line_data

        h, w = img_rgb.shape[:2]
        ori_cam = Camera(torch.tensor(
            [w, h, self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]],
            dtype=torch.float32))

        k = self.dc.get_top_k_template_views * self.dc.skip_template_view
        idx = get_closest_k_template_view_index(pose_in, self.orients, k)
        ctv = torch.stack([self.tviews[i * self.n_pts:(i + 1) * self.n_pts, :]
                           for i in idx[::self.dc.skip_template_view]])
        cob = self.orients[idx[::self.dc.skip_template_view]]

        lines = project_correspondences_line(ctv[0], pose_in, ori_cam)
        bbox = get_bbox_from_p2d(lines['centers_in_image'])
        img, cam = self._prep(img_rgb, bbox.numpy().copy(), ori_cam)

        if hist is None:
            *_, ci, cv_, ni, fd, bd, _ = calculate_basic_line_data(
                ctv[None][:, 0], pose_in[None]._data, cam[None]._data, 1, 0)
            hist = self.model.histogram.calculate_histogram(
                img[None], ci, cv_, ni, fd, bd, True)

        data = {'image': img[None].cuda(),
                'camera': cam[None].cuda(),
                'body2view_pose': pose_in[None].cuda(),
                'closest_template_views': ctv[None].cuda(),
                'closest_orientations_in_body': cob[None].cuda(),
                'fore_hist': hist[0].cuda(),
                'back_hist': hist[1].cuda()}
        pred = self.model._forward(data, visualize=False, tracking=True)

        ps = pred['opt_body2view_pose']
        out = ps[-1][0].cpu()
        d_scale = pose_delta(ps[-2][0].cpu(), ps[-1][0].cpu())
        unc = self.cap.get('unc')
        conf = float(unc.mean()) if unc is not None else float('nan')
        return out, img, cam, hist, conf, d_scale

    def step(self, img_rgb, pose_in, hist):
        """多次迭代精化。返回 (位姿, 直方图, 指标, 裁剪相机, 裁剪图)"""
        import torch
        pose = pose_in
        h0 = hist
        with torch.no_grad():
            for _ in range(max(1, self.a.iters)):
                pose, img, cam, h0, conf, d_scale = self._pass(img_rgb, pose, h0)
        pose = orthonormalize(pose)
        d_init = pose_delta(pose_in, pose)
        met = {'dt': d_scale[0] * 1000, 'dr': d_scale[1],
               'mt': d_init[0] * 1000, 'mr': d_init[1], 'conf': conf}
        return pose, h0, met, cam, img

    def refresh_hist(self, img, pose, cam, hist):
        from src_open.utils.utils import get_closest_template_view_index
        from src_open.models.deep_ac import calculate_basic_line_data
        j = get_closest_template_view_index(pose, self.orients)
        tv = self.tviews[j * self.n_pts:(j + 1) * self.n_pts, :]
        *_, ci, cv_, ni, fd, bd, _ = calculate_basic_line_data(
            tv[None], pose[None]._data, cam[None]._data, 1, 0)
        f, b = self.model.histogram.calculate_histogram(
            img[None], ci, cv_, ni, fd, bd, True)
        lr = self.a.learn_rate
        return ((1 - lr) * hist[0] + lr * f, (1 - lr) * hist[1] + lr * b)

    def commit(self, img_rgb, cand, met, tag):
        """锁定前先把位姿彻底收敛, 再据此建立直方图。"""
        import torch
        pose = cand
        n = max(1, self.a.lock_iters)
        with torch.no_grad():
            for _ in range(n):
                pose, _, _, _, conf, _ = self._pass(img_rgb, pose, None)
        pose = orthonormalize(pose)
        d, e = rot_health(pose)
        moved = pose_delta(cand, pose)
        print(f'[{tag}] conf={met["conf"]:.3f} cs={met["cs"]:.3f} | '
              f'收敛 {n} 遍后又移动 {moved[0]*1000:.1f}mm/{moved[1]:.1f}deg | '
              f'det(R)={d:.6f} 正交误差={e:.2e}')
        if moved[0] * 1000 > self.a.lock_settle_mm or moved[1] > self.a.lock_settle_deg:
            print('       -> 仍未收敛, 放弃本次锁定')
            self.good = 0
            return
        self.pose = pose
        self.last_good = pose
        self.hist = None
        self.bad = 0
        self.state = 'TRACK'

    def _wire(self, vis, R, t, color):
        import cv2
        a = self.a.wire_alpha
        fn = self.wire.draw_net if self.show['net'] else self.wire.draw
        if a >= 0.99:
            fn(vis, R, t, self.K, color, self.a.wire_thick)
            return
        layer = vis.copy()
        fn(layer, R, t, self.K, color, self.a.wire_thick)
        cv2.addWeighted(layer, a, vis, 1 - a, 0, dst=vis)

    def run(self, frames):
        import cv2
        import torch
        from src_open.utils.geometry.wrappers import Pose

        win = 'DeepAC live'
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        sized = False
        writer = None
        paused = False
        t_prev, fps = time.time(), 0.0
        it = iter(frames)
        frame = None

        try:
            while True:
                if not paused or frame is None:
                    try:
                        frame = next(it)
                    except StopIteration:
                        it = iter(frames)
                        continue
                s = self.a.input_scale
                if s != 1.0:
                    if not self.scaled:
                        self.K = self.K.copy()
                        self.K[:2] *= s
                        self.scaled = True
                    frame = cv2.resize(frame, None, fx=s, fy=s,
                                       interpolation=cv2.INTER_AREA)
                img_rgb = frame[..., ::-1].copy()
                vis = frame.copy()

                if not self.centered:
                    H, W = frame.shape[:2]
                    z = self.t0[2]
                    self.t0[0] = (W / 2 - self.K[0, 2]) * z / self.K[0, 0]
                    self.t0[1] = (H / 2 - self.K[1, 2]) * z / self.K[1, 1]
                    self.centered = True
                    if not sized:
                        ds = self.a.display_scale
                        cv2.resizeWindow(win, int(W * ds), int(H * ds))
                        sized = True
                        print(f'[disp ] 窗口 {int(W*ds)}x{int(H*ds)} '
                              f'(--display-scale {ds})')
                    print(f'[frame] {W}x{H}   K: fx={self.K[0,0]:.1f} '
                          f'fy={self.K[1,1]:.1f} cx={self.K[0,2]:.1f} '
                          f'cy={self.K[1,2]:.1f}')
                    if abs(self.K[0, 2] - W / 2) > W * 0.1:
                        print('[warn] cx 与画面中心相差很大 —— K.txt 的标定分辨率'
                              '可能和当前视频流不一致!')

                def smooth(m):
                    b = self.a.conf_smooth
                    self.conf_s = (m['conf'] if self.conf_s is None
                                   else b * self.conf_s + (1 - b) * m['conf'])
                    m['cs'] = self.conf_s
                    return m

                if self.state == 'INIT':
                    pin = Pose.from_Rt(torch.from_numpy(self.R0).float(),
                                       torch.from_numpy(self.t0).float())
                    out, _, met, cam, _ = self.step(img_rgb, pin, None)
                    if self.a.recover_last and self.last_good is not None:
                        o2, _, m2, c2, _ = self.step(img_rgb, self.last_good, None)
                        if m2['conf'] > met['conf']:
                            out, met, cam = o2, m2, c2
                    met = smooth(met)
                    if self.show['wire']:
                        self._wire(vis, self.R0, self.t0, YELLOW)
                    lock = (met['dt'] < self.a.lock_dt and met['dr'] < self.a.lock_dr
                            and met['cs'] > self.a.lock_conf)
                    self.good = self.good + 1 if lock else 0
                    if self.auto and self.good >= self.a.lock_frames:
                        self.commit(img_rgb, out, met, 'LOCK')
                    if self.state == 'INIT':
                        self.pub.send(None, None, self.nframe, 'INIT', False, met)
                    banner = (f"INIT  auto={'ON' if self.auto else 'OFF'}  "
                              f"good={self.good}/{self.a.lock_frames}  "
                              f"[space]=lock")
                else:
                    out, self.hist, met, cam, crop_img = self.step(
                        img_rgb, self.pose, self.hist)
                    met = smooth(met)
                    if met['cs'] >= self.a.gate_conf:
                        self.pose = out
                        if met['cs'] >= self.a.hist_conf:
                            self.last_good = out
                    if met['cs'] >= self.a.hist_conf:
                        self.hist = self.refresh_hist(
                            crop_img, self.pose, cam, self.hist)
                    R = self.pose.R.numpy().astype(np.float64)
                    t = self.pose.t.numpy().astype(np.float64)
                    if self.show['wire']:
                        self._wire(vis, R, t, GREEN)
                    if self.show['ax']:
                        self.wire.draw_axes(vis, R, t, self.K)
                    bad = (met['dt'] > self.a.lost_dt or met['dr'] > self.a.lost_dr
                           or met['cs'] < self.a.lost_conf)
                    self.bad = self.bad + 1 if bad else 0
                    accepted = met['cs'] >= self.a.gate_conf and not bad
                    pub_state = 'TRACK' if accepted else 'HOLD'
                    if self.bad >= self.a.lost_frames:
                        d, e = rot_health(self.pose)
                        print(f'[LOST] conf={met["conf"]:.3f} cs={met["cs"]:.3f} '
                              f'det(R)={d:.6f} 正交误差={e:.2e}')
                        pub_state = 'LOST'
                    self.pub.send(R, t, self.nframe, pub_state,
                                  pub_state == 'TRACK', met)
                    if self.csv is not None:
                        self.csv.write(
                            f"{self.nframe},{time.time():.6f},"
                            + ','.join(f'{v*1000:.4f}' for v in t) + ','
                            + ','.join(f'{v:.8f}' for v in R.reshape(-1))
                            + f",{met['conf']:.4f}\n")
                        self.csv.flush()
                    if pub_state == 'LOST':
                        self.reset()
                    gate = '' if met['cs'] >= self.a.gate_conf else ' HOLD'
                    banner = (f"TRACK{gate}  bad={self.bad}/{self.a.lost_frames}")

                self.nframe += 1
                now = time.time()
                fps = 0.8 * fps + 0.2 / max(now - t_prev, 1e-6)
                t_prev = now
                l1 = f"{banner}  {fps:4.1f}fps"
                l2 = (f"conf={met['conf']:.2f}~{met['cs']:.2f}  "
                      f"dt={met['dt']:.2f}mm dr={met['dr']:.2f}d  "
                      f"move={met['mt']:.0f}mm/{met['mr']:.0f}d")
                W = vis.shape[1]
                fs = max(0.34, min(0.62, W / 2000.0))
                col = CYAN if self.state == 'INIT' else GREEN
                lines_txt = [l1, l2]
                if cv2.getTextSize(l1 + '  ' + l2, cv2.FONT_HERSHEY_SIMPLEX,
                                   fs, 1)[0][0] < W - 12:
                    lines_txt = [l1 + '  |  ' + l2]
                bh = int(20 * fs / 0.5) * len(lines_txt) + 8
                cv2.rectangle(vis, (0, 0), (W, bh), (0, 0, 0), -1)
                for i2, sline in enumerate(lines_txt):
                    cv2.putText(vis, sline,
                                (6, int(15 * fs / 0.5) + i2 * int(20 * fs / 0.5)),
                                cv2.FONT_HERSHEY_SIMPLEX, fs, col, 1, cv2.LINE_AA)
                cv2.imshow(win, vis)

                if self.a.save:
                    if writer is None:
                        writer = cv2.VideoWriter(
                            self.a.save, cv2.VideoWriter_fourcc(*'MJPG'), 20,
                            (vis.shape[1], vis.shape[0]))
                    writer.write(vis)

                k = cv2.waitKeyEx(1)
                if k == -1:
                    continue
                c = k & 0xFF
                if c in (ord('q'), 27):
                    break
                elif c == ord('p'):
                    paused = not paused
                elif c == ord('r'):
                    self.reset()
                elif c == ord('a'):
                    self.auto = not self.auto
                elif c == ord('w'):
                    self.show['wire'] = not self.show['wire']
                elif c == ord('n'):
                    self.show['net'] = not self.show['net']
                elif c == ord('s') and self.state == 'INIT':
                    np.savetxt(self.init_file,
                               np.concatenate([self.R0.reshape(-1),
                                               self.t0 * 1000]).reshape(1, -1),
                               fmt='%.8f')
                    self.R0_home = self.R0.copy()
                    print(f'[init ] 已保存 {self.init_file}  '
                          f'下次加 --init-pose 即可复用')
                elif c == ord('c'):
                    self.show['ax'] = not self.show['ax']
                elif c == ord('g') and self.state == 'TRACK' and self.pose is not None:
                    R = self.pose.R.numpy().astype(np.float64)
                    t = self.pose.t.numpy().astype(np.float64)
                    M = np.eye(4, dtype=np.float64)
                    M[:3, :3], M[:3, 3] = R, t
                    np.savetxt(self.a.grasp_pose, M, fmt='%.12g')
                    print(f'[grasp] 已保存期望相机-工件位姿 cdMo -> {self.a.grasp_pose}')
                elif c == ord(' ') and self.state == 'INIT':
                    self.commit(img_rgb, out, met, 'LOCK manual')
                elif self.state == 'INIT' and chr(c) in PRESETS:
                    name, Rv = PRESETS[chr(c)]
                    self.R0 = Rv.copy()
                    self.R0_home = Rv.copy()
                    print(f'[view ] {name}')
                elif self.state == 'INIT':
                    step = 0.2 if c in b'JLIKUO' else 2.0
                    if c in (ord('j'), ord('J')):
                        self.R0 = rot_axis([0, 1, 0], -step) @ self.R0
                    elif c in (ord('l'), ord('L')):
                        self.R0 = rot_axis([0, 1, 0], step) @ self.R0
                    elif c in (ord('i'), ord('I')):
                        self.R0 = rot_axis([1, 0, 0], -step) @ self.R0
                    elif c in (ord('k'), ord('K')):
                        self.R0 = rot_axis([1, 0, 0], step) @ self.R0
                    elif c in (ord('u'), ord('U')):
                        self.R0 = rot_axis([0, 0, 1], step) @ self.R0
                    elif c in (ord('o'), ord('O')):
                        self.R0 = rot_axis([0, 0, 1], -step) @ self.R0
                    elif c in (ord('-'), ord('='), ord('+')):
                        z0 = self.t0[2]
                        self.t0[2] = (z0 + 0.005 if c == ord('-')
                                      else max(0.02, z0 - 0.005))
                        self.t0[:2] *= self.t0[2] / z0
                    elif k in (81, 65361, 0x250000, 0x01000012) or c == ord('z'):
                        self.t0[0] -= self.t0[2] / self.K[0, 0] * 5
                    elif k in (83, 65363, 0x270000, 0x01000014) or c == ord('x'):
                        self.t0[0] += self.t0[2] / self.K[0, 0] * 5
                    elif k in (82, 65362, 0x260000, 0x01000013) or c == ord('v'):
                        self.t0[1] -= self.t0[2] / self.K[1, 1] * 5
                    elif k in (84, 65364, 0x280000, 0x01000015) or c == ord('b'):
                        self.t0[1] += self.t0[2] / self.K[1, 1] * 5
        finally:
            if writer:
                writer.release()
            if self.csv is not None:
                self.csv.close()
            self.pub.close()
            cv2.destroyAllWindows()


def K_from_video_intrinsics(it):
    """把 SDK 彩色内参转成 3x3 K。RealSense 用 ppx/ppy, 兼容 cx/cy。"""
    cx = getattr(it, 'ppx', None)
    if cx is None:
        cx = it.cx
    cy = getattr(it, 'ppy', None)
    if cy is None:
        cy = it.cy
    return np.array([[it.fx, 0, cx], [0, it.fy, cy], [0, 0, 1]], np.float64)


def _color_format_name(cf):
    """读取彩色帧格式名 (bgr8 / rgb8 / yuyv / ...), 不强制依赖 SDK 枚举。"""
    fmt = None
    if hasattr(cf, 'get_profile'):
        try:
            fmt = cf.get_profile().format()
        except Exception:
            fmt = None
    if fmt is None and hasattr(cf, 'get_format'):
        fmt = cf.get_format()
    if fmt is None:
        raise RuntimeError('无法读取彩色帧格式')
    return str(fmt).rsplit('.', 1)[-1].lower()


def _start_realsense_color(rs):
    """只开彩色流; 默认配置失败时回退 1280x720 / 640x480 BGR8。"""
    pipe = rs.pipeline()
    attempts = [
        dict(width=None, height=None, fmt=None, fps=None),
        dict(width=1280, height=720, fmt=rs.format.bgr8, fps=30),
        dict(width=640, height=480, fmt=rs.format.bgr8, fps=30),
    ]
    last_err = None
    for spec in attempts:
        cfg = rs.config()
        if spec['width'] is None:
            cfg.enable_stream(rs.stream.color)
        else:
            cfg.enable_stream(
                rs.stream.color, spec['width'], spec['height'],
                spec['fmt'], spec['fps'])
        try:
            profile = pipe.start(cfg)
            return pipe, profile
        except RuntimeError as ex:
            last_err = ex
            try:
                pipe.stop()
            except Exception:
                pass
            pipe = rs.pipeline()
    raise RuntimeError('无法打开 RealSense 彩色流: {}'.format(last_err))


def frames_orbbec():
    """Intel RealSense 彩色流。返回 (帧生成器, K 或 None)。内参直接从 SDK 读。

    函数名保持不变, 调用约定与原来一致: ``gen, K = frames_orbbec()``。
    """
    import pyrealsense2 as rs

    pipe, profile = _start_realsense_color(rs)

    K = None
    try:
        vs = profile.get_stream(rs.stream.color).as_video_stream_profile()
        it = vs.get_intrinsics()
        K = K_from_video_intrinsics(it)
        print(f'[realsense] SDK 内参 fx={it.fx:.1f} fy={it.fy:.1f} '
              f'cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}  {it.width}x{it.height}')
    except Exception as ex:
        print(f'[realsense] 读内参失败({ex}), 回退到 K.txt')

    def gen():
        try:
            while True:
                try:
                    fs = pipe.wait_for_frames(1000)
                except RuntimeError:
                    continue
                if fs is None:
                    continue
                cf = fs.get_color_frame()
                if not cf:
                    continue
                yield color_frame_to_bgr(cf)
        finally:
            pipe.stop()
    return gen(), K


frames_realsense = frames_orbbec


def color_frame_to_bgr(cf):
    """把 RealSense 彩色帧转成 BGR ndarray。输入为彩色帧, 输出 HxWx3 uint8。"""
    w, h = cf.get_width(), cf.get_height()
    name = _color_format_name(cf)

    if name in ('mjpeg', 'mjpg'):
        import cv2
        buf = np.frombuffer(bytes(cf.get_data()), dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError('RealSense MJPEG 解码失败')
        return img

    data = np.asanyarray(cf.get_data())
    if name in ('bgr8', 'bgr'):
        if data.ndim == 1:
            data = data.reshape(h, w, 3)
        return np.ascontiguousarray(data)
    if name in ('rgb8', 'rgb'):
        import cv2
        if data.ndim == 1:
            data = data.reshape(h, w, 3)
        return cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    if name in ('bgra8', 'bgra'):
        import cv2
        if data.ndim == 1:
            data = data.reshape(h, w, 4)
        return cv2.cvtColor(data, cv2.COLOR_BGRA2BGR)
    if name in ('rgba8', 'rgba'):
        import cv2
        if data.ndim == 1:
            data = data.reshape(h, w, 4)
        return cv2.cvtColor(data, cv2.COLOR_RGBA2BGR)
    if name in ('yuyv', 'yuy2', 'yuv422'):
        import cv2
        if data.ndim == 1:
            data = data.reshape(h, w, 2)
        return cv2.cvtColor(data, cv2.COLOR_YUV2BGR_YUYV)
    if name in ('y8', 'raw8', 'gray'):
        import cv2
        if data.ndim == 1:
            data = data.reshape(h, w)
        return cv2.cvtColor(data, cv2.COLOR_GRAY2BGR)
    raise RuntimeError(f'未处理的彩色格式 {name}, 请在 color_frame_to_bgr 里补上')


def frames_from(source):
    """返回 (帧序列或生成器, K 或 None)。不支持 ROS2。"""
    if source.startswith('ros2:') or source.startswith('ros:'):
        sys.exit(
            '已去掉 ROS 依赖。请改用图像目录、cam:0 或 realsense;\n'
            '位姿每帧打印到 stdout, 也可用 --output / --log-pose 落盘。'
        )
    import cv2
    if source in ('orbbec', 'realsense', 'rs'):
        return frames_orbbec()
    if source.startswith('cam:'):
        cap = cv2.VideoCapture(int(source[4:]))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        def gen():
            while True:
                ok, f = cap.read()
                if not ok:
                    break
                yield f
        return gen(), None
    paths = sorted(glob.glob(os.path.join(source, '*.png'))) + \
        sorted(glob.glob(os.path.join(source, '*.jpg')))
    if not paths:
        sys.exit(f'no images in {source}')
    return [cv2.imread(p) for p in paths], None


def build_parser():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--source', required=True,
                   help='图像目录 | cam:0 | realsense')
    p.add_argument('--obj', required=True)
    p.add_argument('--data-dir', required=True)
    p.add_argument('--load-cfg', default='workspace/train_bop_deepac/'
                   'logs-2024-01-08-15-52-47/train_cfg.yml')
    p.add_argument('--load-model', default='workspace/train_bop_deepac/'
                   'logs-2024-01-08-15-52-47/model_last.ckpt')
    p.add_argument('--save-dir', default='workspace/live')
    p.add_argument('--geometry-unit-in-meter', type=float, default=0.001)
    p.add_argument('--sharp-deg', type=float, default=10.0)
    p.add_argument('--net-faces', type=int, default=700,
                   help='网状线框抽稀到多少面 (越大越密)')
    p.add_argument('--feature-wire', action='store_true',
                   help='用锐边+轮廓边代替网状线框')
    p.add_argument('--init-z-mm', type=float, default=250)
    p.add_argument('--init-view', default='auto',
                   choices=['auto', 'none', '1', '2', '3', '4', '5', '6'],
                   help='初始朝向: auto=自动选最大投影面(默认), '
                        'none=用 CAD 原始坐标系, 1-6=指定正视图')
    p.add_argument('--init-rpy', type=float, nargs=3, default=[0.0, 0.0, 0.0],
                   metavar=('RX', 'RY', 'RZ'),
                   help='在 --init-view 基础上叠加的欧拉角(度), 依次绕 X/Y/Z')
    p.add_argument('--init-pose', default='',
                   help='初始姿态文件路径; 给了且存在则载入。'
                        'INIT 状态按 s 保存到 <data_dir>/init_pose.txt')
    p.add_argument('--learn-rate', type=float, default=0.2)
    p.add_argument('--iters', type=int, default=3,
                   help='每帧精化迭代次数, 等效扩大捕获半径 (1=原版行为)')
    p.add_argument('--gate-conf', type=float, default=0.35,
                   help='低于此置信度不接受新位姿(保持上一帧)')
    p.add_argument('--hist-conf', type=float, default=0.45,
                   help='低于此置信度冻结颜色直方图, 防止被污染')
    p.add_argument('--wire-thick', type=int, default=1)
    p.add_argument('--wire-alpha', type=float, default=0.65,
                   help='线框透明度, 越小线看起来越细越淡')
    p.add_argument('--conf-smooth', type=float, default=0.7,
                   help='置信度指数平滑系数, 越大越稳越迟钝')
    p.add_argument('--input-scale', type=float, default=1.0,
                   help='输入图像缩放, 0.5 提速约 4 倍 (K 会同步缩放)')
    p.add_argument('--recover-last', action='store_true',
                   help='INIT 时同时用上次锁定位姿做假设, 便于遮挡后自动恢复')
    p.add_argument('--display-scale', type=float, default=1.0,
                   help='显示窗口缩放, 屏幕小就调到 0.7')
    p.add_argument('--save', default='', help='录制输出视频路径 .avi')
    p.add_argument('--output', default='',
                   help='把每帧位姿样本写入文本文件 (与 stdout 同格式)')
    p.add_argument('--no-print', action='store_true',
                   help='不向 stdout 打印位姿 (仍可 --output / --log-pose)')
    p.add_argument('--print-init', action='store_true',
                   help='INIT 状态也打印样本行 (默认只打印 TRACK/HOLD/LOST)')
    p.add_argument('--log-pose', default='',
                   help='把每帧位姿写入 CSV, 供离线精度分析')
    p.add_argument('--grasp-pose', default='workspace/grasp_cdMo.txt',
                   help='TRACK 稳定时按 g 保存期望抓取位姿 cdMo')
    p.add_argument('--auto', action='store_true', help='启用自动锁定')
    p.add_argument('--lock-dt', type=float, default=1e9)
    p.add_argument('--lock-dr', type=float, default=1e9)
    p.add_argument('--lock-conf', type=float, default=0.5,
                   help='平滑后置信度超过它才锁定')
    p.add_argument('--lock-frames', type=int, default=5)
    p.add_argument('--lock-iters', type=int, default=4,
                   help='锁定瞬间额外精化几遍, 确保完全收敛再接受')
    p.add_argument('--lock-settle-mm', type=float, default=8.0,
                   help='额外精化后位姿还移动超过此值就放弃本次锁定')
    p.add_argument('--lock-settle-deg', type=float, default=8.0)
    p.add_argument('--lost-dt', type=float, default=1e9)
    p.add_argument('--lost-dr', type=float, default=1e9)
    p.add_argument('--lost-conf', type=float, default=0.30)
    p.add_argument('--lost-frames', type=int, default=10)
    return p


def main(argv=None):
    a = build_parser().parse_args(argv)
    os.environ.setdefault('CUDA_VISIBLE_DEVICES', '0')
    sys.path.insert(0, os.getcwd())
    frames, K_sdk = frames_from(a.source)
    tr = Tracker(a)
    if K_sdk is not None:
        tr.K = K_sdk
    tr.run(frames)


if __name__ == '__main__':
    main()
