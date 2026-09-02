#!/usr/bin/env python3
"""Intel RealSense 彩色流采集, 供 live_track.py 调用。

不在 import 时加载 pyrealsense2 / cv2: 打开相机或转码时再导入。
"""
import numpy as np


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
