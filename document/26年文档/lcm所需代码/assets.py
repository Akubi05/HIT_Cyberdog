

from io import BytesIO
import math
import struct
from typing import Sequence

try:
    import cv2
except Exception as exc:
    cv2 = None
    CV2_IMPORT_ERROR = exc
import numpy as np

# 中文注释：视觉比例计算的默认防零除常量，避免 ROI 面积极小时 ratio 不稳定。
DEFAULT_RATIO_EPSILON = 1e-5

# 中文注释：LCM 控制命令结构体，负责把步态、速度、姿态和时长编码成底层控制协议。
class robot_control_cmd_lcmt(object):
    __slots__ = [
        "mode", "gait_id", "contact", "life_count", "vel_des", "rpy_des",
        "pos_des", "acc_des", "ctrl_point", "foot_pose", "step_height",
        "value", "duration",
    ]

    # 中文注释：初始化控制命令字段，确保所有协议数组都有固定长度默认值。
    def __init__(self):
        self.mode = 0
        self.gait_id = 0
        self.contact = 0
        self.life_count = 0
        self.vel_des = [0.0 for _ in range(3)]
        self.rpy_des = [0.0 for _ in range(3)]
        self.pos_des = [0.0 for _ in range(3)]
        self.acc_des = [0.0 for _ in range(6)]
        self.ctrl_point = [0.0 for _ in range(3)]
        self.foot_pose = [0.0 for _ in range(6)]
        self.step_height = [0.0 for _ in range(2)]
        self.value = 0
        self.duration = 0

    # 中文注释：把当前控制命令打包为带 fingerprint 的二进制 LCM payload。
    def encode(self):
        buf = BytesIO()
        buf.write(robot_control_cmd_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    # 中文注释：按 LCM 定义的大端布局逐字段写入命令 payload。
    def _encode_one(self, buf):
        buf.write(struct.pack(">bbbb", self.mode, self.gait_id, self.contact, self.life_count))
        buf.write(struct.pack(">3f", *self.vel_des[:3]))
        buf.write(struct.pack(">3f", *self.rpy_des[:3]))
        buf.write(struct.pack(">3f", *self.pos_des[:3]))
        buf.write(struct.pack(">6f", *self.acc_des[:6]))
        buf.write(struct.pack(">3f", *self.ctrl_point[:3]))
        buf.write(struct.pack(">6f", *self.foot_pose[:6]))
        buf.write(struct.pack(">2f", *self.step_height[:2]))
        buf.write(struct.pack(">ii", self.value, self.duration))

    # 中文注释：校验 fingerprint 后把二进制 payload 解码成命令对象。
    @staticmethod
    def decode(data):
        buf = data if hasattr(data, "read") else BytesIO(data)
        if buf.read(8) != robot_control_cmd_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return robot_control_cmd_lcmt._decode_one(buf)

    # 中文注释：按 LCM 字段顺序从缓冲区恢复命令对象。
    @staticmethod
    def _decode_one(buf):
        self = robot_control_cmd_lcmt()
        self.mode, self.gait_id, self.contact, self.life_count = struct.unpack(">bbbb", buf.read(4))
        self.vel_des = struct.unpack(">3f", buf.read(12))
        self.rpy_des = struct.unpack(">3f", buf.read(12))
        self.pos_des = struct.unpack(">3f", buf.read(12))
        self.acc_des = struct.unpack(">6f", buf.read(24))
        self.ctrl_point = struct.unpack(">3f", buf.read(12))
        self.foot_pose = struct.unpack(">6f", buf.read(24))
        self.step_height = struct.unpack(">2f", buf.read(8))
        self.value, self.duration = struct.unpack(">ii", buf.read(8))
        return self

    # 中文注释：返回 LCM 类型 fingerprint 哈希，保持与底层协议一致。
    @staticmethod
    def _get_hash_recursive(parents):
        if robot_control_cmd_lcmt in parents:
            return 0
        tmphash = (0x476b61e296af96f5) & 0xffffffffffffffff
        return (((tmphash << 1) & 0xffffffffffffffff) + (tmphash >> 63)) & 0xffffffffffffffff

    _packed_fingerprint = None

    # 中文注释：缓存并返回命令类型 fingerprint 的二进制表示。
    @staticmethod
    def _get_packed_fingerprint():
        if robot_control_cmd_lcmt._packed_fingerprint is None:
            robot_control_cmd_lcmt._packed_fingerprint = struct.pack(
                ">Q",
                robot_control_cmd_lcmt._get_hash_recursive([]),
            )
        return robot_control_cmd_lcmt._packed_fingerprint

# 中文注释：LCM 控制反馈结构体，负责解析底层控制器返回的模式、进度和错误状态。
class robot_control_response_lcmt(object):
    __slots__ = [
        "mode", "gait_id", "contact", "order_process_bar", "switch_status",
        "ori_error", "footpos_error", "motor_error",
    ]

    # 中文注释：初始化控制反馈字段，默认无错误且 motor_error 长度固定。
    def __init__(self):
        self.mode = 0
        self.gait_id = 0
        self.contact = 0
        self.order_process_bar = 0
        self.switch_status = 0
        self.ori_error = 0
        self.footpos_error = 0
        self.motor_error = [0 for _ in range(12)]

    # 中文注释：把反馈对象编码成 LCM payload，主要用于协议兼容。
    def encode(self):
        buf = BytesIO()
        buf.write(robot_control_response_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    # 中文注释：按反馈协议字段顺序写入二进制缓冲区。
    def _encode_one(self, buf):
        buf.write(struct.pack(
            ">bbbbbbh",
            self.mode,
            self.gait_id,
            self.contact,
            self.order_process_bar,
            self.switch_status,
            self.ori_error,
            self.footpos_error,
        ))
        buf.write(struct.pack(">12i", *self.motor_error[:12]))

    # 中文注释：校验反馈 fingerprint 并反序列化底层响应。
    @staticmethod
    def decode(data):
        buf = data if hasattr(data, "read") else BytesIO(data)
        if buf.read(8) != robot_control_response_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return robot_control_response_lcmt._decode_one(buf)

    # 中文注释：从反馈 payload 中恢复模式、进度和电机错误数组。
    @staticmethod
    def _decode_one(buf):
        self = robot_control_response_lcmt()
        (
            self.mode,
            self.gait_id,
            self.contact,
            self.order_process_bar,
            self.switch_status,
            self.ori_error,
            self.footpos_error,
        ) = struct.unpack(">bbbbbbh", buf.read(8))
        self.motor_error = struct.unpack(">12i", buf.read(48))
        return self

    # 中文注释：返回反馈类型 fingerprint 哈希，匹配 LCM 类型定义。
    @staticmethod
    def _get_hash_recursive(parents):
        if robot_control_response_lcmt in parents:
            return 0
        tmphash = (0x485da98216eda8c7) & 0xffffffffffffffff
        return (((tmphash << 1) & 0xffffffffffffffff) + (tmphash >> 63)) & 0xffffffffffffffff

    _packed_fingerprint = None

    # 中文注释：缓存并返回反馈类型 fingerprint 的二进制表示。
    @staticmethod
    def _get_packed_fingerprint():
        if robot_control_response_lcmt._packed_fingerprint is None:
            robot_control_response_lcmt._packed_fingerprint = struct.pack(
                ">Q",
                robot_control_response_lcmt._get_hash_recursive([]),
            )
        return robot_control_response_lcmt._packed_fingerprint

# 中文注释：角度归一化工具，把弧度约束到 [-pi, pi] 区间。
def normalize_angle(angle):
    angle = float(angle)
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# 中文注释：线段角度归一化工具，把视觉线角限制到可比较的 [-90, 90] 度。
def normalize_line_angle(angle_deg):
    angle_deg = float(angle_deg)
    if angle_deg > 90.0:
        angle_deg -= 180.0
    elif angle_deg < -90.0:
        angle_deg += 180.0
    return angle_deg

# 中文注释：二维向量正向旋转，用于 body/global 坐标变换。
def rotate_vector(x, y, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * x - sin_yaw * y,
        sin_yaw * x + cos_yaw * y,
    )

# 中文注释：二维向量逆向旋转，把 global 方向投影回机体坐标。
def inverse_rotate_vector(x, y, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * x + sin_yaw * y,
        -sin_yaw * x + cos_yaw * y,
    )

# 中文注释：对平面点执行旋转和平移，完成 abs/global 坐标正变换。
def transform_xy(x, y, dx, dy, dyaw_deg):
    rot_x, rot_y = rotate_vector(float(x), float(y), math.radians(float(dyaw_deg)))
    return rot_x + float(dx), rot_y + float(dy)

# 中文注释：对平面点执行逆平移和逆旋转，完成 abs/global 坐标反变换。
def inverse_transform_xy(x, y, dx, dy, dyaw_deg):
    local_x = float(x) - float(dx)
    local_y = float(y) - float(dy)
    return inverse_rotate_vector(local_x, local_y, math.radians(float(dyaw_deg)))

# 中文注释：圆周均值工具，避免跨 pi 边界时 yaw 平均出错。
def circular_mean_angle(angles: Sequence[float]):
    if not angles:
        return 0.0
    sin_sum = sum(math.sin(angle) for angle in angles)
    cos_sum = sum(math.cos(angle) for angle in angles)
    if abs(sin_sum) < 1e-12 and abs(cos_sum) < 1e-12:
        return normalize_angle(angles[0])
    return normalize_angle(math.atan2(sin_sum, cos_sum))

# 中文注释：计算一组角度围绕中心值的最大离散跨度。
def angle_spread_around(angles: Sequence[float], center):
    if not angles:
        return 0.0
    deltas = [normalize_angle(angle - center) for angle in angles]
    return max(deltas) - min(deltas)

# 中文注释：拆分绝对目标点，兼容只给 x/y 或额外给 yaw 的调用。
def split_abs_target_xy_yaw(abs_target):
    if abs_target is None:
        return None, None, None
    values = tuple(map(float, abs_target))
    if len(values) < 2:
        raise ValueError("absolute target requires at least (x, y)")
    target_abs_x, target_abs_y = values[:2]
    target_abs_yaw_deg = values[2] if len(values) >= 3 else None
    return target_abs_x, target_abs_y, target_abs_yaw_deg

# 中文注释：按主轴进度判断 DeepDog 是否到达目标，避免斜向误差误判。
def deepdog_axis_arrival(start_xy, current_xy, target_dx, target_dy, axis=None, tolerance=0.0):
    axis_key = "auto" if axis is None else str(axis).strip().lower()
    if axis_key in ("auto", "dominant", "main"):
        axis_key = "x" if abs(target_dx) >= abs(target_dy) else "y"
    if axis_key not in ("x", "y"):
        raise ValueError(f"unknown DeepDog arrival axis: {axis!r}")

    axis_index = 0 if axis_key == "x" else 1
    target_delta = float(target_dx if axis_key == "x" else target_dy)
    current_delta = float(current_xy[axis_index] - start_xy[axis_index])
    direction = 1.0 if target_delta >= 0.0 else -1.0
    target_distance = abs(target_delta)
    progress = direction * current_delta
    remaining = target_distance - progress
    tolerance = max(float(tolerance), 0.0)
    return {
        "axis": axis_key,
        "direction": direction,
        "target_delta_m": target_delta,
        "target_m": target_distance,
        "progress_m": progress,
        "remaining_m": remaining,
        "reached": remaining <= tolerance,
    }

# 中文注释：把四元数姿态转换为 roll/pitch/yaw。
def quaternion_to_rpy(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

# 中文注释：从四元数提取平面朝向，并在退化姿态时使用 fallback。
def quaternion_planar_heading(qx, qy, qz, qw, fallback=None):
    norm_q = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm_q <= 1e-9:
        return fallback
    qx /= norm_q
    qy /= norm_q
    qz /= norm_q
    qw /= norm_q
    world_x = (
        1.0 - 2.0 * (qy * qy + qz * qz),
        2.0 * (qx * qy + qw * qz),
        2.0 * (qx * qz - qw * qy),
    )
    planar_norm = math.hypot(world_x[0], world_x[1])
    if planar_norm < 1e-9:
        return fallback
    return math.atan2(world_x[1], world_x[0])

# 中文注释：统一从图像对象或 shape 元组里取高度和宽度。
def image_hw(image_or_shape):
    shape = image_or_shape if isinstance(image_or_shape, tuple) else image_or_shape.shape
    return int(shape[0]), int(shape[1])

# 中文注释：把相对比例 ROI 转换成 OpenCV 可用的像素多边形。
def polygon_from_ratios(image_or_shape, ratios):
    height, width = image_hw(image_or_shape)
    return np.array(
        [[int(width * x), int(height * y)] for x, y in ratios],
        dtype=np.int32,
    )

# 中文注释：生成 HSV 阈值 mask，并可选做形态学去噪/闭合。
def hsv_range_mask(hsv, lower, upper, *, kernel_size=None, close=False, kernel_shape=None):
    if cv2 is None:
        raise RuntimeError(f'OpenCV 不可用，不能执行 hsv_range_mask: {CV2_IMPORT_ERROR}')
    if kernel_shape is None:
        kernel_shape = cv2.MORPH_RECT
    mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
    if kernel_size is None:
        return mask
    kernel = cv2.getStructuringElement(kernel_shape, tuple(kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    if close:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

# 中文注释：按多边形 ROI 生成二值 mask。
def polygon_mask(image_or_shape, points):
    if cv2 is None:
        raise RuntimeError(f'OpenCV 不可用，不能执行 polygon_mask: {CV2_IMPORT_ERROR}')
    height, width = image_hw(image_or_shape)
    roi_mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(roi_mask, [points], 255)
    return roi_mask

# 中文注释：统计 mask 在全图或 ROI 内的占比，作为视觉触发特征。
def mask_ratio(mask, roi_mask=None, epsilon=DEFAULT_RATIO_EPSILON):
    if roi_mask is None:
        return np.count_nonzero(mask) / (mask.size + epsilon)
    if cv2 is None:
        raise RuntimeError(f'OpenCV 不可用，不能执行 mask_ratio ROI 计算: {CV2_IMPORT_ERROR}')
    masked = cv2.bitwise_and(mask, mask, mask=roi_mask)
    return np.count_nonzero(masked) / (np.count_nonzero(roi_mask) + epsilon)
