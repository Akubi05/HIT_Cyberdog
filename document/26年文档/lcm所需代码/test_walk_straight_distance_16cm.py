#!/usr/bin/env python3
"""
直线行走距离测试：默认以 16cm 机身高度、mode=11、gait_id=27 向前走指定距离。

默认用法:
    python3 test_walk_straight_distance_16cm.py --distance 1.0

常用调参:
    python3 test_walk_straight_distance_16cm.py --distance 0.5 --speed 0.10
    python3 test_walk_straight_distance_16cm.py --distance 1.0 --body-height 0.16 --step-height 0.02

说明:
    默认使用 global_to_robot 位姿计算相对起点的前向投影距离，到达目标距离后发送零速站立保持。
    如只想按速度和时间粗略估算距离，可加 --open-loop。
"""

import argparse
import math
import os
import signal
import struct
import sys
import threading
import time

import lcm

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from assets import robot_control_cmd_lcmt, robot_control_response_lcmt
from pc_cmd_send import PC_cmd_send


LOCALIZATION_LCMT_FINGERPRINT = bytes.fromhex("fc48de06e344fb12")
DEFAULT_CMD_LCM_URL = os.environ.get("CMD_LCM_URL", "udpm://239.255.76.66:7667?ttl=1")
DEFAULT_CMD_CHANNEL = os.environ.get("CMD_CHANNEL", "robot_control_cmd")
DEFAULT_PC_SEND_INTERVAL_SEC = float(os.environ.get("PC_CMD_SEND_INTERVAL_SEC", "0.05"))
DEFAULT_POSE_LCM_URL = os.environ.get(
    "GLOBAL_POSE_LCM_URL",
    "udpm://239.255.76.66:7670?ttl=1",
)
DEFAULT_POSE_CHANNEL = os.environ.get("GLOBAL_POSE_LCM_CHANNEL", "global_to_robot")
DEFAULT_RESPONSE_LCM_URL = os.environ.get(
    "ROBOT_RESPONSE_LCM_URL",
    "udpm://239.255.76.66:7670?ttl=1",
)
DEFAULT_RESPONSE_CHANNEL = os.environ.get(
    "ROBOT_RESPONSE_LCM_CHANNEL",
    "robot_control_response",
)

DEFAULT_BODY_HEIGHT_M = 0.16
DEFAULT_STEP_HEIGHT_M = 0.02
DEFAULT_SPEED_MPS = 0.10
DEFAULT_DISTANCE_M = 1.0
DEFAULT_MAX_POSE_AGE_SEC = 0.5
DEFAULT_POSE_TIMEOUT_SEC = 5.0
DEFAULT_CONTROL_INTERVAL_SEC = 0.05
DEFAULT_PRINT_INTERVAL_SEC = 0.3
DEFAULT_STAND_SEC = 3.0
DEFAULT_STOP_HOLD_SEC = 1.0
DEFAULT_LIE_DOWN_HOLD_SEC = 1.5


def dot3(left, right):
    return sum(a * b for a, b in zip(left, right))


def body_forward_from_rpy(roll, pitch, yaw):
    """返回起点机身前向轴在 global 坐标系中的单位向量。"""
    return (
        math.cos(yaw) * math.cos(pitch),
        math.sin(yaw) * math.cos(pitch),
        -math.sin(pitch),
    )


def decode_global_to_robot(data):
    """解析狗端 localization_lcmt: xyz/vxyz/rpy/omegaBody/vBody/timestamp。"""
    if len(data) < 76:
        raise ValueError(f"global_to_robot payload 太短: {len(data)}")
    fingerprint = data[:8]
    if fingerprint != LOCALIZATION_LCMT_FINGERPRINT:
        raise ValueError(f"global_to_robot fingerprint 不匹配: {fingerprint.hex()}")

    values = struct.unpack(">15fq", data[8:76])
    x, y, z = values[0:3]
    vx, vy, vz = values[3:6]
    roll, pitch, yaw = values[6:9]
    return {
        "x": x,
        "y": y,
        "z": z,
        "vx": vx,
        "vy": vy,
        "vz": vz,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "recv_time": time.time(),
    }


class GlobalToRobotReceiver:
    def __init__(self, lcm_url, channel):
        self.lcm_url = lcm_url
        self.channel = channel
        self.lc = lcm.LCM(lcm_url)
        self.lock = threading.Lock()
        self.pose = None
        self.received_count = 0
        self.decode_errors = 0
        self.running = False
        self.thread = None
        self.lc.subscribe(channel, self._callback)

    def _callback(self, channel, data):
        try:
            pose = decode_global_to_robot(data)
        except Exception as exc:
            self.decode_errors += 1
            print(f"!!! 解析 global_to_robot 失败: {exc}")
            return
        with self.lock:
            self.pose = pose
            self.received_count += 1

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        print(f">>> 监听 global_to_robot: {self.lcm_url} / {self.channel}")

    def _loop(self):
        while self.running:
            try:
                self.lc.handle_timeout(50)
            except Exception as exc:
                print(f"!!! LCM 位姿接收异常: {exc}")
                time.sleep(0.05)

    def get_pose(self):
        with self.lock:
            if self.pose is None:
                return None
            return dict(self.pose)

    def wait_for_fresh_pose(self, timeout_sec, max_age_sec):
        deadline = time.time() + float(timeout_sec)
        while time.time() < deadline:
            pose = self.get_pose()
            if pose is not None and time.time() - pose["recv_time"] <= max_age_sec:
                return pose
            time.sleep(0.02)
        return None

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=0.5)


class RobotResponseReceiver:
    def __init__(self, lcm_url, channel):
        self.lcm_url = lcm_url
        self.channel = channel
        self.lc = lcm.LCM(lcm_url)
        self.lock = threading.Lock()
        self.response = None
        self.response_recv_time = 0.0
        self.received_count = 0
        self.decode_errors = 0
        self.running = False
        self.thread = None
        self.lc.subscribe(channel, self._callback)

    def _callback(self, channel, data):
        try:
            response = robot_control_response_lcmt.decode(data)
        except Exception as exc:
            self.decode_errors += 1
            print(f"!!! 解析 robot_control_response 失败: {exc}")
            return
        with self.lock:
            self.response = response
            self.response_recv_time = time.time()
            self.received_count += 1

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        print(f">>> 监听 robot_control_response: {self.lcm_url} / {self.channel}")

    def _loop(self):
        while self.running:
            try:
                self.lc.handle_timeout(50)
            except Exception as exc:
                print(f"!!! LCM 控制反馈接收异常: {exc}")
                time.sleep(0.05)

    def get_response(self):
        with self.lock:
            return self.response

    def format_latest(self):
        response = self.get_response()
        if response is None:
            return "未收到控制反馈"
        motor_errors = [int(v) for v in response.motor_error if int(v) != 0]
        motor_summary = "none" if not motor_errors else ",".join(str(v) for v in motor_errors[:4])
        return (
            f"mode/gait={int(response.mode)}/{int(response.gait_id)}, "
            f"bar={int(response.order_process_bar)}, switch={int(response.switch_status)}, "
            f"contact={int(response.contact)}, ori_err={int(response.ori_error)}, "
            f"foot_err={int(response.footpos_error)}, motor_err={motor_summary}"
        )

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=0.5)


def make_cmd(
    *,
    mode,
    gait_id,
    vel=None,
    pos_z=0.0,
    step_height=None,
    contact=15,
    duration=0,
):
    msg = robot_control_cmd_lcmt()
    msg.mode = int(mode)
    msg.gait_id = int(gait_id)
    msg.contact = int(contact)
    msg.life_count = 1
    msg.value = 0
    msg.duration = int(duration)
    msg.vel_des = list(vel or [0.0, 0.0, 0.0])
    msg.rpy_des = [0.0, 0.0, 0.0]
    msg.pos_des = [0.0, 0.0, float(pos_z)]
    msg.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg.ctrl_point = [0.0, 0.0, 0.0]
    msg.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg.step_height = list(step_height or [0.0, 0.0])
    return msg


def send_for(ctrl, msg, seconds, interval):
    deadline = time.time() + float(seconds)
    while time.time() < deadline:
        ctrl.Send_cmd(msg)
        time.sleep(interval)


def projected_forward_distance(start_pose, pose, direction_sign):
    forward_axis = body_forward_from_rpy(
        start_pose["roll"],
        start_pose["pitch"],
        start_pose["yaw"],
    )
    displacement = (
        pose["x"] - start_pose["x"],
        pose["y"] - start_pose["y"],
        pose["z"] - start_pose["z"],
    )
    return direction_sign * dot3(displacement, forward_axis)


def print_lcm_diagnostics(receiver, response_receiver, prefix=">>>"):
    pose_count = receiver.received_count if receiver is not None else 0
    pose_errors = receiver.decode_errors if receiver is not None else 0
    response_count = response_receiver.received_count if response_receiver is not None else 0
    response_errors = response_receiver.decode_errors if response_receiver is not None else 0
    response_text = (
        response_receiver.format_latest()
        if response_receiver is not None
        else "未启用控制反馈监听"
    )
    print(
        f"{prefix} LCM 诊断: pose_count={pose_count}, pose_decode_errors={pose_errors}, "
        f"response_count={response_count}, response_decode_errors={response_errors}, "
        f"latest_response=({response_text})"
    )


def run_walk(ctrl, receiver, response_receiver, args, exit_requested):
    direction_sign = 1.0 if args.direction == "forward" else -1.0
    walk_vel = [direction_sign * args.speed, 0.0, 0.0]

    print(
        ">>> 直线距离测试参数: "
        f"distance={args.distance:.2f}m, speed={args.speed:.2f}m/s, "
        f"direction={args.direction}, body_height={args.body_height:.3f}m, "
        f"mode/gait={args.walk_mode}/{args.gait_id}"
    )

    if not args.open_loop:
        print(f">>> 预检查 global_to_robot 位姿流 {args.precheck_sec:.1f}s")
        precheck_pose = receiver.wait_for_fresh_pose(args.precheck_sec, args.max_pose_age)
        if precheck_pose is None:
            print("!!! 预检查未收到新鲜 global_to_robot，继续尝试恢复站立并打印反馈")
            print_lcm_diagnostics(receiver, response_receiver, prefix="!!!")
        else:
            print(
                ">>> 位姿流正常: "
                f"x={precheck_pose['x']:.3f}, y={precheck_pose['y']:.3f}, "
                f"yaw={math.degrees(precheck_pose['yaw']):.1f}deg"
            )

    stand_msg = make_cmd(
        mode=12,
        gait_id=0,
        vel=[0.0, 0.0, 0.0],
        pos_z=0.0,
        step_height=[0.0, 0.0],
        contact=0,
    )
    print(f">>> 先恢复站立 {args.stand_sec:.1f}s")
    send_for(ctrl, stand_msg, args.stand_sec, args.control_interval)
    print_lcm_diagnostics(receiver, response_receiver)

    if args.stand_only:
        hold_msg = make_cmd(
            mode=args.stop_mode,
            gait_id=args.stop_gait_id,
            vel=[0.0, 0.0, 0.0],
            pos_z=args.body_height,
            step_height=[0.0, 0.0],
            contact=15,
        )
        print(f">>> stand-only: 零速站立保持 {args.stop_hold_sec:.1f}s")
        send_for(ctrl, hold_msg, args.stop_hold_sec, args.control_interval)
        print_lcm_diagnostics(receiver, response_receiver)
        return True

    start_pose = None
    if not args.open_loop:
        start_pose = receiver.wait_for_fresh_pose(args.pose_timeout, args.max_pose_age)
        if start_pose is None:
            print_lcm_diagnostics(receiver, response_receiver, prefix="!!!")
            raise RuntimeError(
                "没有收到新鲜 global_to_robot 位姿，无法按距离闭环；"
                "请先确认狗端 dog_cmd_recv.py 转发脚本正在运行；"
                "如需纯定时估算可加 --open-loop"
            )
        print(
            ">>> 起点位姿: "
            f"x={start_pose['x']:.3f}, y={start_pose['y']:.3f}, z={start_pose['z']:.3f}, "
            f"yaw={math.degrees(start_pose['yaw']):.1f}deg"
        )

    walk_msg = make_cmd(
        mode=args.walk_mode,
        gait_id=args.gait_id,
        vel=walk_vel,
        pos_z=args.body_height,
        step_height=[args.step_height, args.step_height],
        contact=15,
    )
    max_duration = (
        args.max_duration
        if args.max_duration is not None
        else max(8.0, args.distance / args.speed * 3.0 + 3.0)
    )
    start_time = time.time()
    last_print_time = 0.0
    progress = 0.0

    print(
        ">>> 开始直走: "
        f"vel=[{walk_vel[0]:.3f}, {walk_vel[1]:.3f}, {walk_vel[2]:.3f}], "
        f"target={args.distance:.3f}m, timeout={max_duration:.1f}s"
    )
    while not exit_requested["value"]:
        now = time.time()
        elapsed = now - start_time
        if elapsed >= max_duration:
            print(f"!!! 直走超时: elapsed={elapsed:.2f}s, progress={progress:.3f}m")
            return False

        if args.open_loop:
            progress = args.speed * elapsed
        else:
            pose = receiver.get_pose()
            if pose is None:
                time.sleep(0.02)
                continue
            pose_age = now - pose["recv_time"]
            if pose_age > args.max_pose_age:
                print(f"!!! global_to_robot 位姿过旧: age={pose_age:.3f}s")
                return False
            progress = projected_forward_distance(start_pose, pose, direction_sign)

        remaining = args.distance - progress
        if remaining <= args.tolerance:
            print(
                ">>> 到达目标距离: "
                f"progress={progress:.3f}m, target={args.distance:.3f}m, "
                f"elapsed={elapsed:.2f}s"
            )
            return True

        ctrl.Send_cmd(walk_msg)
        if now - last_print_time >= args.print_interval:
            print(
                ">>> 直走中: "
                f"progress={progress:.3f}/{args.distance:.3f}m, "
                f"remaining={remaining:.3f}m, elapsed={elapsed:.2f}s"
            )
            last_print_time = now
        time.sleep(args.control_interval)

    print(">>> 用户中断直走")
    return False


def stop_or_lie_down(ctrl, args):
    stop_msg = make_cmd(
        mode=args.stop_mode,
        gait_id=args.stop_gait_id,
        vel=[0.0, 0.0, 0.0],
        pos_z=args.body_height,
        step_height=[0.0, 0.0],
        contact=15,
    )
    print(
        ">>> 发送零速站立保持: "
        f"mode={args.stop_mode}, gait_id={args.stop_gait_id}, "
        f"body_height={args.body_height:.3f}m, hold={args.stop_hold_sec:.1f}s"
    )
    send_for(ctrl, stop_msg, args.stop_hold_sec, args.control_interval)

    if args.lie_down:
        lie_down_msg = make_cmd(
            mode=7,
            gait_id=0,
            vel=[0.0, 0.0, 0.0],
            pos_z=0.0,
            step_height=[0.0, 0.0],
            contact=15,
        )
        print(f">>> 发送高阻尼趴下: hold={args.lie_down_hold_sec:.1f}s")
        send_for(ctrl, lie_down_msg, args.lie_down_hold_sec, args.control_interval)


def parse_args():
    parser = argparse.ArgumentParser(
        description="16cm 机身高度 mode=11/gait_id=27 直线行走距离测试。"
    )
    parser.add_argument("--distance", type=float, default=DEFAULT_DISTANCE_M, help="目标直走距离，单位 m")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_MPS, help="直走速度，单位 m/s")
    parser.add_argument(
        "--direction",
        choices=("forward", "backward"),
        default="forward",
        help="直走方向，默认 forward",
    )
    parser.add_argument("--body-height", type=float, default=DEFAULT_BODY_HEIGHT_M, help="行走机身高度，单位 m")
    parser.add_argument("--step-height", type=float, default=DEFAULT_STEP_HEIGHT_M, help="行走步高，单位 m")
    parser.add_argument("--walk-mode", type=int, default=11, help="行走 mode，默认 11")
    parser.add_argument("--gait-id", type=int, default=27, help="行走 gait_id，默认 27")
    parser.add_argument("--stop-mode", type=int, default=11, help="结束零速保持 mode，默认 11")
    parser.add_argument("--stop-gait-id", type=int, default=1, help="结束零速保持 gait_id，默认 1")
    parser.add_argument("--stand-sec", type=float, default=DEFAULT_STAND_SEC, help="开始前站立等待时间，单位 s")
    parser.add_argument("--precheck-sec", type=float, default=1.0, help="动作前预检查位姿流时间，单位 s")
    parser.add_argument("--stand-only", action="store_true", help="只执行恢复站立和零速站立保持，不进入直走")
    parser.add_argument("--diagnose-only", action="store_true", help="只监听位姿和控制反馈，不发送任何控制命令")
    parser.add_argument("--stop-hold-sec", type=float, default=DEFAULT_STOP_HOLD_SEC, help="结束零速保持时间，单位 s")
    parser.add_argument("--lie-down", action="store_true", help="测试结束后发送 mode=7/gait_id=0 趴下")
    parser.add_argument("--lie-down-hold-sec", type=float, default=DEFAULT_LIE_DOWN_HOLD_SEC, help="趴下命令保持时间，单位 s")
    parser.add_argument("--open-loop", action="store_true", help="不使用 global_to_robot，按 speed*time 粗略估算距离")
    parser.add_argument("--tolerance", type=float, default=0.03, help="距离到达容差，单位 m")
    parser.add_argument("--max-duration", type=float, default=None, help="最大行走时长，单位 s；默认按距离和速度自动估算")
    parser.add_argument("--pose-timeout", type=float, default=DEFAULT_POSE_TIMEOUT_SEC, help="等待 global_to_robot 起点位姿超时，单位 s")
    parser.add_argument("--max-pose-age", type=float, default=DEFAULT_MAX_POSE_AGE_SEC, help="global_to_robot 最大允许年龄，单位 s")
    parser.add_argument("--control-interval", type=float, default=DEFAULT_CONTROL_INTERVAL_SEC, help="控制循环间隔，单位 s")
    parser.add_argument("--print-interval", type=float, default=DEFAULT_PRINT_INTERVAL_SEC, help="日志打印间隔，单位 s")
    parser.add_argument("--cmd-lcm-url", default=DEFAULT_CMD_LCM_URL, help="robot_control_cmd LCM URL")
    parser.add_argument("--cmd-channel", default=DEFAULT_CMD_CHANNEL, help="robot_control_cmd channel")
    parser.add_argument("--pc-send-interval", type=float, default=DEFAULT_PC_SEND_INTERVAL_SEC, help="PC 后台重发周期，单位 s")
    parser.add_argument("--pose-lcm-url", default=DEFAULT_POSE_LCM_URL, help="global_to_robot LCM URL")
    parser.add_argument("--pose-channel", default=DEFAULT_POSE_CHANNEL, help="global_to_robot channel")
    parser.add_argument("--response-lcm-url", default=DEFAULT_RESPONSE_LCM_URL, help="robot_control_response LCM URL")
    parser.add_argument("--response-channel", default=DEFAULT_RESPONSE_CHANNEL, help="robot_control_response channel")
    args = parser.parse_args()

    if args.distance <= 0.0:
        parser.error("--distance 必须大于 0")
    if args.speed <= 0.0:
        parser.error("--speed 必须大于 0")
    if not 0.12 <= args.body_height <= 0.28:
        parser.error("--body-height 建议保持在 0.12～0.28m")
    if args.step_height < 0.0:
        parser.error("--step-height 不能小于 0")
    if args.stand_sec < 0.0 or args.stop_hold_sec < 0.0 or args.precheck_sec < 0.0:
        parser.error("--stand-sec、--stop-hold-sec 和 --precheck-sec 不能小于 0")
    if args.lie_down_hold_sec <= 0.0:
        parser.error("--lie-down-hold-sec 必须大于 0")
    if args.tolerance <= 0.0:
        parser.error("--tolerance 必须大于 0")
    if args.max_duration is not None and args.max_duration <= 0.0:
        parser.error("--max-duration 必须大于 0")
    if args.pose_timeout <= 0.0 or args.max_pose_age <= 0.0:
        parser.error("--pose-timeout 和 --max-pose-age 必须大于 0")
    if args.control_interval <= 0.0 or args.print_interval <= 0.0:
        parser.error("--control-interval 和 --print-interval 必须大于 0")
    if args.pc_send_interval <= 0.0:
        parser.error("--pc-send-interval 必须大于 0")
    return args


def main():
    args = parse_args()
    exit_requested = {"value": False}

    def on_signal(signum, frame):
        exit_requested["value"] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    receiver = None
    response_receiver = None
    ctrl = None
    try:
        if not args.open_loop:
            receiver = GlobalToRobotReceiver(args.pose_lcm_url, args.pose_channel)
            receiver.start()
        response_receiver = RobotResponseReceiver(args.response_lcm_url, args.response_channel)
        response_receiver.start()

        if args.diagnose_only:
            wait_sec = max(args.precheck_sec, 3.0)
            print(f">>> diagnose-only: 只监听 {wait_sec:.1f}s，不发送控制命令")
            time.sleep(wait_sec)
            print_lcm_diagnostics(receiver, response_receiver)
            has_pose = receiver is not None and receiver.received_count > 0
            has_response = response_receiver.received_count > 0
            return 0 if has_pose or has_response else 1

        ctrl = PC_cmd_send(
            lcm_url=args.cmd_lcm_url,
            cmd_channel=args.cmd_channel,
            send_interval=args.pc_send_interval,
        )
        ctrl.run()
        time.sleep(0.5)

        ok = run_walk(ctrl, receiver, response_receiver, args, exit_requested)
        return 0 if ok and not exit_requested["value"] else 1
    except KeyboardInterrupt:
        print("\n>>> 用户中断")
        return 130
    finally:
        if ctrl is not None:
            try:
                stop_or_lie_down(ctrl, args)
            finally:
                ctrl.stop()
        if receiver is not None:
            receiver.stop()
        if response_receiver is not None:
            response_receiver.stop()
        print(">>> 直线行走距离测试结束")


if __name__ == "__main__":
    sys.exit(main())
