#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Slow open/close gripper with OpenArm CAN Library (SocketCAN).

要点（对应文档）：
1) enable/param 查询等慢操作用更长 recv 超时（1000~2000us）
2) 控制/状态回包用 STATE 模式，并在每次命令后调用 recv_all(timeout_us)
3) 为了“缓慢开合”，用 MIT 位置控制做线性插值 + 限频 sleep
"""

import time
import signal
import sys
import openarm_can as oa

# ----------- 用户需要按实际硬件修改的配置 -----------
CAN_IFACE = "can0"
USE_CANFD = True  # 文档 demo 默认 CANFD；如果你没开 CANFD 改 False

# 只初始化夹爪（如果你还有 arm 电机，也可以一并 init_arm_motors）
GRIPPER_MOTOR_TYPE = oa.MotorType.DM4310
GRIPPER_SEND_ID = 0x08
GRIPPER_RECV_ID = 0x18

# ----------- 超时与控制频率（对应文档建议）-----------
SLOW_TIMEOUT_US = 2000  # enable/disable/参数查询等慢操作
FAST_TIMEOUT_US = 500   # 控制/状态回包

CONTROL_HZ = 200        # 建议 200~500 更稳；8 电机不要 >1000Hz
DT = 1.0 / CONTROL_HZ

# ----------- “开合目标位置”需要你按实际夹爪标定 -----------
# 这里的 q 是“电机位置目标”，单位/方向取决于电机与夹爪机构标定方式。
# 你必须根据自己的夹爪：哪个位置是“打开”，哪个是“闭合”，以及安全行程范围。
Q_OPEN = 0.0
Q_CLOSE = 0.8

# 运动耗时（越大越慢）
MOVE_DURATION_S = 3.0

# PD 参数：为了动作柔和，kp/kd 可以从小开始；太小可能推不动，太大可能很硬/冲击
KP = 20
KD = 2

# 力矩前馈（一般先 0；需要更大夹持力时再加，但注意安全）
TAU_FF = 0.05

_stop = False

def _handle_sigint(sig, frame):
    global _stop
    _stop = True

signal.signal(signal.SIGINT, _handle_sigint)
signal.signal(signal.SIGTERM, _handle_sigint)

def recv_all(arm: oa.OpenArm, timeout_us: int, note: str = ""):
    """每次发命令后都要 recv_all，用 timeout 控制等待回包的时间跨度。"""
    if note:
        print(f"[recv] {note} timeout={timeout_us}us")
    arm.recv_all(timeout_us)

def lerp(a: float, b: float, t: float) -> float:
    """线性插值：t∈[0,1]"""
    return a + (b - a) * t

def move_gripper_slow(openarm: oa.OpenArm, q_from: float, q_to: float, duration_s: float):
    """
    缓慢移动夹爪：把目标位置从 q_from 平滑插值到 q_to。
    每一步都发 MIT 控制，并 recv_all + sleep，保证动作慢且通信稳定。
    """
    gripper = openarm.get_gripper()

    steps = max(1, int(duration_s * CONTROL_HZ))
    start_t = time.perf_counter()

    for i in range(steps + 1):
        if _stop:
            return

        loop_start = time.perf_counter()

        t = i / steps
        q_cmd = lerp(q_from, q_to, t)

        # 关键：STATE 模式下，mit_control_all() 的回包应按状态解析
        # 对夹爪只有一个电机，所以传入单个 MITParam
        gripper.mit_control_all([
            oa.MITParam(KP, KD, q_cmd, 0.0, TAU_FF)
        ])

        # 关键：控制命令后立刻 recv，处理电机回包并更新状态
        recv_all(openarm, FAST_TIMEOUT_US, note=f"mit q={q_cmd:.3f}")

        # 可选：打印当前位置（需要你的绑定提供 get_position）
        try:
            m = gripper.get_motors()[0]
            print(f"[state] target={q_cmd:.3f} pos={m.get_position():.3f}")
        except Exception:
            pass

        # 限频：保证循环频率不超过设定（文档强调电机多时不要太快）
        elapsed = time.perf_counter() - loop_start
        remain = DT - elapsed
        if remain > 0:
            time.sleep(remain)

    total = time.perf_counter() - start_t
    print(f"[move] done in {total:.2f}s ({steps} steps @ {CONTROL_HZ}Hz)")

def main():
    print("[1] Create OpenArm")
    openarm = oa.OpenArm(CAN_IFACE, USE_CANFD)

    print("[2] Init gripper motor")
    openarm.init_gripper_motor(GRIPPER_MOTOR_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID)

    print("[3] Enable motor (IGNORE, slow timeout)")
    openarm.set_callback_mode_all(oa.CallbackMode.IGNORE)
    openarm.enable_all()
    recv_all(openarm, SLOW_TIMEOUT_US, note="enable_all")

    print("[4] Switch to STATE mode")
    openarm.set_callback_mode_all(oa.CallbackMode.STATE)

    # 可选：尝试读取当前位作为起点（读不到就用 Q_OPEN）
    q_current = Q_OPEN
    try:
        openarm.refresh_all()
        recv_all(openarm, FAST_TIMEOUT_US, note="refresh_all (pre)")
        q_current = openarm.get_gripper().get_motors()[0].get_position()
        print(f"[info] current position = {q_current:.3f}")
    except Exception:
        print("[info] cannot read current position; use Q_OPEN as start")

    print("[5] One-shot: slowly OPEN, then slowly CLOSE")
    if not _stop:
        print("[action] slowly OPEN")
        move_gripper_slow(openarm, q_current, Q_OPEN, MOVE_DURATION_S)
        q_current = Q_OPEN

    time.sleep(0.5)

    if not _stop:
        print("[action] slowly CLOSE")
        move_gripper_slow(openarm, q_current, Q_CLOSE, MOVE_DURATION_S)
        q_current = Q_CLOSE

    print("[6] Clean shutdown: disable_all")
    openarm.set_callback_mode_all(oa.CallbackMode.IGNORE)
    openarm.disable_all()
    recv_all(openarm, SLOW_TIMEOUT_US, note="disable_all")
    print("[done]")