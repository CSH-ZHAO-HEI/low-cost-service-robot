#!/usr/bin/env python3
"""
Capability tests for composite tasks C1-C4.
Extends ManipTester with reset / judge logic for multi-step pick-and-place tasks.

Task definitions (manual baseline, no LLM, no VLM):
  C1  Pick Coke can from BalconyTable_01_001, put on NightStand_01_001
  C2  Pick red_block from NightStand_01_002, put between blue/yellow
  C3  Pick all ground blocks (red/blue/yellow), put in Trash_01_001
  C4  Navigate ±0.5 m square around CoffeeTable; pick red_block when found → trash

Prerequisites:
  1. ./run_gazebo.sh
  2. ./run_rtab.sh
  3. ./run_teb.sh
  4. rosrun small_brain_sim arm_task_server.py
  5. python3 get_scene.py

Usage:
  python3 C1-C4/composite_capability_test.py C1
  python3 C1-C4/composite_capability_test.py --all
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Dict, List, Tuple

import rospy
import yaml
from gazebo_msgs.msg import ModelState, LinkStates
from gazebo_msgs.srv import DeleteModel, GetModelState, SetModelState, SpawnModel
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
G1_G4_DIR     = os.path.join(PROJECT_ROOT, "experiments", "G1-G4")
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
SCENE_PATH    = os.path.join(PROJECT_ROOT, "gazebo_scene.yaml")
MODEL_DIR     = os.path.join(PROJECT_ROOT, "small_brain", "models")

for _p in [G1_G4_DIR, BIG_BRAIN_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

# ── 物件名稱 ─────────────────────────────────────────────────────
RED        = "red_block"
BLUE       = "blue_block"
YELLOW     = "yellow_block"
CUP        = "Coke"
COFFEETABLE = "CoffeeTable_01_001"
NIGHTSTAND  = "NightStand_01_001"
NIGHTSTAND2 = "NightStand_01_002"
TRASH       = "Trash_01_001"
ROBOT       = "mini_mec_six_arm"

# ── 尺寸常數 ─────────────────────────────────────────────────────
BLOCK_HALF      = 0.025
CUP_HALF        = 0.058

# snap_down = 讓夾爪中心對準物件中心的偏移量（model origin = 底部）
CUP_SNAP_DOWN   = CUP_HALF    # C1 可樂罐：罐高 11.6cm / 2
BLOCK_SNAP_DOWN = -0.01       # C2 方塊：往上推，讓方塊進入夾爪（可微調）

# ── 手臂姿態 ─────────────────────────────────────────────────────
ARM_CLAMP_POSE    = [0.0, -1.1, 0.66, 1.0, 0.0]
ARM_PUT_POSE      = [0.0, -0.5, 0.3,  0.3, 0.0]

# 各桌面拾取姿態（桌高不同，夾爪高度不同，分開定義）
BALCONY_PICK_POSE    = [0.0, -0.65, 0.60, 1.0, 0.0]  # BalconyTable z≈0.278
NIGHTSTAND_PICK_POSE = [0.0, -0.60, 0.55, 0.90, 0.0]  # NightStand z≈0.369，夾爪更高
GRIP_OPEN   = [ 0.45, -0.45,  0.45,  0.45,  0.45,  0.45]
GRIP_CLOSE  = [-0.45,  0.45, -0.45, -0.45, -0.45, -0.45]
GRIP_JOINTS = ['joint6','joint7','joint8','joint9','joint10','joint11']

# ── 場景位置 ─────────────────────────────────────────────────────
COFFEETABLE_XY        = (1.512, -1.733)
COFFEETABLE_PLACE_XY  = (1.512, -1.5)
COFFEETABLE_SURFACE_Z = 0.2686

BALCONY_TABLE_XY        = (-0.556, 4.111)
BALCONY_TABLE_PLACE_XY  = (-0.556, 3.84)
BALCONY_TABLE_SURFACE_Z = 0.2780

NIGHTSTAND_XY        = (-7.726, 2.86)
NIGHTSTAND_BBOX      = (0.37, 0.24)
NIGHTSTAND_SURFACE_Z = 0.3694

NIGHTSTAND2_XY        = (-4.407, 2.86)
NIGHTSTAND2_EDGE_XY   = (-4.407, 2.65)   # 南側邊緣 (2.62) 再留 3cm
NIGHTSTAND2_SURFACE_Z = 0.36939

TRASH_XY        = (2.36, -0.796)
TRASH_SURFACE_Z = 0.143

BLUE_POSE   = (2.15,  -0.20, BLOCK_HALF)
YELLOW_POSE = (2.57,  -0.20, BLOCK_HALF)
C2_DROP_XY  = ((BLUE_POSE[0] + YELLOW_POSE[0]) / 2.0,
               (BLUE_POSE[1] + YELLOW_POSE[1]) / 2.0)

# ── 各任務初始位置 ────────────────────────────────────────────────
C1_CUP_POSE = (BALCONY_TABLE_PLACE_XY[0], BALCONY_TABLE_PLACE_XY[1], BALCONY_TABLE_SURFACE_Z)

C2_RED_Z    = NIGHTSTAND2_SURFACE_Z + BLOCK_HALF
C2_RED_POSE = (NIGHTSTAND2_EDGE_XY[0], NIGHTSTAND2_EDGE_XY[1], C2_RED_Z)

# C3 方塊放在 approach 附近地面（比 approach 稍遠離家具，手臂伸得到）
C3_RED_POSE    = (-2.005, -4.75, BLOCK_HALF)   # AirconditionerB approach(-2.005,-4.677) 前
C3_BLUE_POSE   = ( 3.296,  1.50, BLOCK_HALF)   # Ball 與 Trash 之間（離 trash 近一點）
C3_YELLOW_POSE = ( 6.60,  -3.837, BLOCK_HALF)  # KitchenCabinet approach(6.408,-3.837) 前

# C4 N3-style: 沿 SofaC_01_001 的 4m × 4m square 巡邏
SOFA_C         = "SofaC_01_001"
SOFA_C_XY      = (0.331, -1.903)
C4_SQUARE_HALF = 2.0    # 與 N3 相同（4m × 4m）
# red_block 放在第一段外側（更東更北）
# segment 1 path 在 x = 2.331，紅塊在 (3.5, -2.5)：往外 1.17m、往北 0.4m
# 機器人在 segment 1 中段過路時最近距離 ≈ 1.17m，在 DETECT_RANGE 1.5 內
C4_RED_POSE    = (3.50, -2.50, BLOCK_HALF)

HOME_POSE = (0.0, 0.0, 0.05, 0.0)

# ── 判斷閾值 ──────────────────────────────────────────────────────
PICK_Z_THRESHOLD  = 0.08
PLACE_XY_TOL      = 0.15
PLACE_Z_TOL       = 0.10
TRASH_NEAR_TOL    = 0.25
C4_DETECT_RANGE   = 2.50    # corner 偵測範圍，覆蓋外側紅塊
TABLE_POSE_TOL    = 0.10
RESET_POS_TOL     = 0.04
RESET_BLOCK_Z_TOL = 0.06
RESET_ROBOT_Z_TOL = 0.08
RESET_YAW_TOL     = 0.08
RESET_STABLE_S    = 2.0

BLOCK_MODEL_FILES = {
    RED:    os.path.join(MODEL_DIR, "red_block",   "model.sdf"),
    BLUE:   os.path.join(MODEL_DIR, "blue_block",  "model.sdf"),
    YELLOW: os.path.join(MODEL_DIR, "yellow_block","model.sdf"),
    CUP:    os.path.join(PROJECT_ROOT, "scene", "coke", "model.sdf"),
}

import ros_bridge                   # noqa: E402
from action import robot_api        # noqa: E402
from manip_capability_test import (  # noqa: E402
    ManipTester,
    load_scene,
    dist_xy,
    norm_angle,
    model_pose,
    model_pose_yaw,
    set_model_pose_once,
    set_robot_pose,
    wait_robot_home,
    wait_model_near,
)
import manip_capability_test as _cap_mod  # noqa: E402

_cap_mod.BLOCK_MODEL_FILES[CUP] = BLOCK_MODEL_FILES[CUP]


# ════════════════════════════════════════════════════════════════
# CompositeTester
# ════════════════════════════════════════════════════════════════

class CompositeTester(ManipTester):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._arm_pub2     = rospy.Publisher('/arm_controller/command',  JointTrajectory, queue_size=1)
        self._gripper_pub2 = rospy.Publisher('/hand_controller/command', JointTrajectory, queue_size=1)
        self._attach_pub   = rospy.Publisher('/block_follower/attach',   String,          queue_size=1)
        self._link_pos: dict = {}
        self._cup_attached = False
        self._cup_model    = None
        self._cup_z_offset = 0.0
        # C4 state（reset_c4 會清除）
        self.c4_completed_waypoints: list = []
        self.c4_detected_red: bool = False
        self.c4_resumed_after_drop: bool = False
        self.c4_patrol_completed: bool = False
        rospy.Subscriber('/gazebo/link_states', LinkStates, self._on_link_states, queue_size=1)
        time.sleep(0.3)

    def _on_link_states(self, msg) -> None:
        for name, pose in zip(msg.name, msg.pose):
            self._link_pos[name] = pose.position
        # 世界坐標追蹤：每個物理步把模型貼到夾爪中心（同 arm_task_server）
        if self._cup_attached and self._cup_model:
            c = self._get_gripper_center()
            if c and not rospy.is_shutdown():
                ms = ModelState()
                ms.model_name = self._cup_model
                ms.reference_frame = 'world'
                ms.pose.position.x = c[0]
                ms.pose.position.y = c[1]
                ms.pose.position.z = c[2] + self._cup_z_offset
                ms.pose.orientation.w = 1.0
                try:
                    self.model_pub.publish(ms)
                except Exception:
                    self._cup_attached = False

    def _cup_attach(self, model: str, z_offset: float) -> None:
        """開始世界坐標追蹤（不用 block_follower，換姿態不跳）。"""
        self._cup_model    = model
        self._cup_z_offset = z_offset
        self._cup_attached = True

    def _cup_detach(self) -> None:
        self._cup_attached = False
        self.detach_pub.publish(String(data="detach"))

    def _get_gripper_center(self):
        p7 = self._link_pos.get('mini_mec_six_arm::link7')
        p9 = self._link_pos.get('mini_mec_six_arm::link9')
        if p7 is None or p9 is None:
            return None
        return (
            (p7.x + p9.x) / 2.0,
            (p7.y + p9.y) / 2.0,
            (p7.z + p9.z) / 2.0 - 0.03,
        )

    def _set_arm(self, pose, secs: float = 2.0) -> None:
        traj = JointTrajectory()
        traj.joint_names = ['joint1','joint2','joint3','joint4','joint5']
        traj.header.stamp = rospy.Time.now()
        pt = JointTrajectoryPoint()
        pt.positions = pose
        pt.time_from_start = rospy.Duration(secs)
        traj.points = [pt]
        self._arm_pub2.publish(traj)
        rospy.sleep(secs + 0.2)

    def _set_gripper(self, pos, secs: float = 1.0) -> None:
        traj = JointTrajectory()
        traj.joint_names = GRIP_JOINTS
        traj.header.stamp = rospy.Time.now()
        pt = JointTrajectoryPoint()
        pt.positions = pos
        pt.time_from_start = rospy.Duration(secs)
        traj.points = [pt]
        self._gripper_pub2.publish(traj)
        rospy.sleep(secs + 0.2)

    def _clear_costmaps(self) -> bool:
        """清除 move_base 的 global + local costmap，去掉 ARM_PUT 留下的幻影障礙物。"""
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=3.0)
            rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
            print("[clear_costmaps] OK")
            return True
        except Exception as e:
            print(f"[clear_costmaps] fail: {e}")
            return False

    # ── reset ─────────────────────────────────────────────────────

    def _base_reset(self) -> None:
        self.cancel_navigation()
        self.detach()
        self.stop_base()
        for _ in range(3):
            set_robot_pose(self.set_model_state, *HOME_POSE)
            time.sleep(0.1)
        wait_robot_home(self.get_model_state)

    def _teleport_block_to_stable(self, name, pose, teleport_duration=1.2,
                                   pos_tol=RESET_POS_TOL, z_tol=RESET_BLOCK_Z_TOL):
        if not self.model_exists(name):
            self.spawn_block_model(name, pose)
            time.sleep(0.5)
        self.teleport_model(name, *pose, duration=teleport_duration)
        wait_model_near(self.get_model_state, name, pose,
                        timeout_s=8.0, pos_tol=pos_tol, z_tol=z_tol)

    def reset_c1(self) -> None:
        print(f"[reset] C1 Coke on BalconyTable at {C1_CUP_POSE}")
        self._base_reset()
        self._teleport_block_to_stable(CUP, C1_CUP_POSE, teleport_duration=1.5, z_tol=0.10)
        time.sleep(1.0)
        self.arm_home()

    def _force_respawn(self, name: str, pose) -> None:
        """強制刪除再 spawn，確保使用最新 SDF（尺寸變更後必須 respawn）。"""
        self.delete_model_if_exists(name)
        time.sleep(0.3)
        self.spawn_block_model(name, pose)
        time.sleep(0.5)

    def reset_table_pick(self, red_pose=C2_RED_POSE) -> None:
        print(f"[reset] C2 red on NightStand_01_002 at {red_pose}")
        self._base_reset()
        self._teleport_block_to_stable(RED, red_pose, teleport_duration=1.5, z_tol=0.10)
        self._teleport_block_to_stable(BLUE,   BLUE_POSE)
        self._teleport_block_to_stable(YELLOW, YELLOW_POSE)
        time.sleep(1.0)
        self.arm_home()

    def reset_c3(self) -> None:
        print(f"[reset] C3 red={C3_RED_POSE}, blue={C3_BLUE_POSE}, yellow={C3_YELLOW_POSE}")
        self._base_reset()
        self._teleport_block_to_stable(RED,    C3_RED_POSE)
        self._teleport_block_to_stable(BLUE,   C3_BLUE_POSE)
        self._teleport_block_to_stable(YELLOW, C3_YELLOW_POSE)
        time.sleep(1.0)
        self.arm_home()

    def reset_c4(self) -> None:
        # 清狀態
        self.c4_completed_waypoints = []
        self.c4_detected_red = False
        self.c4_resumed_after_drop = False
        self.c4_patrol_completed = False
        corner2 = (SOFA_C_XY[0] + C4_SQUARE_HALF, SOFA_C_XY[1] - C4_SQUARE_HALF)
        d_red_c2 = math.hypot(C4_RED_POSE[0] - corner2[0], C4_RED_POSE[1] - corner2[1])
        print(f"[reset] C4 sofa center={SOFA_C_XY}, red={C4_RED_POSE}")
        print(f"[reset] C4 red↔corner2={corner2} distance={d_red_c2:.3f}m (DETECT_RANGE={C4_DETECT_RANGE})")
        self._base_reset()   # robot → HOME_POSE
        self._teleport_block_to_stable(RED, C4_RED_POSE)
        # 不動 blue/yellow（C4 不使用）
        time.sleep(1.0)
        self.arm_home()

    # ── judge ─────────────────────────────────────────────────────

    def check_c1(self) -> bool:
        x, y, z = model_pose(self.get_model_state, CUP)
        scene = load_scene()
        info  = scene[NIGHTSTAND]
        cx = float(info["object_x"]); cy = float(info["object_y"])
        hx = float(info.get("bbox_half_x", NIGHTSTAND_BBOX[0]))
        hy = float(info.get("bbox_half_y", NIGHTSTAND_BBOX[1]))
        in_bbox = (cx-hx) <= x <= (cx+hx) and (cy-hy) <= y <= (cy+hy)
        z_ok    = z > float(info.get("surface_z", NIGHTSTAND_SURFACE_Z)) - 0.05
        print(f"[check] C1 cup=({x:.3f},{y:.3f},{z:.3f}) in_bbox={in_bbox} z_ok={z_ok}")
        return in_bbox and z_ok

    def check_c2(self) -> bool:
        return self.check_red_between_objs(BLUE, YELLOW)

    def check_c3(self) -> bool:
        ok = True
        for name in [RED, BLUE, YELLOW]:
            x, y, z = model_pose(self.get_model_state, name)
            err  = dist_xy((x, y), TRASH_XY)
            near = err <= TRASH_NEAR_TOL
            print(f"[check] C3 {name} dist_to_trash={err:.3f} near={near}")
            if not near: ok = False
        return ok

    def check_c4(self) -> bool:
        """C4: red 在 trash 附近 + 完成全部 N3 waypoints + (若有 detected → 必須 resumed)。"""
        x, y, z = model_pose(self.get_model_state, RED)
        red_to_trash = dist_xy((x, y), TRASH_XY)
        expected = {
            "C4 square corner 1", "C4 square corner 2",
            "C4 square corner 3", "C4 square corner 4",
            "C4 close square",
        }
        completed_set = set(self.c4_completed_waypoints)
        all_visited   = expected.issubset(completed_set)

        print(f"[check] C4 red=({x:.3f},{y:.3f},{z:.3f}) red_to_trash={red_to_trash:.3f}")
        print(f"[check] completed_waypoints={self.c4_completed_waypoints}")
        print(f"[check] detected_red={self.c4_detected_red}")
        print(f"[check] resumed_after_drop={self.c4_resumed_after_drop}")
        print(f"[check] patrol_completed={self.c4_patrol_completed}")

        ok = (red_to_trash <= TRASH_NEAR_TOL
              and self.c4_patrol_completed
              and all_visited)
        if self.c4_detected_red:
            ok = ok and self.c4_resumed_after_drop
        return ok

    # ── task runners ──────────────────────────────────────────────

    def run_c1(self) -> bool:
        """C1: BalconyTable 夾 Coke → NightStand_01_001。世界坐標追蹤，中継點換姿態。"""
        print("\n========== C1 pick Coke (BalconyTable) → put on NightStand ==========")
        self.reset_c1()

        print("[C1] Step1 開夾爪 + ARM_CLAMP")
        self._set_gripper(GRIP_OPEN, secs=0.5)
        self._set_arm(ARM_CLAMP_POSE, secs=1.5)

        print(f"[C1] Step2 teleport Coke → {C1_CUP_POSE}")
        self.teleport_model(CUP, *C1_CUP_POSE, duration=0.4)
        rospy.sleep(0.3)

        # ── BalconyTable 拾取段 ──────────────────────────────────────
        ax, ay, ayaw = ros_bridge.get_obj_approach_pos("BalconyTable_01_001")
        ay_mid   = ay - 0.25
        ay_final = ay - 0.05   # 比 YAML approach 稍退一點點

        print(f"[C1] Step3a nav → BalconyTable 中継 ({ax:.2f},{ay_mid:.2f})")
        ros_bridge.move_to_goal(ax, ay_mid)
        print(f"[C1] Step3b [中継] BALCONY_PICK_POSE + rotate")
        self._set_arm(BALCONY_PICK_POSE, secs=2.0)
        ros_bridge.rotate_to_face(*BALCONY_TABLE_XY)
        print(f"[C1] Step3c drive_forward 0.20m（繞開 TEB）")
        ros_bridge.drive_forward(0.20)
        ros_bridge.rotate_to_face(*BALCONY_TABLE_XY)   # 終點再確認朝向

        c = self._get_gripper_center()
        if c is None:
            rx, ry = ros_bridge.get_current_pos()
            ryaw   = ros_bridge.get_current_orientation()
            c = (rx + 0.40*math.cos(ryaw), ry + 0.40*math.sin(ryaw), 0.35)
        snap = (c[0], c[1], c[2] - CUP_SNAP_DOWN)
        print(f"[C1] Step4 snap → {snap}")
        self.teleport_model(CUP, *snap, duration=0.3)
        self._set_gripper(GRIP_CLOSE, secs=0.8)
        # 改用 block_follower（C++ plugin，物理引擎內追蹤，比 Python 平滑）
        self._attach_pub.publish(String(
            data=f"mini_mec_six_arm::link7,{CUP},{snap[0]:.4f},{snap[1]:.4f},{snap[2]:.4f}"))
        rospy.sleep(0.5)

        # Step 5: 轉向 + ARM_CLAMP + re-snap（link7 朝向變了，重算 offset）
        print(f"[C1] Step5 轉向 NightStand + ARM_CLAMP + re-snap")
        ros_bridge.rotate_to_face(*NIGHTSTAND_XY)
        self._set_arm(ARM_CLAMP_POSE, secs=2.0)
        self.detach_pub.publish(String(data="detach"))
        rospy.sleep(0.4)
        c2 = self._get_gripper_center()
        if c2 is None:
            c2 = c
        snap2 = (c2[0], c2[1], c2[2] - CUP_SNAP_DOWN)
        print(f"[C1]       re-snap → ({snap2[0]:.3f},{snap2[1]:.3f},{snap2[2]:.3f})")
        self.teleport_model(CUP, *snap2, duration=0.3)
        self._attach_pub.publish(String(
            data=f"mini_mec_six_arm::link7,{CUP},{snap2[0]:.4f},{snap2[1]:.4f},{snap2[2]:.4f}"))
        rospy.sleep(0.3)

        # ── NightStand 放置段 ────────────────────────────────────────
        scene = load_scene()
        ns = scene[NIGHTSTAND]
        ns_ax  = float(ns['approach_x'])
        ns_ay  = float(ns['approach_y'])
        ns_yaw = float(ns.get('approach_yaw', 0.0))
        ns_ox, ns_oy = float(ns['object_x']), float(ns['object_y'])
        ns_px = float(ns.get('place_x', ns_ox))
        ns_py = float(ns.get('place_y', ns_oy))
        ns_sz = float(ns.get('surface_z', NIGHTSTAND_SURFACE_Z))

        print(f"[C1] Step6a nav → NightStand 中継 ({ns_ax:.2f},{ns_ay-0.15:.2f})")
        ros_bridge.move_to_goal(ns_ax, ns_ay - 0.15)
        print(f"[C1] Step6b [中継] 轉向 + ARM_PUT")
        ros_bridge.rotate_to_face(ns_ox, ns_oy)
        self._set_arm(ARM_PUT_POSE, secs=2.0)
        print(f"[C1] Step6c drive_forward 0.15m（繞開 TEB）")
        ros_bridge.drive_forward(0.15)
        ros_bridge.rotate_to_face(ns_ox, ns_oy)        # 終點再確認朝向

        print(f"[C1] Step7 detach + drop")
        self.detach_pub.publish(String(data="detach"))
        rospy.sleep(0.4)
        self._set_gripper(GRIP_OPEN, secs=0.8)
        gc = self._get_gripper_center()
        drop_x = gc[0] if gc else ns_px
        drop_y = gc[1] if gc else ns_py
        self.teleport_model(CUP, drop_x, drop_y, ns_sz + BLOCK_HALF, duration=0.4)
        return self.check_c1()

    def run_c2(self) -> bool:
        """C2: NightStand_01_002 南側邊邊夾紅塊 → 放到藍黃中間。與 C1 相同 manual 流程。

          Step 1   開夾爪 + ARM_CLAMP
          Step 3a  nav → NightStand2 中継 (approach_x+0.25m)
          Step 3b  [中継] BALCONY_PICK_POSE + rotate to face NightStand2
          Step 3c  nav → NightStand2 最終 (approach_x)
          Step 4   snap + 夾緊 + _cup_attach（世界坐標追蹤）
          Step 5   轉向 drop 區 + ARM_CLAMP
          Step 6a  nav → 藍黃中継
          Step 6b  [中継] 轉向 + ARM_PUT
          Step 6c  nav → 藍黃最終
          Step 7   detach + 開夾爪 + drop
        """
        print("\n========== C2 pick red (NightStand_01_002) → put between blue/yellow ==========")
        self.reset_table_pick(C2_RED_POSE)

        print("[C2] Step1 開夾爪 + ARM_CLAMP")
        self._set_gripper(GRIP_OPEN, secs=0.5)
        self._set_arm(ARM_CLAMP_POSE, secs=1.5)

        # ── NightStand_01_002 拾取段（X 軸方向接近，approach_yaw≈π）────
        scene = load_scene()
        ns2   = scene[NIGHTSTAND2]
        ax    = float(ns2['approach_x'])
        ay    = float(ns2['approach_y'])
        ayaw  = float(ns2.get('approach_yaw', math.pi))
        # X 軸方向被 NightStand 前後夾死，改從南邊進
        mid_x = NIGHTSTAND2_XY[0]   # x 對齊 NightStand 中心
        mid_y = ay - 0.85           # 南邊 0.85m，drive_forward 0.2m 後距桌約 0.4m

        print(f"[C2] Step3a nav → NightStand2 中継南側 ({mid_x:.2f},{mid_y:.2f})")
        ros_bridge.move_to_goal(mid_x, mid_y)        # 中継不限朝向
        print(f"[C2] Step3b [中継] NIGHTSTAND_PICK_POSE + rotate to face NightStand2")
        self._set_arm(NIGHTSTAND_PICK_POSE, secs=2.0)
        ros_bridge.rotate_to_face(*NIGHTSTAND2_XY)   # 對準後朝北
        print(f"[C2] Step3c drive_forward 0.20m 向北（繞開 TEB）")
        ros_bridge.drive_forward(0.20)
        ros_bridge.rotate_to_face(*NIGHTSTAND2_XY)   # 終點再確認朝向

        c = self._get_gripper_center()
        if c is None:
            rx, ry = ros_bridge.get_current_pos()
            ryaw   = ros_bridge.get_current_orientation()
            c = (rx + 0.40*math.cos(ryaw), ry + 0.40*math.sin(ryaw), 0.35)
        snap = (c[0], c[1], c[2] - BLOCK_SNAP_DOWN)   # 方塊邏輯，非可樂罐
        print(f"[C2] Step4 snap → ({snap[0]:.3f},{snap[1]:.3f},{snap[2]:.3f})")
        self.teleport_model(RED, *snap, duration=0.3)
        self._set_gripper(GRIP_CLOSE, secs=0.8)
        self._attach_pub.publish(String(
            data=f"mini_mec_six_arm::link7,{RED},{snap[0]:.4f},{snap[1]:.4f},{snap[2]:.4f}"))
        rospy.sleep(0.5)

        # Step5: 轉向 + ARM_CLAMP + re-snap（ARM_CLAMP 後 link7 朝向已變，重算 offset）
        print(f"[C2] Step5 轉向 drop 區 + ARM_CLAMP + re-snap")
        ros_bridge.rotate_to_face(*C2_DROP_XY)
        self._set_arm(ARM_CLAMP_POSE, secs=2.0)
        self.detach_pub.publish(String(data="detach"))
        rospy.sleep(0.4)
        c2 = self._get_gripper_center()
        if c2 is None:
            c2 = c
        snap2 = (c2[0], c2[1], c2[2] - BLOCK_SNAP_DOWN)
        print(f"[C2]       re-snap → ({snap2[0]:.3f},{snap2[1]:.3f},{snap2[2]:.3f})")
        self.teleport_model(RED, *snap2, duration=0.3)
        self._attach_pub.publish(String(
            data=f"mini_mec_six_arm::link7,{RED},{snap2[0]:.4f},{snap2[1]:.4f},{snap2[2]:.4f}"))
        rospy.sleep(0.3)

        # ── 藍黃 drop 段（參考 put_down_between_objs：動態算接近方向）──────
        drop_x, drop_y = C2_DROP_XY
        rx, ry = ros_bridge.get_current_pos()
        dx, dy = drop_x - rx, drop_y - ry
        dist   = math.hypot(dx, dy)
        ux, uy = (dx / dist, dy / dist) if dist > 1e-3 else (1.0, 0.0)
        ayaw_drop = math.atan2(uy, ux)
        ARM_REACH = 0.40
        PRE_EXTRA = 1.2
        pre_x = drop_x - ux * (ARM_REACH + PRE_EXTRA)
        pre_y = drop_y - uy * (ARM_REACH + PRE_EXTRA)

        print(f"[C2] Step6a nav → 藍黃中継 ({pre_x:.2f},{pre_y:.2f})")
        ros_bridge.move_to_goal(pre_x, pre_y, ayaw_drop)
        print(f"[C2] Step6b [中継] 轉向 + ARM_PUT")
        ros_bridge.rotate_to_face(drop_x, drop_y)
        self._set_arm(ARM_PUT_POSE, secs=2.0)

        approach_x = drop_x - ux * ARM_REACH
        approach_y = drop_y - uy * ARM_REACH
        print(f"[C2] Step6c nav → 藍黃最終 ({approach_x:.2f},{approach_y:.2f})")
        ros_bridge.move_to_goal(approach_x, approach_y)
        ros_bridge.rotate_to_face(drop_x, drop_y)

        print(f"[C2] Step7 detach + drop ({drop_x:.3f},{drop_y:.3f})")
        self.detach_pub.publish(String(data="detach"))
        rospy.sleep(0.4)
        self._set_gripper(GRIP_OPEN, secs=0.8)
        self.teleport_model(RED, drop_x, drop_y, BLOCK_HALF, duration=0.4)
        return self.check_c2()

    def _drop_to_trash(self, block_name: str, force_side: str = None) -> None:
        """TEB 導航到中継點 → 轉向 + 換 ARM_PUT → drive_forward 走到 drop → 放下。
        force_side='N/S/E/W' 強制方位；None 則根據當前位置自動選最近方位。"""
        info = ros_bridge.get_obj_info(TRASH)
        obj_x = float(info['object_x'])
        obj_y = float(info['object_y'])
        surface_z = float(info.get('surface_z', TRASH_SURFACE_Z))

        if force_side in ('E', 'W', 'N', 'S'):
            side = force_side
            ux, uy = {'E': (1.0, 0.0), 'W': (-1.0, 0.0),
                      'N': (0.0, 1.0), 'S': (0.0, -1.0)}[side]
        else:
            # 機器人在 trash 的哪個方位（取主軸）
            rx, ry = ros_bridge.get_current_pos()
            dx, dy = rx - obj_x, ry - obj_y
            if abs(dx) > abs(dy):
                ux, uy = (1.0, 0.0) if dx > 0 else (-1.0, 0.0)
                side = 'E' if dx > 0 else 'W'
            else:
                ux, uy = (0.0, 1.0) if dy > 0 else (0.0, -1.0)
                side = 'N' if dy > 0 else 'S'
        ayaw = math.atan2(-uy, -ux)   # 朝向 trash

        MID_DIST  = 0.80
        DROP_DIST = 0.40
        mid_x = obj_x + ux * MID_DIST
        mid_y = obj_y + uy * MID_DIST
        print(f"[C3] {block_name} 從 {side} 接近 → 中継點 ({mid_x:.2f},{mid_y:.2f})  TEB nav")
        ros_bridge.move_to_goal(mid_x, mid_y, ayaw)

        # 2. 朝向 trash + 換 ARM_PUT（手臂在空曠處展開）
        print(f"[C3] 中継點：rotate to face trash + arm → ARM_PUT")
        ros_bridge.rotate_to_face(obj_x, obj_y)
        self._set_arm(ARM_PUT_POSE, secs=2.0)

        # 3. drive_forward 走到 drop 位置（不用 TEB，繞開 costmap）
        forward_d = MID_DIST - DROP_DIST
        print(f"[C3] drive_forward {forward_d:.2f}m → drop 位置（距 trash {DROP_DIST}m）")
        ros_bridge.drive_forward(forward_d)

        # 4. 放下
        print(f"[C3] detach + 開夾爪 + drop")
        self.detach_pub.publish(String(data="detach"))
        rospy.sleep(0.4)
        self._set_gripper(GRIP_OPEN, secs=0.8)
        self.teleport_model(block_name, obj_x, obj_y, surface_z + BLOCK_HALF, duration=0.4)

    def run_c3(self) -> bool:
        """C3: 地面紅/藍/黃 逐一丟到垃圾桶。
        用 _drop_to_trash（單次前進）取代 put_down_obj_by_offset（雙次前進）。
        每次丟完：倒退脫離垃圾桶 → 切回 ARM_CLAMP → 拾下一個。
        """
        print("\n========== C3 pick all ground blocks → trash ==========")
        self.reset_c3()
        blocks = [RED, BLUE, YELLOW]
        for i, name in enumerate(blocks):
            print(f"[C3] picking {name}")
            robot_api.pick_up_obj(name)
            self._drop_to_trash(name)
            if i < len(blocks) - 1:
                print(f"[C3] 倒退 0.50m 脫離垃圾桶")
                ros_bridge.drive_forward(-0.50)
                print(f"[C3] arm → ARM_CLAMP（搬運姿態）")
                self._set_arm(ARM_CLAMP_POSE, secs=1.5)
                self._clear_costmaps()   # 清掉 ARM_PUT 期間的幻影障礙物
        return self.check_c3()

    def run_c4(self) -> bool:
        """C4: 沿 N3 SofaC_01_001 4m × 4m square 巡邏。發現紅塊就暫停、撿起、丟垃圾桶，
        回到當時的 waypoint 位置與朝向後繼續完成剩下的 square（含 close square）。"""
        print("\n========== C4 N3-style square patrol around SofaC, pick red → trash ==========")
        self.reset_c4()
        sx, sy = SOFA_C_XY
        half   = C4_SQUARE_HALF
        # N3 順序（同 nav_capability_test.task_n3）
        waypoints = [
            (sx + half, sy + half, "C4 square corner 1"),
            (sx + half, sy - half, "C4 square corner 2"),
            (sx - half, sy - half, "C4 square corner 3"),
            (sx - half, sy + half, "C4 square corner 4"),
            (sx + half, sy + half, "C4 close square"),
        ]
        print(f"[C4] sofa center=({sx:.3f},{sy:.3f}) half={half}")
        print(f"[C4] waypoints: {[(w[0],w[1]) for w in waypoints]}")

        red_dropped = False
        for wx, wy, label in waypoints:
            print(f"[C4] {label} → ({wx:.2f},{wy:.2f})")
            ros_bridge.move_to_goal(wx, wy)
            self.c4_completed_waypoints.append(label)

            # 到 corner 1 後：奔向紅塊撿+丟（trash 東邊），倒退 + ARM_CLAMP，直接走 corner 2
            if label == "C4 square corner 1" and not red_dropped:
                bx, by, bz = model_pose(self.get_model_state, RED)
                if bz < PICK_Z_THRESHOLD:
                    print(f"[C4] @corner 1：奔向紅塊 ({bx:.2f},{by:.2f}) 撿+丟")
                    self.c4_detected_red = True

                    robot_api.pick_up_obj(RED)
                    self._drop_to_trash(RED, force_side='E')
                    red_dropped = True

                    print("[C4] 倒退 + ARM_CLAMP + 清 costmap，下一步直接走 corner 2")
                    self.stop_base()
                    ros_bridge.drive_forward(-0.40)
                    rospy.sleep(0.5)
                    self._set_arm(ARM_CLAMP_POSE, secs=1.5)
                    self._clear_costmaps()
                    # 不 nav 回 resume 點，直接讓 for-loop 下一個 waypoint (corner 2) 接手
                    self.c4_resumed_after_drop = True

        self.c4_patrol_completed = True
        if not red_dropped:
            print("[C4] red_block 未被偵測到（巡邏全程未進入偵測範圍）")
        return self.check_c4()


# ════════════════════════════════════════════════════════════════
# CLI
# ════════════════════════════════════════════════════════════════

TASK_RUNNERS = {
    "C1": CompositeTester.run_c1,
    "C2": CompositeTester.run_c2,
    "C3": CompositeTester.run_c3,
    "C4": CompositeTester.run_c4,
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("tasks", nargs="*", choices=sorted(TASK_RUNNERS))
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--reset-only", action="store_true")
    parser.add_argument("--pause-after-reset", type=float, default=0.0)
    args = parser.parse_args()

    tasks: List[str]
    if args.all:
        tasks = ["C1", "C2", "C3", "C4"]
    elif args.tasks:
        tasks = args.tasks
    else:
        parser.error("Use --all or specify tasks, e.g. C1 C4")

    tester = CompositeTester(pause_after_reset=args.pause_after_reset)

    if args.reset_only:
        task = tasks[0]
        {"C1": tester.reset_c1,
         "C3": tester.reset_c3,
         "C4": tester.reset_c4}.get(task, lambda: tester.reset_table_pick())()
        print("[reset-only] done")
        return 0

    results: Dict[str, bool] = {}
    for task in tasks:
        try:
            ok = TASK_RUNNERS[task](tester)
            results[task] = ok
            print(f"[{task}] {'PASS' if ok else 'FAIL'}")
        except Exception as exc:
            results[task] = False
            print(f"[{task}] FAIL: {exc}")

    print("\n===== summary =====")
    for task, ok in results.items():
        print(f"  {task}: {'PASS' if ok else 'FAIL'}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
