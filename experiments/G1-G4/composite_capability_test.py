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
  python3 G1-G4/composite_capability_test.py C1
  python3 G1-G4/composite_capability_test.py --all
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
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
SCENE_PATH    = os.path.join(PROJECT_ROOT, "gazebo_scene.yaml")
MODEL_DIR     = os.path.join(PROJECT_ROOT, "small_brain", "models")

if BIG_BRAIN_DIR not in sys.path:
    sys.path.insert(0, BIG_BRAIN_DIR)

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
CUP_SNAP_DOWN   = CUP_HALF

# ── 手臂姿態 ─────────────────────────────────────────────────────
ARM_CLAMP_POSE    = [0.0, -1.1, 0.66, 1.0, 0.0]
ARM_PUT_POSE      = [0.0, -0.5, 0.3,  0.3, 0.0]
BALCONY_PICK_POSE = [0.0, -0.65, 0.60, 1.0, 0.0]
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

C3_RED_POSE    = (-0.50,  0.50, BLOCK_HALF)
C3_BLUE_POSE   = ( 1.00,  1.00, BLOCK_HALF)
C3_YELLOW_POSE = ( 1.50,  0.50, BLOCK_HALF)

C4_SQUARE_HALF = 0.5
C4_RED_POSE    = (
    COFFEETABLE_XY[0] + C4_SQUARE_HALF,
    COFFEETABLE_XY[1] + C4_SQUARE_HALF,
    BLOCK_HALF,
)

HOME_POSE = (0.0, 0.0, 0.05, 0.0)

# ── 判斷閾值 ──────────────────────────────────────────────────────
PICK_Z_THRESHOLD  = 0.08
PLACE_XY_TOL      = 0.15
PLACE_Z_TOL       = 0.10
TRASH_NEAR_TOL    = 0.25
C4_DETECT_RANGE   = 0.90
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
        self._link_pos: dict = {}
        self._cup_attached = False
        self._cup_model    = None
        self._cup_z_offset = 0.0
        rospy.Subscriber('/gazebo/link_states', LinkStates, self._on_link_states, queue_size=1)
        time.sleep(0.3)

    def _on_link_states(self, msg) -> None:
        for name, pose in zip(msg.name, msg.pose):
            self._link_pos[name] = pose.position
        # 世界坐標追蹤：每個物理步把模型貼到夾爪中心（同 arm_task_server）
        if self._cup_attached and self._cup_model:
            c = self._get_gripper_center()
            if c:
                ms = ModelState()
                ms.model_name = self._cup_model
                ms.reference_frame = 'world'
                ms.pose.position.x = c[0]
                ms.pose.position.y = c[1]
                ms.pose.position.z = c[2] + self._cup_z_offset
                ms.pose.orientation.w = 1.0
                self.model_pub.publish(ms)

    def _cup_attach(self, model: str, z_offset: float) -> None:
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

    def reset_table_pick(self, red_pose=C2_RED_POSE) -> None:
        print(f"[reset] C2 red on NightStand_01_002 at {red_pose}")
        self._base_reset()
        self._teleport_block_to_stable(RED, red_pose, teleport_duration=1.5, z_tol=0.08)
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
        print(f"[reset] C4 red={C4_RED_POSE}")
        self._base_reset()
        self._teleport_block_to_stable(RED, C4_RED_POSE)
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
        x, y, z = model_pose(self.get_model_state, RED)
        err = dist_xy((x, y), TRASH_XY)
        print(f"[check] C4 red dist_to_trash={err:.3f}")
        return err <= TRASH_NEAR_TOL

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
        ay_final = ay

        print(f"[C1] Step3a nav → BalconyTable 中継 ({ax:.2f},{ay_mid:.2f})")
        ros_bridge.move_to_goal(ax, ay_mid)
        print(f"[C1] Step3b [中継] BALCONY_PICK_POSE + rotate")
        self._set_arm(BALCONY_PICK_POSE, secs=2.0)
        ros_bridge.rotate_to_face(*BALCONY_TABLE_XY)
        print(f"[C1] Step3c nav → BalconyTable 最終 ({ax:.2f},{ay_final:.2f})")
        ros_bridge.move_to_goal(ax, ay_final)

        c = self._get_gripper_center()
        if c is None:
            rx, ry = ros_bridge.get_current_pos()
            ryaw   = ros_bridge.get_current_orientation()
            c = (rx + 0.40*math.cos(ryaw), ry + 0.40*math.sin(ryaw), 0.35)
        snap = (c[0], c[1], c[2] - CUP_SNAP_DOWN)
        print(f"[C1] Step4 snap → ({snap[0]:.3f},{snap[1]:.3f},{snap[2]:.3f})")
        self.teleport_model(CUP, *snap, duration=0.3)
        self._set_gripper(GRIP_CLOSE, secs=0.8)
        self._cup_attach(CUP, -CUP_SNAP_DOWN)
        rospy.sleep(0.3)

        print(f"[C1] Step5 轉向 NightStand + ARM_CLAMP")
        ros_bridge.rotate_to_face(*NIGHTSTAND_XY)
        self._set_arm(ARM_CLAMP_POSE, secs=2.0)

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
        print(f"[C1] Step6c nav → NightStand 最終 ({ns_ax:.2f},{ns_ay+0.10:.2f})")
        ros_bridge.move_to_goal(ns_ax, ns_ay + 0.10)

        print(f"[C1] Step7 detach + drop")
        self._cup_detach()
        rospy.sleep(0.4)
        self._set_gripper(GRIP_OPEN, secs=0.8)
        gc = self._get_gripper_center()
        drop_x = gc[0] if gc else ns_px
        drop_y = gc[1] if gc else ns_py
        self.teleport_model(CUP, drop_x, drop_y, ns_sz + BLOCK_HALF, duration=0.4)
        return self.check_c1()

    def run_c2(self) -> bool:
        """C2: NightStand_01_002 南側邊邊夾紅塊 → 放到藍黃中間。與 C1 相同 manual 流程。"""
        print("\n========== C2 pick red (NightStand_01_002) → put between blue/yellow ==========")
        self.reset_table_pick(C2_RED_POSE)

        print("[C2] Step1 開夾爪 + ARM_CLAMP")
        self._set_gripper(GRIP_OPEN, secs=0.5)
        self._set_arm(ARM_CLAMP_POSE, secs=1.5)

        scene = load_scene()
        ns2   = scene[NIGHTSTAND2]
        ax    = float(ns2['approach_x'])
        ay    = float(ns2['approach_y'])
        ayaw  = float(ns2.get('approach_yaw', math.pi))

        print(f"[C2] Step3a nav → NightStand2 中継 ({ax+0.25:.2f},{ay:.2f})")
        ros_bridge.move_to_goal(ax + 0.25, ay)          # 中継不限朝向
        print(f"[C2] Step3b [中継] BALCONY_PICK_POSE + rotate")
        self._set_arm(BALCONY_PICK_POSE, secs=2.0)
        ros_bridge.rotate_to_face(*NIGHTSTAND2_XY)
        print(f"[C2] Step3c nav → NightStand2 最終 ({ax:.2f},{ay:.2f})")
        ros_bridge.move_to_goal(ax, ay, ayaw)            # 最終才強制 approach_yaw

        c = self._get_gripper_center()
        if c is None:
            rx, ry = ros_bridge.get_current_pos()
            ryaw   = ros_bridge.get_current_orientation()
            c = (rx + 0.40*math.cos(ryaw), ry + 0.40*math.sin(ryaw), 0.35)
        snap = (c[0], c[1], c[2] - CUP_SNAP_DOWN)
        print(f"[C2] Step4 snap → ({snap[0]:.3f},{snap[1]:.3f},{snap[2]:.3f})")
        self.teleport_model(RED, *snap, duration=0.3)
        self._set_gripper(GRIP_CLOSE, secs=0.8)
        self._cup_attach(RED, -CUP_SNAP_DOWN)
        rospy.sleep(0.3)

        print(f"[C2] Step5 轉向 drop 區 + ARM_CLAMP")
        ros_bridge.rotate_to_face(*C2_DROP_XY)
        self._set_arm(ARM_CLAMP_POSE, secs=2.0)

        drop_x, drop_y = C2_DROP_XY
        drop_yaw = math.pi / 2.0

        print(f"[C2] Step6a nav → 藍黃中継 ({drop_x:.2f},{drop_y-0.45:.2f})")
        ros_bridge.move_to_goal(drop_x, drop_y - 0.45)
        print(f"[C2] Step6b [中継] 轉向 + ARM_PUT")
        ros_bridge.rotate_to_face(drop_x, drop_y)
        self._set_arm(ARM_PUT_POSE, secs=2.0)
        print(f"[C2] Step6c nav → 藍黃最終 ({drop_x:.2f},{drop_y-0.20:.2f})")
        ros_bridge.move_to_goal(drop_x, drop_y - 0.20, drop_yaw)

        print(f"[C2] Step7 detach + drop ({drop_x:.3f},{drop_y:.3f})")
        self._cup_detach()
        rospy.sleep(0.4)
        self._set_gripper(GRIP_OPEN, secs=0.8)
        self.teleport_model(RED, drop_x, drop_y, BLOCK_HALF, duration=0.4)
        return self.check_c2()

    def run_c3(self) -> bool:
        """C3: 地面紅/藍/黃 逐一丟到垃圾桶。"""
        print("\n========== C3 pick all ground blocks → trash ==========")
        self.reset_c3()
        for name in [RED, BLUE, YELLOW]:
            print(f"[C3] picking {name}")
            robot_api.pick_up_obj(name)
            robot_api.put_down_obj_by_offset(TRASH, 0.0, 0.0)
        return self.check_c3()

    def run_c4(self) -> bool:
        """C4: 繞 CoffeeTable ±0.5m 正方形巡邏，發現紅塊就夾起丟垃圾桶。"""
        print("\n========== C4 square patrol around CoffeeTable, pick red → trash ==========")
        self.reset_c4()
        tx, ty = COFFEETABLE_XY
        half   = C4_SQUARE_HALF
        corners = [
            (tx + half, ty + half),
            (tx + half, ty - half),
            (tx - half, ty - half),
            (tx - half, ty + half),
        ]
        picked = False
        for i, (cx, cy) in enumerate(corners):
            print(f"[C4] corner {i+1}/4 → ({cx:.3f},{cy:.3f})")
            robot_api.move_to_xy(cx, cy)
            if not picked:
                rx, ry, rz = model_pose(self.get_model_state, RED)
                robot_dist  = dist_xy((rx, ry), (cx, cy))
                print(f"[C4] red_block at ({rx:.3f},{ry:.3f},{rz:.3f}) dist={robot_dist:.3f}")
                if rz < PICK_Z_THRESHOLD and robot_dist < C4_DETECT_RANGE:
                    print(f"[C4] detected! picking up")
                    robot_api.pick_up_obj(RED)
                    robot_api.put_down_obj_by_offset(TRASH, 0.0, 0.0)
                    picked = True
        if not picked:
            print("[C4] red_block not found during patrol")
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
