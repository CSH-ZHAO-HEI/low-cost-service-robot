#!/usr/bin/env python3
"""
Manual capability tests for manipulation tasks G1-G4.

This does not call an LLM. It uses the project's real robot_api/arm services:
  G1: pick up the red block when it is already within arm reach
  G2: navigate to the red block on the ground, then pick it up
  G3: put the held object on the CoffeeTable
  G4: put the held object between blue_block and yellow_block

Before each task, the robot is put back at (0, 0, yaw=0), and red_block is
deleted/re-spawned at the task start pose. Blue/yellow are left untouched.

Prerequisites:
  1. ./run_gazebo.sh
  2. ./run_rtab.sh
  3. ./run_teb.sh or ./run_teb_compare.sh
  4. rosrun small_brain_sim arm_task_server.py
  5. python3 get_scene.py

Usage:
  python3 G1-G4/manip_capability_test.py G1
  python3 G1-G4/manip_capability_test.py G4
  python3 G1-G4/manip_capability_test.py --all
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Dict, Iterable, List, Tuple

import rospy
import yaml
from actionlib_msgs.msg import GoalID
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, GetModelState, SetModelState, SpawnModel
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger


PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
SCENE_PATH = os.path.join(PROJECT_ROOT, "gazebo_scene.yaml")
MODEL_DIR = os.path.join(PROJECT_ROOT, "small_brain", "models")

RED = "red_block"
BLUE = "blue_block"
YELLOW = "yellow_block"
TABLE = "CoffeeTable_01_001"
ROBOT = "mini_mec_six_arm"
BLOCK_HALF = 0.025

HOME_POSE = (0.0, 0.0, 0.05, 0.0)
G1_RED_POSE = (0.45, -0.05, BLOCK_HALF)
G2_RED_POSE = (-2.20, -0.85, BLOCK_HALF)
BLUE_POSE = (2.15, -0.20, BLOCK_HALF)
YELLOW_POSE = (2.57, -0.20, BLOCK_HALF)
BLOCK_MODEL_FILES = {
    RED: os.path.join(MODEL_DIR, "red_block", "model.sdf"),
    BLUE: os.path.join(MODEL_DIR, "blue_block", "model.sdf"),
    YELLOW: os.path.join(MODEL_DIR, "yellow_block", "model.sdf"),
}

PICK_Z_THRESHOLD = 0.08
PLACE_XY_TOL = 0.12
PLACE_Z_TOL = 0.08
BETWEEN_Y_TOL = 0.08
RESET_POS_TOL = 0.04
RESET_BLOCK_Z_TOL = 0.04
RESET_ROBOT_Z_TOL = 0.08
RESET_YAW_TOL = 0.08
RESET_STABLE_S = 2.0

if BIG_BRAIN_DIR not in sys.path:
    sys.path.insert(0, BIG_BRAIN_DIR)

import ros_bridge  # noqa: E402
from action import robot_api  # noqa: E402


def load_scene() -> Dict[str, dict]:
    with open(SCENE_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def norm_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def model_pose(get_model_state, name: str) -> Tuple[float, float, float]:
    resp = get_model_state(name, "")
    if not resp.success:
        raise RuntimeError(f"Gazebo model '{name}' not found: {resp.status_message}")
    p = resp.pose.position
    return (p.x, p.y, p.z)


def model_pose_yaw(get_model_state, name: str) -> Tuple[float, float, float, float]:
    resp = get_model_state(name, "")
    if not resp.success:
        raise RuntimeError(f"Gazebo model '{name}' not found: {resp.status_message}")
    p = resp.pose.position
    q = resp.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return (p.x, p.y, p.z, yaw)


def set_model_pose_once(set_model_state, name: str, x: float, y: float, z: float) -> None:
    msg = ModelState()
    msg.model_name = name
    msg.reference_frame = "world"
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = float(z)
    msg.pose.orientation.w = 1.0
    msg.twist.linear.x = 0.0
    msg.twist.linear.y = 0.0
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = 0.0
    resp = set_model_state(msg)
    if not resp.success:
        raise RuntimeError(f"SetModelState failed for {name}: {resp.status_message}")


def set_model_pose(set_model_state, get_model_state, name: str, x: float, y: float, z: float) -> None:
    target = (float(x), float(y), float(z))
    last_pose = None
    for _ in range(10):
        set_model_pose_once(set_model_state, name, *target)
        time.sleep(0.05)
        try:
            last_pose = model_pose(get_model_state, name)
            if dist_xy((last_pose[0], last_pose[1]), (target[0], target[1])) <= 0.03 and abs(last_pose[2] - target[2]) <= 0.04:
                return
        except Exception:
            pass
    raise RuntimeError(f"Failed to reset {name} to {target}, last_pose={last_pose}")


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def set_robot_pose(set_model_state, x: float, y: float, z: float, yaw: float) -> None:
    msg = ModelState()
    msg.model_name = ROBOT
    msg.reference_frame = "world"
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = float(z)
    qx, qy, qz, qw = yaw_to_quat(yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    msg.twist.linear.x = 0.0
    msg.twist.linear.y = 0.0
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = 0.0
    resp = set_model_state(msg)
    if not resp.success:
        raise RuntimeError(f"SetModelState failed for {ROBOT}: {resp.status_message}")


def wait_model_near(
    get_model_state,
    name: str,
    target: Tuple[float, float, float],
    timeout_s: float = 8.0,
    pos_tol: float = RESET_POS_TOL,
    z_tol: float = RESET_BLOCK_Z_TOL,
) -> Tuple[float, float, float]:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        x, y, z = model_pose(get_model_state, name)
        if dist_xy((x, y), (target[0], target[1])) <= pos_tol and abs(z - target[2]) <= z_tol:
            return (x, y, z)
        time.sleep(0.1)
    x, y, z = model_pose(get_model_state, name)
    raise RuntimeError(
        f"{name} did not reset near {target}; current=({x:.3f}, {y:.3f}, {z:.3f})"
    )


def wait_robot_home(get_model_state, timeout_s: float = 8.0) -> Tuple[float, float, float, float]:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        gx, gy, gz, gyaw = model_pose_yaw(get_model_state, ROBOT)
        ox, oy = ros_bridge.get_current_pos()
        oyaw = ros_bridge.get_current_orientation()
        gazebo_ok = (
            dist_xy((gx, gy), (HOME_POSE[0], HOME_POSE[1])) <= RESET_POS_TOL
            and abs(gz - HOME_POSE[2]) <= RESET_ROBOT_Z_TOL
            and abs(norm_angle(gyaw - HOME_POSE[3])) <= RESET_YAW_TOL
        )
        odom_ok = (
            dist_xy((ox, oy), (HOME_POSE[0], HOME_POSE[1])) <= RESET_POS_TOL
            and abs(norm_angle(oyaw - HOME_POSE[3])) <= RESET_YAW_TOL
        )
        if gazebo_ok and odom_ok:
            return (gx, gy, gz, gyaw)
        time.sleep(0.1)
    gx, gy, gz, gyaw = model_pose_yaw(get_model_state, ROBOT)
    ox, oy = ros_bridge.get_current_pos()
    oyaw = ros_bridge.get_current_orientation()
    raise RuntimeError(
        "robot did not reset home; "
        f"gazebo=({gx:.3f}, {gy:.3f}, {gz:.3f}, yaw={gyaw:.3f}), "
        f"odom=({ox:.3f}, {oy:.3f}, yaw={oyaw:.3f})"
    )


def wait_reset_stable(
    get_model_state,
    red_pose: Tuple[float, float, float],
    stable_s: float = RESET_STABLE_S,
    timeout_s: float = 10.0,
) -> Dict[str, Tuple[float, ...]]:
    targets = {RED: red_pose}
    stable_since = None
    deadline = time.time() + timeout_s
    last = {}
    while time.time() < deadline:
        robot = model_pose_yaw(get_model_state, ROBOT)
        blocks = {name: model_pose(get_model_state, name) for name in targets}
        robot_ok = (
            dist_xy((robot[0], robot[1]), (HOME_POSE[0], HOME_POSE[1])) <= RESET_POS_TOL
            and abs(robot[2] - HOME_POSE[2]) <= RESET_ROBOT_Z_TOL
            and abs(norm_angle(robot[3] - HOME_POSE[3])) <= RESET_YAW_TOL
        )
        odom_x, odom_y = ros_bridge.get_current_pos()
        odom_yaw = ros_bridge.get_current_orientation()
        odom_ok = (
            dist_xy((odom_x, odom_y), (HOME_POSE[0], HOME_POSE[1])) <= RESET_POS_TOL
            and abs(norm_angle(odom_yaw - HOME_POSE[3])) <= RESET_YAW_TOL
        )
        blocks_ok = all(
            dist_xy((pose[0], pose[1]), (target[0], target[1])) <= RESET_POS_TOL
            and abs(pose[2] - target[2]) <= RESET_BLOCK_Z_TOL
            for name, target in targets.items()
            for pose in [blocks[name]]
        )
        last = {ROBOT: robot, **blocks}
        if robot_ok and odom_ok and blocks_ok:
            if stable_since is None:
                stable_since = time.time()
            if time.time() - stable_since >= stable_s:
                return last
        else:
            stable_since = None
        time.sleep(0.1)
    pretty = ", ".join(f"{name}={tuple(round(v, 3) for v in pose)}" for name, pose in last.items())
    odom_x, odom_y = ros_bridge.get_current_pos()
    odom_yaw = ros_bridge.get_current_orientation()
    raise RuntimeError(
        f"reset did not stay stable for {stable_s:.1f}s; last {pretty}, "
        f"odom=({odom_x:.3f}, {odom_y:.3f}, yaw={odom_yaw:.3f})"
    )


class ManipTester:
    def __init__(self, pause_after_reset: float = 0.0) -> None:
        self.pause_after_reset = pause_after_reset
        ros_bridge.init()
        rospy.wait_for_service("/gazebo/get_model_state", timeout=10.0)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
        rospy.wait_for_service("/gazebo/delete_model", timeout=10.0)
        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=10.0)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.delete_model_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_model_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.detach_pub = rospy.Publisher("/block_follower/detach", String, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.model_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        time.sleep(0.5)  # 等 publisher 連上 Gazebo

        # This is a small-brain capability test. Do not call JudgeLLM/VLM.
        robot_api.judge_llm.judge = lambda *args, **kwargs: True

    def detach(self) -> None:
        deadline = time.time() + 2.0
        while self.detach_pub.get_num_connections() == 0 and time.time() < deadline:
            time.sleep(0.05)
        for _ in range(3):
            self.detach_pub.publish(String(data="detach"))
            time.sleep(0.1)
        time.sleep(0.2)

    def stop_base(self) -> None:
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)

    def cancel_navigation(self) -> None:
        try:
            if getattr(ros_bridge, "_move_client", None) is not None:
                ros_bridge._move_client.cancel_all_goals()
        except Exception as exc:
            print(f"[warn] action goal cancel failed: {exc}")
        cancel = GoalID()
        cancel.stamp = rospy.Time(0)
        cancel.id = ""
        for _ in range(5):
            self.cancel_pub.publish(cancel)
            self.cmd_pub.publish(Twist())
            time.sleep(0.1)

    def model_exists(self, name: str) -> bool:
        try:
            return bool(self.get_model_state(name, "").success)
        except Exception:
            return False

    def delete_model_if_exists(self, name: str) -> None:
        if not self.model_exists(name):
            return
        for _ in range(3):
            resp = self.delete_model_srv(name)
            time.sleep(0.1)
            if resp.success or not self.model_exists(name):
                return
        if self.model_exists(name):
            raise RuntimeError(f"failed to delete stale model {name}")

    def spawn_block_model(self, name: str, pose_xyz: Tuple[float, float, float]) -> None:
        model_file = BLOCK_MODEL_FILES[name]
        if not os.path.exists(model_file):
            raise RuntimeError(f"missing model file: {model_file}")
        with open(model_file, "r", encoding="utf-8") as f:
            sdf_xml = f.read()
        pose = Pose()
        pose.position.x = float(pose_xyz[0])
        pose.position.y = float(pose_xyz[1])
        pose.position.z = float(pose_xyz[2])
        pose.orientation.w = 1.0
        resp = self.spawn_model_srv(name, sdf_xml, "", pose, "world")
        if not resp.success:
            raise RuntimeError(f"spawn {name} failed: {resp.status_message}")

    def recreate_red_model(self, red_pose: Tuple[float, float, float]) -> None:
        self.delete_model_if_exists(RED)
        self.spawn_block_model(RED, red_pose)

    def set_initial_scene_poses(self, red_pose: Tuple[float, float, float]) -> None:
        for _ in range(3):
            set_robot_pose(self.set_model_state, *HOME_POSE)
            set_model_pose_once(self.set_model_state, RED, *red_pose)
            time.sleep(0.05)

    def reset_robot_and_red(self, red_pose: Tuple[float, float, float]) -> None:
        self.cancel_navigation()
        self.stop_base()
        self.recreate_red_model(red_pose)
        self.set_initial_scene_poses(red_pose)
        self.cancel_navigation()
        self.stop_base()
        wait_robot_home(self.get_model_state)
        wait_model_near(self.get_model_state, RED, red_pose)

    def arm_home(self) -> None:
        try:
            rospy.wait_for_service("/arm/home", timeout=3.0)
            home = rospy.ServiceProxy("/arm/home", Trigger)
            resp = home()
            if not resp.success:
                print(f"[warn] /arm/home failed: {resp.message}")
        except Exception as exc:
            print(f"[warn] /arm/home unavailable: {exc}")

    def teleport_model(self, name: str, x: float, y: float, z: float,
                       duration: float = 0.5) -> None:
        """與 arm_task_server 的 teleport 同機制：高頻 publish 到 topic，
        對 static=true 模型有效（service 只打一次無法覆蓋 Gazebo 內部狀態）。"""
        msg = ModelState()
        msg.model_name = name
        msg.reference_frame = "world"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        t0 = time.time()
        while time.time() - t0 < duration and not rospy.is_shutdown():
            self.model_pub.publish(msg)
            time.sleep(0.005)  # 200Hz，與 arm_task_server teleport 同頻率

    def reset_scene(
        self,
        red_pose: Tuple[float, float, float] = G2_RED_POSE,
        pause_after_reset: float = None,
    ) -> None:
        if pause_after_reset is None:
            pause_after_reset = self.pause_after_reset
        print(
            f"[reset] robot={HOME_POSE[:2]}, red={red_pose}, "
            "blue/yellow unchanged"
        )
        self.cancel_navigation()
        self.detach()
        self.stop_base()

        # 1/3 車歸位
        print("[reset] 1/3 robot → home")
        for _ in range(3):
            set_robot_pose(self.set_model_state, *HOME_POSE)
            time.sleep(0.1)
        wait_robot_home(self.get_model_state)

        # 2/3 方塊歸位：高頻 topic publish（與 pick 時 teleport 同機制）
        print("[reset] 2/3 teleport red_block → target")
        if not self.model_exists(RED):
            print("[reset]   red_block not found, spawning first")
            self.spawn_block_model(RED, red_pose)
            time.sleep(0.5)
        self.teleport_model(RED, *red_pose, duration=0.5)
        red_actual = wait_model_near(self.get_model_state, RED, red_pose)
        time.sleep(1.0)
        stable = wait_reset_stable(self.get_model_state, red_pose)
        robot_pose = stable[ROBOT]
        red_actual = stable[RED]
        print(
            "[reset] confirmed "
            f"robot=({robot_pose[0]:.3f},{robot_pose[1]:.3f},yaw={robot_pose[3]:.3f}), "
            f"red=({red_actual[0]:.3f},{red_actual[1]:.3f},{red_actual[2]:.3f})"
        )

        # 3/3 手臂歸位
        print("[reset] 3/3 arm → ARM_CLAMP")
        self.arm_home()

        if pause_after_reset > 0:
            print(f"[reset] pause {pause_after_reset:.1f}s for visual check")
            time.sleep(pause_after_reset)

    def call_arm_pick_only(self, target_name: str) -> None:
        x, y, z = model_pose(self.get_model_state, target_name)
        rospy.set_param("/arm_task/target_name", target_name)
        rospy.set_param("/arm_task/source_z", float(max(0.0, z)))
        rospy.wait_for_service("/arm/pick", timeout=5.0)
        pick = rospy.ServiceProxy("/arm/pick", Trigger)
        resp = pick()
        if not resp.success:
            raise RuntimeError(f"/arm/pick failed: {resp.message}")

    def ensure_red_held(self) -> None:
        x, y, z = model_pose(self.get_model_state, RED)
        if z > PICK_Z_THRESHOLD:
            print(f"[setup] red_block already appears held/high: ({x:.3f}, {y:.3f}, {z:.3f})")
            return
        print("[setup] picking red_block for held-object task")
        robot_api.pick_up_obj(RED)
        if not self.check_picked():
            raise RuntimeError("setup pick failed: red_block did not move above ground")

    def check_picked(self) -> bool:
        x, y, z = model_pose(self.get_model_state, RED)
        print(f"[check] red_block pose=({x:.3f}, {y:.3f}, {z:.3f})")
        return z > PICK_Z_THRESHOLD

    def check_red_near(self, target_x: float, target_y: float, target_z: float, label: str) -> bool:
        x, y, z = model_pose(self.get_model_state, RED)
        xy_error = dist_xy((x, y), (target_x, target_y))
        z_error = abs(z - target_z)
        print(
            f"[check] {label}: red=({x:.3f}, {y:.3f}, {z:.3f}), "
            f"target=({target_x:.3f}, {target_y:.3f}, {target_z:.3f}), "
            f"xy_error={xy_error:.3f}, z_error={z_error:.3f}"
        )
        return xy_error <= PLACE_XY_TOL and z_error <= PLACE_Z_TOL

    def check_red_on_table(self, table_name: str = TABLE) -> bool:
        scene = load_scene()
        info = scene[table_name]
        x, y, z = model_pose(self.get_model_state, RED)
        cx = float(info["object_x"])
        cy = float(info["object_y"])
        hx = float(info.get("bbox_half_x", 0.0))
        hy = float(info.get("bbox_half_y", 0.0))
        target_z = float(info.get("surface_z", 0.0)) + BLOCK_HALF
        in_bbox = (cx - hx) <= x <= (cx + hx) and (cy - hy) <= y <= (cy + hy)
        z_error = abs(z - target_z)
        print(
            f"[check] {table_name} surface bbox: red=({x:.3f}, {y:.3f}, {z:.3f}), "
            f"bbox_x=({cx-hx:.3f},{cx+hx:.3f}), bbox_y=({cy-hy:.3f},{cy+hy:.3f}), "
            f"z_error={z_error:.3f}"
        )
        return in_bbox and z_error <= PLACE_Z_TOL

    def check_red_between_objs(self, obj_a: str, obj_b: str) -> bool:
        rx, ry, rz = model_pose(self.get_model_state, RED)
        ax, ay, az = model_pose(self.get_model_state, obj_a)
        bx, by, bz = model_pose(self.get_model_state, obj_b)
        vx, vy = bx - ax, by - ay
        wx, wy = rx - ax, ry - ay
        seg_len2 = vx * vx + vy * vy
        t = ((wx * vx + wy * vy) / seg_len2) if seg_len2 > 1e-9 else 0.0
        t_clamped = min(1.0, max(0.0, t))
        proj_x = ax + t_clamped * vx
        proj_y = ay + t_clamped * vy
        lateral_error = dist_xy((rx, ry), (proj_x, proj_y))
        target_z = max(0.0, ((az - BLOCK_HALF) + (bz - BLOCK_HALF)) / 2.0) + BLOCK_HALF
        z_error = abs(rz - target_z)
        print(
            f"[check] between {obj_a}/{obj_b}: red=({rx:.3f}, {ry:.3f}, {rz:.3f}), "
            f"segment_t={t:.3f}, lateral_error={lateral_error:.3f}, z_error={z_error:.3f}"
        )
        return 0.0 <= t <= 1.0 and lateral_error <= BETWEEN_Y_TOL and z_error <= PLACE_Z_TOL

    def run_g1(self) -> bool:
        print("\n========== G1 pick up red block within arm reach ==========")
        self.reset_scene(red_pose=G1_RED_POSE)
        self.call_arm_pick_only(RED)
        return self.check_picked()

    def run_g2(self) -> bool:
        print("\n========== G2 navigate to ground red block and pick it up ==========")
        self.reset_scene(red_pose=G2_RED_POSE)
        robot_api.pick_up_obj(RED)
        return self.check_picked()

    def run_g3(self) -> bool:
        print("\n========== G3 put held object on table ==========")
        self.reset_scene(red_pose=G2_RED_POSE)
        self.ensure_red_held()
        robot_api.put_down_obj_by_offset(TABLE, 0.0, 0.0)
        scene = load_scene()
        info = scene[TABLE]
        target_x = float(info.get("place_x", info["object_x"]))
        target_y = float(info.get("place_y", info["object_y"]))
        target_z = float(info.get("surface_z", 0.0)) + BLOCK_HALF
        return self.check_red_on_table(TABLE)

    def run_g4(self) -> bool:
        print("\n========== G4 put held object between blue/yellow ==========")
        self.reset_scene(red_pose=G2_RED_POSE)
        self.ensure_red_held()
        robot_api.put_down_between_objs(BLUE, YELLOW)
        bx, by, bz = model_pose(self.get_model_state, BLUE)
        yx, yy, yz = model_pose(self.get_model_state, YELLOW)
        mid_x = (bx + yx) / 2.0
        mid_y = (by + yy) / 2.0
        target_z = max(0.0, ((bz - BLOCK_HALF) + (yz - BLOCK_HALF)) / 2.0) + BLOCK_HALF
        return self.check_red_between_objs(BLUE, YELLOW)


TASK_RUNNERS = {
    "G1": ManipTester.run_g1,
    "G2": ManipTester.run_g2,
    "G3": ManipTester.run_g3,
    "G4": ManipTester.run_g4,
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("tasks", nargs="*", choices=sorted(TASK_RUNNERS))
    parser.add_argument("--all", action="store_true", help="Run G1 G2 G3 G4.")
    parser.add_argument(
        "--reset-only",
        action="store_true",
        help="Only reset the scene for the selected task, then exit.",
    )
    parser.add_argument(
        "--pause-after-reset",
        type=float,
        default=0.0,
        help="Pause after reset confirmation so the Gazebo pose can be checked visually.",
    )
    args = parser.parse_args()

    tasks: List[str]
    if args.all:
        tasks = ["G1", "G2", "G3", "G4"]
    elif args.tasks:
        tasks = args.tasks
    else:
        parser.error("Use --all or specify tasks, e.g. G1 G4")

    tester = ManipTester(pause_after_reset=args.pause_after_reset)
    if args.reset_only:
        red_pose = G1_RED_POSE if tasks[0] == "G1" else G2_RED_POSE
        tester.reset_scene(red_pose=red_pose, pause_after_reset=args.pause_after_reset)
        print("[reset-only] done")
        return 0

    results = {}
    for task in tasks:
        try:
            ok = TASK_RUNNERS[task](tester)
            results[task] = ok
            print(f"[{task}] {'PASS' if ok else 'FAIL'}")
        except Exception as exc:
            results[task] = False
            print(f"[{task}] FAIL, reason={exc}")

    print("\n========== summary ==========")
    for task, ok in results.items():
        print(f"{task}: {'PASS' if ok else 'FAIL'}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
