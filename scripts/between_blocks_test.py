#!/usr/bin/env python3
"""
between_blocks_test.py - small-brain tests for G4/C2 "put between blue/yellow".

Modes:
  g4   : place blue/yellow near the left side of Trash_01_001, make red_block held,
         then put red_block between them.
  c2   : place blue/yellow near CoffeeTable, run the tuned CoffeeTable pick-only
         flow for red_block, then put red_block between them.

Usage:
  python3 between_blocks_test.py g4
  python3 between_blocks_test.py g4 --spawn-only
  python3 between_blocks_test.py c2
  python3 between_blocks_test.py g4 --assume-held
  python3 between_blocks_test.py c2 --dry-run
"""
import argparse
import math
import os
import subprocess
import sys
import time

import actionlib
import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import LinkStates, ModelState
from gazebo_msgs.srv import GetModelState, SpawnModel
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
SCENE_YAML = os.environ.get("SCENE_PATH", os.path.join(PROJECT_ROOT, "gazebo_scene.yaml"))
MODEL_DIR = os.path.join(PROJECT_ROOT, "small_brain", "models")

RED = "red_block"
BLUE = "blue_block"
YELLOW = "yellow_block"
BLOCK_HALF = 0.025
ARM_REACH_M = 0.40
BLOCK_MODEL_FILES = {
    BLUE: MODEL_DIR + "/blue_block/model.sdf",
    YELLOW: MODEL_DIR + "/yellow_block/model.sdf",
}

ARM_CLAMP = [0.0, -1.1, 0.66, 1.0, 0.0]
ARM_PUT = [0.0, -0.5, 0.3, 0.3, 0.0]
GRIP_JOINTS = ["joint6", "joint7", "joint8", "joint9", "joint10", "joint11"]
GRIP_OPEN = [0.45, -0.45, 0.45, 0.45, 0.45, 0.45]
GRIP_CLOSE = [-0.45, 0.45, -0.45, -0.45, -0.45, -0.45]
GRIP_Z_OFFSET = -0.03

GROUND_BLUE_OFFSET = (-0.21, 0.596)   # left of Trash_01_001, away from bin approach path
GROUND_YELLOW_OFFSET = (0.21, 0.596)
TABLE_BLUE_OFFSET = (-0.11, 0.0)      # CoffeeTable middle line
TABLE_YELLOW_OFFSET = (0.11, 0.0)
TABLE_RED_OFFSET = (0.28, 0.0)        # red starts on the right side of CoffeeTable
COFFEE_PICK_POSE = [0.0, -0.35, 0.95, 1.3, 0.0]
COFFEE_BUFFER = 0.30

_odom_xy = (0.0, 0.0)
_odom_yaw = 0.0
_link_positions = {}
_cmd_pub = None
_model_pub = None
_arm_pub = None
_gripper_pub = None
_attach_pub = None
_detach_pub = None
_mb_client = None
_get_model_state = None


def odom_cb(msg):
    global _odom_xy, _odom_yaw
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    _odom_xy = (p.x, p.y)
    _odom_yaw = math.atan2(siny, cosy)


def link_states_cb(msg):
    for name, pose in zip(msg.name, msg.pose):
        _link_positions[name] = pose.position


def get_gripper_center():
    p7 = _link_positions.get("mini_mec_six_arm::link7")
    p9 = _link_positions.get("mini_mec_six_arm::link9")
    if p7 is None or p9 is None:
        return None
    return (
        (p7.x + p9.x) / 2.0,
        (p7.y + p9.y) / 2.0,
        (p7.z + p9.z) / 2.0 + GRIP_Z_OFFSET,
    )


def load_scene():
    with open(SCENE_YAML, "r") as f:
        return yaml.safe_load(f) or {}


def save_scene(scene):
    with open(SCENE_YAML, "w") as f:
        yaml.safe_dump(scene, f, sort_keys=True)


def block_scene_entry(x, y, z):
    return {
        "object_x": round(float(x), 3),
        "object_y": round(float(y), 3),
        "approach_x": round(float(x) - ARM_REACH_M, 3),
        "approach_y": round(float(y), 3),
        "approach_yaw": 0.0,
        "place_x": round(float(x), 3),
        "place_y": round(float(y), 3),
        "surface_z": round(max(0.0, float(z) - BLOCK_HALF), 3),
        "bbox_half_x": BLOCK_HALF,
        "bbox_half_y": BLOCK_HALF,
        "size_w": 0.05,
        "size_h": 0.05,
    }


def update_scene_block(model_name, x, y, z):
    scene = load_scene()
    scene[model_name] = block_scene_entry(x, y, z)
    save_scene(scene)


def model_exists(model_name):
    try:
        return bool(_get_model_state(model_name, "").success)
    except Exception:
        return False


def ensure_model(model_name, x, y, z):
    if model_exists(model_name):
        return
    model_file = BLOCK_MODEL_FILES[model_name]
    if not os.path.exists(model_file):
        raise RuntimeError(f"missing model file: {model_file}")
    with open(model_file, "r") as f:
        sdf_xml = f.read()
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=5.0)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    pose = ModelState().pose
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    resp = spawn_model(model_name, sdf_xml, "", pose, "world")
    if not resp.success:
        raise RuntimeError(f"spawn {model_name} failed: {resp.status_message}")
    print(f"    spawned {model_name}")


def yaw_to_goal_quat(goal, yaw):
    goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)


def set_arm(pose, secs=1.5):
    traj = JointTrajectory()
    traj.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pose
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    _arm_pub.publish(traj)
    rospy.sleep(secs + 0.2)


def set_gripper(pos, secs=0.7):
    traj = JointTrajectory()
    traj.joint_names = GRIP_JOINTS
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pos
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    _gripper_pub.publish(traj)
    rospy.sleep(secs + 0.2)


def teleport(model_name, x, y, z, duration=0.3):
    msg = ModelState()
    msg.model_name = model_name
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    msg.reference_frame = "world"
    t0 = time.time()
    while time.time() - t0 < duration and not rospy.is_shutdown():
        _model_pub.publish(msg)
        rospy.sleep(0.02)
    print(f"    -> {model_name}: ({x:.3f}, {y:.3f}, {z:.3f})")


def navigate_to(x, y, yaw, timeout=120):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    yaw_to_goal_quat(goal, yaw)
    print(f"    -> move_base ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.0f} deg)")

    _mb_client.cancel_all_goals()
    rospy.sleep(0.1)
    try:
        _mb_client.stop_tracking_goal()
    except Exception:
        pass
    _mb_client.send_goal(goal)
    if not _mb_client.wait_for_result(rospy.Duration(timeout)):
        _mb_client.cancel_goal()
        return False
    ok = _mb_client.get_state() == GoalStatus.SUCCEEDED
    try:
        _mb_client.stop_tracking_goal()
    except Exception:
        pass
    print("    reached" if ok else f"    failed state={_mb_client.get_state()}")
    return ok


def rotate_to_face(target_x, target_y, timeout=10.0, tol=0.03):
    rate = rospy.Rate(20)
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - t0 > timeout:
            _cmd_pub.publish(Twist())
            return False
        rx, ry = _odom_xy
        target_yaw = math.atan2(target_y - ry, target_x - rx)
        err = math.atan2(math.sin(target_yaw - _odom_yaw), math.cos(target_yaw - _odom_yaw))
        if abs(err) < tol:
            _cmd_pub.publish(Twist())
            print(f"    aligned yaw={math.degrees(_odom_yaw):.0f} deg")
            return True
        max_omega = 1.2 if abs(err) > 0.15 else max(0.3, abs(err) * 4.0)
        cmd = Twist()
        cmd.angular.z = max(-max_omega, min(max_omega, err * 1.5))
        _cmd_pub.publish(cmd)
        rate.sleep()
    _cmd_pub.publish(Twist())
    return False


def compute_approach(x, y):
    rx, ry = _odom_xy
    dx, dy = x - rx, y - ry
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        return x - ARM_REACH_M, y, 0.0
    ux, uy = dx / dist, dy / dist
    return x - ARM_REACH_M * ux, y - ARM_REACH_M * uy, math.atan2(uy, ux)


def prepare_red_held():
    print("\n[hold] make red_block held by gripper")
    set_gripper(GRIP_OPEN, secs=0.5)
    set_arm(ARM_CLAMP, secs=1.5)
    rospy.sleep(0.5)
    c = get_gripper_center()
    if c is None:
        raise RuntimeError("no gripper link state yet")
    teleport(RED, c[0], c[1], c[2], duration=0.3)
    set_gripper(GRIP_CLOSE, secs=0.7)
    msg = f"mini_mec_six_arm::link5,{RED},{c[0]:.4f},{c[1]:.4f},{c[2]:.4f}"
    _attach_pub.publish(String(data=msg))
    print(f"    -> attach {msg}")
    rospy.sleep(0.5)


def drive_forward(distance, speed=0.12, timeout=5.0):
    if abs(distance) < 1e-3:
        return
    x0, y0 = _odom_xy
    rate = rospy.Rate(20)
    t0 = rospy.Time.now().to_sec()
    cmd = Twist()
    cmd.linear.x = speed if distance > 0 else -speed
    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - t0 > timeout:
            break
        x, y = _odom_xy
        if math.hypot(x - x0, y - y0) >= abs(distance):
            break
        _cmd_pub.publish(cmd)
        rate.sleep()
    _cmd_pub.publish(Twist())


def plan_pair(mode):
    scene = load_scene()
    if mode == "g4":
        base = scene["Trash_01_001"]
        surface_z = 0.0
        bx = base["object_x"] + GROUND_BLUE_OFFSET[0]
        by = base["object_y"] + GROUND_BLUE_OFFSET[1]
        yx = base["object_x"] + GROUND_YELLOW_OFFSET[0]
        yy = base["object_y"] + GROUND_YELLOW_OFFSET[1]
        label = "Trash_01_001 left side"
    else:
        base = scene["CoffeeTable_01_001"]
        surface_z = float(base["surface_z"])
        bx = base["place_x"] + TABLE_BLUE_OFFSET[0]
        by = base["place_y"] + TABLE_BLUE_OFFSET[1]
        yx = base["place_x"] + TABLE_YELLOW_OFFSET[0]
        yy = base["place_y"] + TABLE_YELLOW_OFFSET[1]
        label = "CoffeeTable edge"

    mid_x = (bx + yx) / 2.0
    mid_y = (by + yy) / 2.0
    return label, surface_z, bx, by, yx, yy, mid_x, mid_y


def plan_c2_red():
    scene = load_scene()
    base = scene["CoffeeTable_01_001"]
    surface_z = float(base["surface_z"])
    red_x = base["place_x"] + TABLE_RED_OFFSET[0]
    red_y = base["place_y"] + TABLE_RED_OFFSET[1]
    return red_x, red_y, surface_z


def spawn_pair(mode):
    label, surface_z, bx, by, yx, yy, mid_x, mid_y = plan_pair(mode)
    z = surface_z + BLOCK_HALF
    print(f"\n[spawn] blue/yellow near {label}, surface_z={surface_z:.3f}")
    ensure_model(BLUE, bx, by, z)
    ensure_model(YELLOW, yx, yy, z)
    teleport(BLUE, bx, by, z, duration=0.35)
    teleport(YELLOW, yx, yy, z, duration=0.35)
    update_scene_block(BLUE, bx, by, z)
    update_scene_block(YELLOW, yx, yy, z)
    print(f"    midpoint: ({mid_x:.3f}, {mid_y:.3f}, {z:.3f})")
    return mid_x, mid_y, surface_z


def run_c2_table_pick():
    scene = load_scene()
    table = scene["CoffeeTable_01_001"]
    red_x, red_y, surface_z = plan_c2_red()
    red_z = surface_z + BLOCK_HALF
    bbox_half_y = float(table.get("bbox_half_y", 0.334))
    approach_x = red_x
    approach_y = float(table["object_y"]) + bbox_half_y + COFFEE_BUFFER
    approach_yaw = -math.pi / 2.0

    print("\n[pick] C2 CoffeeTable right-side red_block")
    print(f"    red starts at ({red_x:.3f}, {red_y:.3f}, {red_z:.3f})")
    teleport(RED, red_x, red_y, red_z, duration=0.35)

    set_gripper(GRIP_OPEN, secs=0.5)
    set_arm(ARM_CLAMP, secs=1.5)
    if not navigate_to(approach_x, approach_y, approach_yaw):
        raise RuntimeError("CoffeeTable pick navigation failed")
    drive_forward(0.08)

    print(f"    -> arm CoffeeTable pick pose {COFFEE_PICK_POSE}")
    set_arm(COFFEE_PICK_POSE, secs=2.0)
    rotate_to_face(red_x, red_y)

    c = get_gripper_center()
    if c is None:
        raise RuntimeError("no gripper link state yet")
    teleport(RED, c[0], c[1], c[2], duration=0.3)
    set_gripper(GRIP_CLOSE, secs=0.8)
    msg = f"mini_mec_six_arm::link5,{RED},{c[0]:.4f},{c[1]:.4f},{c[2]:.4f}"
    _attach_pub.publish(String(data=msg))
    print(f"    -> attach {msg}")
    rospy.sleep(0.5)


def put_between(mid_x, mid_y, surface_z):
    print("\n[put] navigate close to midpoint and drop")
    ax, ay, ayaw = compute_approach(mid_x, mid_y)
    print(f"    approach: ({ax:.3f}, {ay:.3f}), drop: ({mid_x:.3f}, {mid_y:.3f})")
    if not navigate_to(ax, ay, ayaw):
        raise RuntimeError("move_base failed")
    rotate_to_face(mid_x, mid_y)

    try:
        rospy.wait_for_service("/arm/prepare_put", timeout=3.0)
        rospy.wait_for_service("/arm/drop", timeout=3.0)
        prepare_put = rospy.ServiceProxy("/arm/prepare_put", Trigger)
        drop = rospy.ServiceProxy("/arm/drop", Trigger)

        print("    -> /arm/prepare_put")
        resp = prepare_put()
        if not resp.success:
            raise RuntimeError("/arm/prepare_put failed: " + resp.message)
        rotate_to_face(mid_x, mid_y)

        rospy.set_param("/arm_task/target_name", "ground")
        rospy.set_param("/arm_task/use_target_xy", True)
        rospy.set_param("/arm_task/target_x", float(mid_x))
        rospy.set_param("/arm_task/target_y", float(mid_y))
        rospy.set_param("/arm_task/surface_z", float(surface_z))
        print("    -> /arm/drop with target_xy")
        try:
            resp = drop()
            if not resp.success:
                raise RuntimeError("/arm/drop failed: " + resp.message)
        finally:
            rospy.set_param("/arm_task/use_target_xy", False)
    except rospy.ROSException:
        print("    ! /arm services not available, using direct topic fallback")
        set_arm(ARM_PUT, secs=2.0)
        rotate_to_face(mid_x, mid_y)
        _detach_pub.publish(String(data="detach"))
        rospy.sleep(0.4)
        set_gripper(GRIP_OPEN, secs=0.8)
        teleport(RED, mid_x, mid_y, surface_z + BLOCK_HALF, duration=0.4)
    print(f"    dropped at target ({mid_x:.3f}, {mid_y:.3f}, {surface_z + BLOCK_HALF:.3f})")


def report_error(mid_x, mid_y):
    try:
        resp = _get_model_state(RED, "")
        if resp.success:
            x = resp.pose.position.x
            y = resp.pose.position.y
            err = math.hypot(x - mid_x, y - mid_y)
            print(f"\n[result] red_block=({x:.3f}, {y:.3f}), midpoint=({mid_x:.3f}, {mid_y:.3f}), err={err:.3f} m")
    except Exception as exc:
        print(f"\n[result] cannot read red_block final pose: {exc}")


def init_ros():
    global _cmd_pub, _model_pub, _arm_pub, _gripper_pub, _attach_pub, _detach_pub
    global _mb_client, _get_model_state

    rospy.init_node("between_blocks_test", anonymous=True)
    rospy.Subscriber("/ground_truth/odom", Odometry, odom_cb, queue_size=5)
    rospy.Subscriber("/gazebo/link_states", LinkStates, link_states_cb, queue_size=1)
    _cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    _model_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
    _arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
    _gripper_pub = rospy.Publisher("/hand_controller/command", JointTrajectory, queue_size=1)
    _attach_pub = rospy.Publisher("/block_follower/attach", String, queue_size=1)
    _detach_pub = rospy.Publisher("/block_follower/detach", String, queue_size=1)
    _mb_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    print("waiting for move_base...")
    if not _mb_client.wait_for_server(rospy.Duration(10.0)):
        raise RuntimeError("move_base action server not available")
    rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
    _get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rospy.sleep(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["g4", "c2"])
    ap.add_argument("--assume-held", action="store_true", help="do not create a held red_block first")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--spawn-only", action="store_true", help="only place blue/yellow blocks and update gazebo_scene.yaml")
    args = ap.parse_args()

    if args.dry_run:
        label, surface_z, bx, by, yx, yy, mid_x, mid_y = plan_pair(args.mode)
        z = surface_z + BLOCK_HALF
        print(f"mode={args.mode} near {label}")
        print(f"  blue:   ({bx:.3f}, {by:.3f}, {z:.3f})")
        print(f"  yellow: ({yx:.3f}, {yy:.3f}, {z:.3f})")
        print(f"  mid:    ({mid_x:.3f}, {mid_y:.3f}, {z:.3f})")
        if args.mode == "c2":
            red_x, red_y, red_surface_z = plan_c2_red()
            print(f"  red:    ({red_x:.3f}, {red_y:.3f}, {red_surface_z + BLOCK_HALF:.3f})")
        return

    init_ros()
    mid_x, mid_y, surface_z = spawn_pair(args.mode)
    if args.spawn_only:
        print("\n[done] spawn-only complete")
        return
    if args.mode == "c2":
        run_c2_table_pick()
    elif not args.assume_held:
        prepare_red_held()
    put_between(mid_x, mid_y, surface_z)
    report_error(mid_x, mid_y)


if __name__ == "__main__":
    main()
