#!/usr/bin/python3
"""原地拾取姿勢调试脚本。

用途：车子已经停在某个物件（例如掉在车旁的 red_block）非常近的地方，
不需要再导航，直接用一组手臂姿势把它捡起来。

用法：
    python3 C1-C4/inplace_pickup_test.py [object_name]

你只要改下面 INPLACE_READY_POSE / INPLACE_PICK_POSE 这两组关节角度，
重新运行即可——脚本每一步都会把方块瞬移到「当前姿势算出来的夾爪 (x,y)」位置，
不用你自己算 teleport 偏移。

注意：用 time.sleep（wall clock），不依赖 /clock
（目前 /use_sim_time=true 但 /clock 没在发布，rospy.sleep 会卡死）。
"""
import time

import rospy
from gazebo_msgs.msg import LinkStates, ModelState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ════════════════════════════════════════════════════════════════
# 可调姿势常量 —— 在这里改角度，重新运行脚本即可看效果
# ════════════════════════════════════════════════════════════════

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
GRIP_JOINTS = ['joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11']

GRIP_OPEN = [0.45, -0.45, 0.45, 0.45, 0.45, 0.45]
GRIP_CLOSE = [-0.45, 0.45, -0.45, -0.45, -0.45, -0.45]

# 搬运/待机姿势（与 robot_api_demo.ARM_CLAMP_POSE 相同）
ARM_CLAMP_POSE = [0.0, -1.1, 0.12, 1.0, 0.0]

# 原地拾取：物件就在车体附近（比一般 approach 距离更近），
# 手臂需要伸得更短、压得更低。下面两个先给初始猜测值，
# 自己跑起来看夾爪位置再微调。
INPLACE_READY_POSE = [0.0, -1.0, 0.85, 1.0, 0.0]
INPLACE_PICK_POSE = [0.0, -1.2, 1.05, 1.05, 0.0]

BLOCK_HALF = 0.025
OBJECT_NAME = "red_block"

# ════════════════════════════════════════════════════════════════


_link_pos = {}
_model_pub = None
_arm_pub = None
_gripper_pub = None


def _on_link_states(msg):
    for name, pose in zip(msg.name, msg.pose):
        _link_pos[name] = pose.position


def _init_io():
    global _model_pub, _arm_pub, _gripper_pub
    _model_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
    _arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
    _gripper_pub = rospy.Publisher("/hand_controller/command", JointTrajectory, queue_size=1)
    rospy.Subscriber("/gazebo/link_states", LinkStates, _on_link_states, queue_size=1)
    time.sleep(0.5)


def set_arm(pose, secs=1.5):
    traj = JointTrajectory()
    traj.joint_names = ARM_JOINTS
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pose
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    _arm_pub.publish(traj)
    time.sleep(secs + 0.2)


def set_gripper(pos, secs=0.8):
    traj = JointTrajectory()
    traj.joint_names = GRIP_JOINTS
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pos
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    _gripper_pub.publish(traj)
    time.sleep(secs + 0.2)


def get_gripper_center():
    p7 = _link_pos.get("mini_mec_six_arm::link7")
    p9 = _link_pos.get("mini_mec_six_arm::link9")
    if p7 is None or p9 is None:
        return None
    return ((p7.x + p9.x) / 2.0, (p7.y + p9.y) / 2.0, (p7.z + p9.z) / 2.0 - 0.03)


def teleport_model(name, x, y, z, duration=0.3):
    msg = ModelState()
    msg.model_name = name
    msg.reference_frame = "world"
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = float(z)
    msg.pose.orientation.w = 1.0
    t0 = time.time()
    while time.time() - t0 < duration and not rospy.is_shutdown():
        _model_pub.publish(msg)
        time.sleep(0.005)


def main():
    rospy.init_node("inplace_pickup_test", anonymous=True, disable_signals=True)
    _init_io()

    print(f"[inplace_pickup] target object = {OBJECT_NAME}")

    # 1. 开夾爪
    print("[inplace_pickup] STEP 1: 开夹爪")
    set_gripper(GRIP_OPEN)

    # 2. 就位姿势 —— 把方块瞬移到这个姿势的夾爪正下方
    print(f"[inplace_pickup] STEP 2: 移到 INPLACE_READY_POSE = {INPLACE_READY_POSE}")
    set_arm(INPLACE_READY_POSE)
    c = get_gripper_center()
    if c:
        print(f"[inplace_pickup]   gripper center = ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})")
        teleport_model(OBJECT_NAME, c[0], c[1], BLOCK_HALF)

    # 3. 下降到拾取姿势 —— 再次把方块对齐到夾爪正下方
    print(f"[inplace_pickup] STEP 3: 移到 INPLACE_PICK_POSE = {INPLACE_PICK_POSE}")
    set_arm(INPLACE_PICK_POSE)
    c = get_gripper_center()
    if c:
        print(f"[inplace_pickup]   gripper center = ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})")
        teleport_model(OBJECT_NAME, c[0], c[1], BLOCK_HALF)

    time.sleep(0.5)

    # 4. 关夾爪
    print("[inplace_pickup] STEP 4: 关夹爪")
    set_gripper(GRIP_CLOSE)

    # 5. 回到搬运姿势
    print(f"[inplace_pickup] STEP 5: 回到 ARM_CLAMP_POSE = {ARM_CLAMP_POSE}")
    set_arm(ARM_CLAMP_POSE)

    c = get_gripper_center()
    if c:
        print(f"[inplace_pickup]   gripper center = ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})")

    print("[inplace_pickup] 完成。如果夾爪没对准方块，调整上面 INPLACE_READY_POSE / "
          "INPLACE_PICK_POSE 的角度后重新运行。")


if __name__ == "__main__":
    main()
