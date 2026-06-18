#!/usr/bin/env python3
"""
Publish arm/gripper poses for quick Gazebo tuning.

Examples:
  rosrun small_brain_sim arm_pose_tuner.py --preset clamp
  rosrun small_brain_sim arm_pose_tuner.py --arm 0 -0.6 0.5 0.5 0
  rosrun small_brain_sim arm_pose_tuner.py --preset put --interactive
"""

import argparse
import shlex
import sys

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]
HAND_JOINTS = ["joint6", "joint7", "joint8", "joint9", "joint10", "joint11"]

ARM_PRESETS = {
    "zero": [0.0, 0.0, 0.0, 0.0, 0.0],
    "clamp": [0.0, -1.1, 0.66, 1.0, 0.0],
    "ground_pick": [0.0, -1.3, 0.66, 1.0, 0.0],
    "put": [0.0, -0.5, 0.3, 0.3, 0.0],
    "pick_high": [0.0, -0.5, 0.3, 0.3, 0.0],
    "pick_high_lower": [0.0, -0.6, 0.5, 0.5, 0.0],
}

HAND_PRESETS = {
    "zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "open": [0.45, -0.45, 0.45, 0.45, 0.45, 0.45],
    "close": [-0.45, 0.45, -0.45, -0.45, -0.45, -0.45],
}


def _csv(values):
    return ", ".join(f"{v:.3f}" for v in values)


def _publish(pub, joints, positions, secs):
    traj = JointTrajectory()
    traj.joint_names = joints
    traj.header.stamp = rospy.Time.now()

    pt = JointTrajectoryPoint()
    pt.positions = [float(v) for v in positions]
    pt.time_from_start = rospy.Duration(float(secs))
    traj.points = [pt]

    pub.publish(traj)


def _print_state(arm, hand):
    print("")
    print(f"arm  [{_csv(arm)}]")
    print(f"hand [{_csv(hand)}]")
    print("")
    print("commands:")
    print("  j <1-5> <delta>       add delta rad to arm joint")
    print("  set <1-5> <value>     set arm joint value rad")
    print("  arm a b c d e         replace full arm pose")
    print("  preset <name>         arm preset: " + ", ".join(sorted(ARM_PRESETS)))
    print("  hand <name>           hand preset: " + ", ".join(sorted(HAND_PRESETS)))
    print("  p                     publish current pose")
    print("  show                  print current pose")
    print("  q                     quit")
    print("")


def _interactive(pub_arm, pub_hand, arm, hand, secs):
    _print_state(arm, hand)
    _publish(pub_arm, ARM_JOINTS, arm, secs)
    _publish(pub_hand, HAND_JOINTS, hand, secs)

    while not rospy.is_shutdown():
        try:
            raw = input("arm-tune> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("")
            return
        if not raw:
            continue

        try:
            parts = shlex.split(raw)
        except ValueError as exc:
            print(f"parse error: {exc}")
            continue

        cmd = parts[0].lower()
        try:
            if cmd in ("q", "quit", "exit"):
                return
            if cmd == "show":
                _print_state(arm, hand)
                continue
            if cmd == "p":
                pass
            elif cmd == "j":
                if len(parts) != 3:
                    raise ValueError("usage: j <1-5> <delta>")
                idx = int(parts[1]) - 1
                if idx < 0 or idx >= len(arm):
                    raise ValueError("joint index must be 1..5")
                arm[idx] += float(parts[2])
            elif cmd == "set":
                if len(parts) != 3:
                    raise ValueError("usage: set <1-5> <value>")
                idx = int(parts[1]) - 1
                if idx < 0 or idx >= len(arm):
                    raise ValueError("joint index must be 1..5")
                arm[idx] = float(parts[2])
            elif cmd == "arm":
                if len(parts) != 6:
                    raise ValueError("usage: arm <j1> <j2> <j3> <j4> <j5>")
                arm[:] = [float(v) for v in parts[1:]]
            elif cmd == "preset":
                if len(parts) != 2 or parts[1] not in ARM_PRESETS:
                    raise ValueError("unknown arm preset")
                arm[:] = list(ARM_PRESETS[parts[1]])
            elif cmd == "hand":
                if len(parts) != 2 or parts[1] not in HAND_PRESETS:
                    raise ValueError("unknown hand preset")
                hand[:] = list(HAND_PRESETS[parts[1]])
            else:
                raise ValueError("unknown command")

            _publish(pub_arm, ARM_JOINTS, arm, secs)
            _publish(pub_hand, HAND_JOINTS, hand, secs)
            print(f"arm=[{_csv(arm)}] hand=[{_csv(hand)}]")
        except ValueError as exc:
            print(f"error: {exc}")


def main():
    parser = argparse.ArgumentParser(
        description="Send arm/gripper joint poses to Gazebo controllers."
    )
    parser.add_argument(
        "--preset",
        choices=sorted(ARM_PRESETS),
        default="clamp",
        help="arm preset to use when --arm is not provided",
    )
    parser.add_argument(
        "--arm",
        nargs=5,
        type=float,
        metavar=("J1", "J2", "J3", "J4", "J5"),
        help="explicit arm joint pose in radians",
    )
    parser.add_argument(
        "--hand",
        choices=sorted(HAND_PRESETS),
        default="zero",
        help="gripper preset",
    )
    parser.add_argument(
        "--secs",
        type=float,
        default=2.0,
        help="trajectory duration in seconds",
    )
    parser.add_argument(
        "--repeat",
        type=float,
        default=0.0,
        help="republish every N seconds until Ctrl+C; 0 means publish once",
    )
    parser.add_argument(
        "-i",
        "--interactive",
        action="store_true",
        help="start an interactive tuning shell after publishing",
    )
    args = parser.parse_args()

    arm = list(args.arm if args.arm is not None else ARM_PRESETS[args.preset])
    hand = list(HAND_PRESETS[args.hand])

    rospy.init_node("arm_pose_tuner", anonymous=False)
    pub_arm = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
    pub_hand = rospy.Publisher("/hand_controller/command", JointTrajectory, queue_size=1)
    rospy.sleep(1.0)

    rospy.loginfo("[arm_pose_tuner] arm=[%s]", _csv(arm))
    rospy.loginfo("[arm_pose_tuner] hand=%s [%s]", args.hand, _csv(hand))
    _publish(pub_arm, ARM_JOINTS, arm, args.secs)
    _publish(pub_hand, HAND_JOINTS, hand, args.secs)

    if args.interactive:
        _interactive(pub_arm, pub_hand, arm, hand, args.secs)
        return

    if args.repeat > 0:
        rate = rospy.Rate(1.0 / args.repeat)
        while not rospy.is_shutdown():
            _publish(pub_arm, ARM_JOINTS, arm, args.secs)
            _publish(pub_hand, HAND_JOINTS, hand, args.secs)
            rate.sleep()
    else:
        rospy.sleep(args.secs + 0.2)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
