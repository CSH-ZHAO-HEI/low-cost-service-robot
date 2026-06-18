#!/usr/bin/env python3
# coding=utf-8
"""
Global keyboard teleop — works even when Gazebo/RViz is fullscreen.
Keypresses are captured system-wide via pynput (no terminal focus needed).

BASE (/cmd_vel):
  W / S       = forward / backward
  A / D       = rotate left / right
  Q / E       = strafe forward-left / forward-right  (mecanum)
  SPACE       = brake (instant stop)
  Z / X       = base speed +/-  (accelerate / decelerate)

ARM (/arm_controller/command):
  1 / 2       = joint1 +/-   (base rotate)
  C / V       = joint2 +/-   (big arm)
  B / N       = joint3 +/-   (small arm)
  F / G       = joint4 +/-   (wrist pitch)
  T / Y       = joint5 +/-   (wrist roll)
  O / P       = gripper open / close
  M           = arm → arm_clamp  [0, -1.1, 0.66, 1.0, 0]
  ; / '       = arm precision -/+

Ctrl+C in terminal to quit.
"""
import sys
import threading
import rospy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pynput import keyboard

# ── Constants ─────────────────────────────────────────────────────────────────
ARM_CLAMP     = [0.0, -1.1, 0.66, 1.0, 0.0]
JOINT_LIMITS  = [(-3.14, 3.14), (-1.57, 1.57), (-1.57, 1.57),
                 (-1.57, 1.57), (-3.14, 3.14)]
GRIPPER_OPEN  = [ 0.45,  0.45, -0.45, -0.45, -0.45, -0.45]
GRIPPER_CLOSE = [-0.45, -0.45,  0.45,  0.45,  0.45,  0.45]

# Base key → (linear.x, linear.y, angular.z) direction
BASE_KEYS = {
    'w': ( 1,  0,  0),
    's': (-1,  0,  0),
    'a': ( 0,  0,  1),
    'd': ( 0,  0, -1),
    'q': ( 1,  1,  0),
    'e': ( 1, -1,  0),
}

# Arm key → (joint_index, direction)
ARM_KEYS = {
    '1': (0,  1), '2': (0, -1),
    'c': (1,  1), 'v': (1, -1),
    'b': (2,  1), 'n': (2, -1),
    'f': (3,  1), 'g': (3, -1),
    't': (4,  1), 'y': (4, -1),
}

# ── State ──────────────────────────────────────────────────────────────────────
held_keys  = set()       # currently held base movement keys
state_lock = threading.Lock()
joints     = list(ARM_CLAMP)
base_speed = 1.0
arm_prec   = 0.05

pub_vel  = None
pub_arm  = None
pub_hand = None

# ── Helpers ───────────────────────────────────────────────────────────────────
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def send_arm(duration=0.3):
    traj = JointTrajectory()
    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = list(joints)
    pt.time_from_start = rospy.Duration(duration)
    traj.points = [pt]
    pub_arm.publish(traj)

def send_hand(positions, duration=0.5):
    traj = JointTrajectory()
    traj.joint_names = ['joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11']
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = list(positions)
    pt.time_from_start = rospy.Duration(duration)
    traj.points = [pt]
    pub_hand.publish(traj)

def publish_base():
    """Compute and publish /cmd_vel from currently held keys."""
    lx = ly = az = 0.0
    with state_lock:
        keys = set(held_keys)
        spd  = base_speed
    for k in keys:
        if k in BASE_KEYS:
            dx, dy, dz = BASE_KEYS[k]
            lx += dx; ly += dy; az += dz
    # clamp to unit direction
    twist = Twist()
    twist.linear.x  = clamp(lx, -1, 1) * spd
    twist.linear.y  = clamp(ly, -1, 1) * spd
    twist.angular.z = clamp(az, -1, 1) * 3.0
    pub_vel.publish(twist)

# ── pynput callbacks ───────────────────────────────────────────────────────────
def get_char(key):
    """Return lowercase char for a pynput key, or None."""
    try:
        return key.char.lower() if key.char else None
    except AttributeError:
        return None

def on_press(key):
    global joints, base_speed, arm_prec
    ch = get_char(key)

    # ── Brake ──
    if key == keyboard.Key.space:
        with state_lock:
            held_keys.clear()
        twist = Twist()
        pub_vel.publish(twist)
        print("BRAKE")
        return

    if ch is None:
        return

    # ── Base hold keys ──
    if ch in BASE_KEYS:
        with state_lock:
            held_keys.add(ch)
        publish_base()
        return

    # ── Base speed ──
    if ch == 'z':
        with state_lock:
            base_speed = clamp(base_speed + 0.1, 0.05, 3.0)
            s = base_speed
        print("base_speed=%.2f" % s)
        return
    if ch == 'x':
        with state_lock:
            base_speed = clamp(base_speed - 0.1, 0.05, 3.0)
            s = base_speed
        print("base_speed=%.2f" % s)
        return

    # ── Arm joints ──
    if ch in ARM_KEYS:
        idx, sign = ARM_KEYS[ch]
        joints[idx] = clamp(joints[idx] + sign * arm_prec,
                            JOINT_LIMITS[idx][0], JOINT_LIMITS[idx][1])
        send_arm()
        return

    # ── Gripper ──
    if ch == 'o':
        send_hand(GRIPPER_OPEN)
        print("Gripper: OPEN")
        return
    if ch == 'p':
        send_hand(GRIPPER_CLOSE)
        print("Gripper: CLOSE")
        return

    # ── Arm reset ──
    if ch == 'm':
        joints[:] = ARM_CLAMP
        send_arm(duration=2.0)
        print("Arm → arm_clamp")
        return

    # ── Arm precision ──
    if ch == "'":
        arm_prec = clamp(arm_prec + 0.01, 0.01, 0.2)
        print("arm_prec=%.3f" % arm_prec)
        return
    if ch == ';':
        arm_prec = clamp(arm_prec - 0.01, 0.01, 0.2)
        print("arm_prec=%.3f" % arm_prec)
        return

def on_release(key):
    ch = get_char(key)
    if ch and ch in BASE_KEYS:
        with state_lock:
            held_keys.discard(ch)
        publish_base()   # re-publish without released key (or stop if none held)

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global pub_vel, pub_arm, pub_hand

    rospy.init_node('teleop_key')
    pub_vel  = rospy.Publisher('/cmd_vel',                 Twist,           queue_size=5)
    pub_arm  = rospy.Publisher('/arm_controller/command',  JointTrajectory, queue_size=5)
    pub_hand = rospy.Publisher('/hand_controller/command', JointTrajectory, queue_size=5)

    print(__doc__)
    print("base_speed=%.2f  arm_prec=%.3f" % (base_speed, arm_prec))
    print("[ Global keyboard capture active — Gazebo can be fullscreen ]")

    # Start pynput listener in background thread
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        twist = Twist()
        pub_vel.publish(twist)
        print("Teleop off")

if __name__ == '__main__':
    main()
