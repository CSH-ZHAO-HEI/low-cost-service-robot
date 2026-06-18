#!/usr/bin/env python3
"""一鍵刪除並重新 spawn 三個方塊，套用最新 SDF 尺寸。"""
import os, sys, time
PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "experiments", "G1-G4"))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "big_brain"))

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import DeleteModel, SpawnModel

MODEL_DIR = os.path.join(PROJECT_ROOT, "small_brain", "models")
BLOCKS = {
    "red_block":    (os.path.join(MODEL_DIR, "red_block",    "model.sdf"), (-0.5,  0.5, 0.025)),
    "blue_block":   (os.path.join(MODEL_DIR, "blue_block",   "model.sdf"), ( 2.15,-0.20, 0.025)),
    "yellow_block": (os.path.join(MODEL_DIR, "yellow_block", "model.sdf"), ( 2.57,-0.20, 0.025)),
}

rospy.init_node("respawn_blocks", anonymous=True)
delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
spawn  = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
rospy.wait_for_service("/gazebo/delete_model", timeout=10)
rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=10)

for name, (sdf_path, xyz) in BLOCKS.items():
    try:
        delete(name)
        print(f"  deleted {name}")
    except Exception:
        pass
    time.sleep(0.3)
    with open(sdf_path) as f:
        sdf = f.read()
    p = Pose()
    p.position.x, p.position.y, p.position.z = xyz
    p.orientation.w = 1.0
    spawn(name, sdf, "", p, "world")
    print(f"  spawned {name}  size from {sdf_path}")
    time.sleep(0.3)

print("done")
