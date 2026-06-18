# Prompt used by AdjustLLM to generate recovery code after JudgeLLM/VLM failure.

ADJUST_PROMPT = """
# ALL COORDINATES AND DISTANCES ARE IN METERS (m).
# Generate only Python code that retries or repairs the failed atomic action.
# Keep the recovery short: usually one to three API calls.
# If the feedback says "move closer", "gripper is empty", "grasp_missed", or
# the object is floating/next to the target, retry the same primitive with
# extra_forward=0.08 to 0.12. Use 0.08 first for table objects, 0.10 for floor
# objects, and 0.12 only when the previous attempt was still clearly short.
# Do not use centimeter-style values such as 50, 100, 150, or 200 unless the
# task explicitly uses those as meter coordinates.

import numpy as np
from action.robot_api import (
    move_to_xy,
    move_to_obj_by_offset,
    pick_up_xy,
    pick_up_obj,
    put_down_xy,
    put_down_obj_by_offset,
)
from utils.utils import get_obj_xy, get_obj_size, parse_obj_name, load_L2_memory
from ros_bridge import (is_reachable, get_current_pos, list_obstacles_near,
                        drive_forward, rotate_to_face)
# is_reachable / get_current_pos / list_obstacles_near : query tools (no motion).
# drive_forward(distance, speed=0.15) : RAW /cmd_vel, BYPASSES TEB.
#   Use with NEGATIVE distance to back the robot straight out of a stuck position.
#   No path planning, no rotation — just open-loop motion. Distance < 1 m.
# rotate_to_face(target_x, target_y) : rotate in place via /cmd_vel, BYPASSES TEB.
#   Use after backing up to re-orient toward the original target before re-nav.

objects = load_L2_memory()

# Global Instruction: "pick up the red block"
# Failed Action: Pick up red_block
# VLM Feedback: {"error_type": "grasp_missed", "suggested_correction": "Move closer to red_block and try pick_up_obj(red_block) again."}
red_block_obj = parse_obj_name("red block", objects)
pick_up_obj(red_block_obj, extra_forward=0.10)

# Global Instruction: "pick up the red block on the floor"
# Failed Action: Pick up red_block
# VLM Feedback: {"error_type": "localization_error", "suggested_correction": "Move closer to the red block on the floor and retry the grasp."}
red_block_obj = parse_obj_name("red block", objects)
pick_up_obj(red_block_obj, extra_forward=0.10)

# Global Instruction: "pick up the blue block from the floor and put it into the trash"
# Failed Action: Pick up blue_block
# VLM Feedback: {"error_type": "grasp_missed", "suggested_correction": "Move closer to blue_block and try pick_up_obj(blue_block) again."}
blue_block_obj = parse_obj_name("blue block", objects)
pick_up_obj(blue_block_obj, extra_forward=0.10)

# Global Instruction: "put the held block into the trash bin"
# Failed Action: Put down on Trash_01_001
# VLM Feedback: {"error_type": "placement_error", "suggested_correction": "The block is next to the trash, not inside. Move closer to the trash and retry placement."}
trash_obj = parse_obj_name("Trash_01_001", objects)
put_down_obj_by_offset(trash_obj, 0.0, 0.0, extra_forward=0.10)

# Global Instruction: "move around the sofa in a 4x4 meter square"
# Failed Action: Move around sofa
# VLM Feedback: {"error_type": "localization_error", "suggested_correction": "Retry the square path around the sofa using 2.0 meter half-size."}
sofa_obj = parse_obj_name("sofa", objects)
sofa_x, sofa_y = get_obj_xy(sofa_obj)
move_to_xy(sofa_x + 2.0, sofa_y + 2.0)
move_to_xy(sofa_x + 2.0, sofa_y - 2.0)
move_to_xy(sofa_x - 2.0, sofa_y - 2.0)
move_to_xy(sofa_x - 2.0, sofa_y + 2.0)
move_to_xy(sofa_x + 2.0, sofa_y + 2.0)

# Global Instruction: "go to the trash can with the smallest sum of absolute coordinates, then return to start"
# Failed Action: Move to selected trash
# VLM Feedback: {"error_type": "localization_error", "suggested_correction": "Select the nearest-to-origin trash can and move to its approach point."}
trash_names = objects.get("trash", [])
trash_obj = min(trash_names, key=lambda name: abs(get_obj_xy(name)[0]) + abs(get_obj_xy(name)[1]))
move_to_obj_by_offset(trash_obj, 0.0, 0.0)

# ── Example recovery patterns (illustrative, you may combine differently) ──
# Failed Action: Move to <some_target> (nav state=4)
# VLM Feedback: {"error_type": "obstacle_detected", ...}
# Pattern A — try a TEB waypoint first (good when there's open space nearby):
import math
rx, ry = get_current_pos()
neighbors = list_obstacles_near(rx, ry, radius=2.5)
if neighbors:
    blocker = neighbors[0]
    ox, oy = blocker["x"], blocker["y"]
    dx, dy = rx - ox, ry - oy
    norm = max(math.hypot(dx, dy), 1e-3)
    ux, uy = dx / norm, dy / norm
    for d in (1.0, 1.5, 2.0):
        wp_x, wp_y = rx + ux * d, ry + uy * d
        if is_reachable(wp_x, wp_y):
            move_to_xy(wp_x, wp_y)
            break
# move_to_obj_by_offset(parse_obj_name(<target>, objects), 0.0, 0.0)

# Pattern B — raw backup when TEB keeps spinning out (no path planning, blind):
# (use when Pattern A repeatedly fails or robot is severely wedged)
drive_forward(-0.6)
# rotate_to_face(target_x, target_y)  # optional: re-orient before re-nav
# move_to_obj_by_offset(parse_obj_name(<target>, objects), 0.0, 0.0)
"""
