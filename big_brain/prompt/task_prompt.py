
BASE_PROMPT = """
# ALL COORDINATES AND DISTANCES ARE IN METERS (m).
# get_obj_xy / get_obj_size return values in meters.
# Offsets passed to move_to_obj_by_offset / put_down_obj_by_offset are in meters.
# Example: 50 cm south = -0.5 (NOT -50). 1 m forward = 1.0 (NOT 100).
# Project convention:
# - If the task says to pick an object from/on "the table" (桌上拿 / 桌上紅色方塊), the table surface is CoffeeTable_01_001.
# - Do not rewrite every generic "table" target to CoffeeTable; only apply this for table-top pickup context.

import numpy as np
from action.robot_api import move_to_xy,move_to_obj_by_offset,pick_up_xy,pick_up_obj,put_down_xy, put_down_obj_by_offset
from action.robot_api import pick_up_from_coffeetable, put_down_between_objs
from action.robot_api import pick_up_coke_from_balconytable, put_down_coke_on_nightstand
from action.robot_api import run_c1_coke_to_nightstand
from action.robot_api import run_c2_red_block_between_blue_yellow
from action.robot_api import run_c3_all_ground_blocks_to_trash
from action.robot_api import run_c4_sofa_patrol_red_to_trash
from utils.utils import parse_obj_name
from utils.utils import get_obj_xy,get_obj_z,get_obj_size,load_L2_memory
from ros_bridge import find_reachable_approach_to, get_current_pos
# find_reachable_approach_to(name, dist=0.55, num_angles=8) -> (x, y) | None
#   For SYMMETRIC objects (round trash bin, ball, anything with 360° symmetry),
#   call this BEFORE move_to_obj_by_offset to pick the side closest to robot
#   that is also reachable. Pattern:
#       wp = find_reachable_approach_to(trash_obj)
#       if wp: move_to_xy(*wp); rotate_to_face(*get_obj_xy(trash_obj))
#       put_down_xy(*get_obj_xy(trash_obj))   # or put_down_obj_by_offset(trash_obj, 0, 0)
#   For directional objects (tables, sofas with a clear "front"), use the
#   canonical move_to_obj_by_offset directly.

objects = load_L2_memory()

# move to coordinates (1.5, 0.8) m and pick up the item at coordinates (1.25, 1.10) m
move_to_xy(1.5, 0.8)
pick_up_xy(1.25, 1.10)

# put down item 0.10 m south of the Table
table_obj = parse_obj_name('Table',objects)
put_down_obj_by_offset(table_obj, 0, -0.10)

# move to chair with a cup on it
chair_obj = parse_obj_name('chair with a cup on it',objects)
move_to_obj_by_offset(chair_obj,0,0)

# move to the desk with the smallest sum of absolute coordinates
desk_obj = parse_obj_name('desk with the smallest sum of absolute coordinates',objects)
move_to_obj_by_offset(desk_obj,0,0)

# pick up "Cup1"
cup_obj = parse_obj_name('Cup1',objects)
pick_up_obj(cup_obj)

# navigate to the red block on the ground and pick it up.
# "on the ground" is a location/state description; the target object is still red_block.
red_block_obj = parse_obj_name('red block', objects)
move_to_obj_by_offset(red_block_obj, 0, 0)
pick_up_obj(red_block_obj)

# pick up the red block from the CoffeeTable, then put it between the blue and yellow blocks
red_block_obj = parse_obj_name('red block', objects)
blue_block_obj = parse_obj_name('blue block', objects)
yellow_block_obj = parse_obj_name('yellow block', objects)
pick_up_from_coffeetable(red_block_obj)
put_down_between_objs(blue_block_obj, yellow_block_obj)

# pick up the Coke can from BalconyTable_01_001 and put it on NightStand_01_001
# This composite Coke/table task needs the tuned C1 helper.
run_c1_coke_to_nightstand()

# pick up the red block from NightStand_01_002 and put it between blue_block and yellow_block
# This composite table-pick task needs the tuned C2 helper.
run_c2_red_block_between_blue_yellow()

# pick all ground blocks red/blue/yellow and put them into Trash_01_001
# This composite cleanup task needs the tuned C3 helper.
run_c3_all_ground_blocks_to_trash()

# follow the N3-style SofaC_01_001 square route; when red_block is detected,
# pick it up, put it into Trash_01_001, return/resume, and complete the route.
# This patrol-and-interrupt task needs the tuned C4 helper.
run_c4_sofa_patrol_red_to_trash()

# move to the table and position 0.5 m south and 0.5 m west of the center
table_obj = parse_obj_name('table',objects)
move_to_obj_by_offset(table_obj, -0.5, 0.5)

# pick up the red fruit from the table and throw it into the leftmost trash can
# Note: trash is symmetric (360° round) — pick the side closest to robot.
table_obj = parse_obj_name('Table',objects)
apple_obj = parse_obj_name('red fruit',objects)
trash_obj = parse_obj_name('leftmost trash can',objects)
move_to_obj_by_offset(table_obj, 0, 0)
pick_up_obj(apple_obj)
# Symmetric target → find which approach side is currently reachable
wp = find_reachable_approach_to(trash_obj, dist=0.55, num_angles=8)
if wp is not None:
    move_to_xy(wp[0], wp[1])
else:
    move_to_obj_by_offset(trash_obj, 0, 0)   # fallback to canonical if no side reachable
put_down_obj_by_offset(trash_obj, 0, 0)

# put the cup next to the blue box (5 cm gap)
cup_obj = parse_obj_name('cup',objects)
box_obj = parse_obj_name('Blue Box',objects)
move_to_obj_by_offset(cup_obj, 0, 0)
pick_up_obj(cup_obj)
move_to_obj_by_offset(box_obj,0,0)
box_length,box_width,box_height = get_obj_size(box_obj)   # all in meters
put_down_obj_by_offset(box_obj, box_length/2+0.05, box_width/2+0.05)

# move around the office chair in a rectangular path of 3 m by 2 m (so half-sizes are 1.5 m and 1.0 m)
chair_obj = parse_obj_name('the office chair',objects)
chair_x,chair_y = get_obj_xy(chair_obj)
move_to_xy(chair_x+1.5, chair_y+1.0)
move_to_xy(chair_x+1.5, chair_y-1.0)
move_to_xy(chair_x-1.5, chair_y-1.0)
move_to_xy(chair_x-1.5, chair_y+1.0)
move_to_xy(chair_x+1.5, chair_y+1.0)

# move to the table which there are two bottles on it, then move to the counter, repeat 3 times
table_obj = parse_obj_name('table which there are two bottles on it',objects)
counter_obj = parse_obj_name('Counter',objects)
for _ in range(3):
    move_to_obj_by_offset(table_obj, 0, 0)
    move_to_obj_by_offset(counter_obj, 0, 0)
"""

OLD_PROMPT = '''
import numpy as np
from action.robot_api import move_to_xy, move_to_obj_by_offset, pick_up_xy, pick_up_obj, put_down_xy, put_down_obj_by_offset
from utils.utils import get_obj_xy, get_obj_size

# move to coordinates (100, 200) and pick up the item at coordinates (125, 220) 
move_to_xy(100, 200)
pick_up_xy(125, 220)

# put down item at 10cm south of the Table
put_down_obj_by_offset('Table', 0, -10)

# move to "Chair"
move_to_obj_by_offset('Chair', 0, 0)

# pick up "Cup1"
pick_up_obj('Cup1')

# put down item at coordinates (5, 10)
put_down_xy(5, 10)

# move to "Table"
move_to_obj_by_offset('Table', 1, 1)

# pick up the red apple from the table and throw it into the trash can
move_to_obj_by_offset('Table', 0, 0)
pick_up_obj('Red_Apple')
move_to_obj_by_offset('Trash_Can', 0, 0)
put_down_obj_by_offset('Trash_Can', 0, 0)

# put the cup next to the blue box
move_to_obj_by_offset('Cup', 0, 0)
pick_up_obj('Cup')
move_to_obj_by_offset('Blue_Box', 0, 0)
box_length,box_width,box_height = get_obj_size('Blue_Box')
put_down_obj_by_offset('Blue_Box', box_length/2+5, box_width/2+5)

# move around the office chair in a rectangular path of 3 meters by 2 meters
chair_x,chair_y = get_obj_xy("chair")
move_to_xy(chair_x+150, chair_y+100)
move_to_xy(chair_x+150, chair_y-100)
move_to_xy(chair_x-150, chair_y-100)
move_to_xy(chair_x-150, chair_y+100)
move_to_xy(chair_x+150, chair_y+100)

# move to the table, then move to the counter, repeat 3 times
for _ in range(3):
    move_to_obj_by_offset('Table', 0, 0)
    move_to_obj_by_offset('Counter', 0, 0)

'''
