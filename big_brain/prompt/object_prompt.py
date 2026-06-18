
# 筛key后填 or 直接放
OBJECT_PROMPT = """
import numpy as np
from utils.utils import get_obj_xy,get_obj_size,get_obj_rgb,get_robot_pos

objects = {
    "desk": ['desk1','desk2'],
    "chair": ['chair']
}
# desk1
ret_val = [objects["desk"][0]]

objects = {
    "desk" : ['desk1', 'desk2'],
    "fruits" : ['apple', 'banana', 'pear']
}

# object that can be eaten
ret_val = objects['fruits']

objects = {
    "desk" : ['desk1', 'desk2'],
    "fruits" : ['apple', 'banana', 'pear'],
    "bottle" : ['bottle1', 'bottle2']
}

# the blue bottle
ret_val = []
for bottle_name in objects['bottle']:
    if get_obj_rgb(bottle_name) == "blue":
        ret_val.append(bottle_name)

objects = {
    "red_block": ['red_block1', 'red_block2', 'red_block3'],
}

# the block on the ground
ret_val = []
for block in objects['red_block']:
    z = get_obj_z(block)
    if 0 < z < 10:
        ret_val.append(block)

objects = {
    "chair": ['chair1', 'chair2', 'chair3'],
}
# the leftmost chair
chair_positions = np.array([get_obj_xy(chair_name) for chair_name in objects['chair']])
min_x_idx = np.argmin(chair_positions[:, 0])
ret_val = objects['chair'][min_x_idx]


objects = {
    "bottle" : ['bottle1', 'bottle2'],
    "fruits" : ['lemon']
}

# bottle behind the lemon
lemon_x, lemon_y = get_obj_xy(objects['fruits'][0])
bottle_positions = np.array([get_obj_xy(bottle_name) for bottle_name in objects['bottle']])
behind_bottle_idx = np.where(bottle_positions[:, 1] > lemon_y)[0]
ret_val = [objects['bottle'][idx] for idx in behind_bottle_idx]

objects = {
    "desk" : ['desk1', 'desk2'],
    "bottle" : ['bottle1', 'bottle2', 'bottle3']
}

# the desk which has two bottles on it.
ret_val = []
for obj_name in objects['desk']:
    desk_x, desk_y = get_obj_xy(obj_name)
    desk_size_x, desk_size_y,desk_size_z = get_obj_size(obj_name)
    count = 0
    for bottle_name in objects['bottle']:
        bottle_x, bottle_y = get_obj_xy(bottle_name)
        bottle_z = get_obj_z(bottle_name)
        if desk_x - desk_size_x/2 < bottle_x < desk_x + desk_size_x/2 and desk_y - desk_size_y/2 < bottle_y < desk_y + desk_size_y/2 and bottle_z > desk_size_z:
            count += 1
    if count == 2:
        ret_val.append(obj_name)
        break
if ret_val.isEmpty():
    raise ValueError("No desk has two bottles on it.")

objects = {
    "fruits" : ['apple', 'banana', 'pear']
}

# the fruit which is closest to the robot
ret_val = []
robot_x, robot_y = get_robot_pos()
fruit_positions = np.array([get_obj_xy(fruit_name) for fruit_name in objects['fruits']])
distances = np.linalg.norm(fruit_positions - np.array([robot_x, robot_y]), axis=1)
closest_fruit_idx = np.argmin(distances)
ret_val = [objects['fruits'][closest_fruit_idx]]
"""

CAP_OBJECT_PROMPT = """
import numpy as np
from env_utils import get_obj_pos, get_loc_pos

objects = ['banana', 'plum', 'chips', 'water bottle', 'cookie', 'chair', 'table']
# the banana.
ret_val = 'banana'
objects = ['banana', 'plum', 'chips', 'water bottle', 'cookie', 'chair', 'table']
# the snack closest to the robot.
snack_names = ['banana', 'plum', 'chips', 'cookie']
snack_positions = np.array([get_obj_pos(snack_name) for snack_name in snack_names])
closest_snack_idx = get_closest_idx(points=snack_positions, point=np.zeros(3))
closest_snack_name = snack_names[closest_snack_idx]
ret_val = closest_snack_name
objects = ['countertop', 'cart', 'barstool', 'table', 'swivel chair', 'lawn chair']
# the left most chair.
chair_names = ['barstool', 'swivel chair', 'lawn chair']
chair_positions = np.array([get_loc_pos(chair_name) for chair_name in chair_names])
left_chair_idx = get_left_most_idx(chair_positions)
left_chair_name = chair_names[left_chair_idx]
ret_val = left_chair_name
objects = ['avocado', 'peach', 'coke', 'countertop', 'pear', 'water bottle', 'ramen', 'chair']
# the fruit near the front.
fruit_names = ['avocado', 'peach', 'pear']
fruit_positions = np.array([get_obj_pos(fruit_name) for fruit_name in fruit_names])
front_fruit_idx = get_front_most_idx(fruit_positions)
front_fruit_name = fruit_names[front_fruit_idx]
ret_val = front_fruit_name
objects = ['apple', 'coke', 'banana', 'pear', 'water bottle', 'granola bar']
# a fruit.
fruit_names = ['apple', 'banana', 'pear']
ret_val = np.random.choice(fruit_names)
objects = ['white bowl', 'green bowl', 'sprite can', 'protein bar', 'lemon', 'chocolate bar']
# bars behind the lemon.
bar_names = ['chocolate bar', 'protein bar']
lemon_pos = get_obj_pos('lemon')
use_bar_names = []
for bar_name in bar_names:
    if get_obj_pos(bar_name)[1] > lemon_pos[1]:
        use_bar_names.append(bar_name)
ret_val = use_bar_names
objects = ['white bowl', 'green bowl', 'sprite can', 'protein bar', 'banana', 'water bottle']
# the drinks.
ret_val = ['sprite can', 'water bottle']
objects = ['white bowl', 'green bowl', 'sprite can', 'protein bar', 'banana', 'water bottle']
# the blocks.
ret_val = []
"""