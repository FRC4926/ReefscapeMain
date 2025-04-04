# import json
# import re

# FIELD_Y_DIM = (26*12 + 5) * 0.0254
# PATHPLANNER_ROOT = 'src/main/deploy/pathplanner'
# PATHPLANNER_PATHS = f'{PATHPLANNER_ROOT}/paths'
# PATHPLANNER_AUTOS = f'{PATHPLANNER_ROOT}/autos'

# # returns new position for align command
# def flip_align_position(position: int) -> int:
#     original_to_flipped = {
#         1: 1,
#         2: 6,
#         3: 5,
#         4: 4,
#         5: 3,
#         6: 2,
#     }
#     return original_to_flipped[position]
# def flip_align_command_name(name: str) -> str:
#     align_fmt = r'^Align(Left|Right)([1-6])(Slow)?$'
#     align_match = re.match(align_fmt, name)
#     direction = align_match.group(1)
#     position  = int(align_match.group(2))
#     slow = 'Slow' if align_match.group(3) == 'Slow' else ''

#     flipped_direction = 'Right' if (direction == 'Left') else 'Left'
#     flipped_position  = flip_align_position(position)
#     return f'Align{flipped_direction}{flipped_position}{slow}'

# def name_is_directional(name: str) -> bool:
#     return ('Left' in name) or ('Right' in name)

# def flip_name(name: str) -> str:
#     if 'Left' in name:
#         return name.replace('Left', 'Right')
#     elif 'Right' in name:
#         return name.replace('Right', 'Left')
#     else:
#         return f'{name}.flipped'

# def flip_prev_y(prev_y: float, curr_y: float):
#     dy = curr_y - prev_y
#     new_dy = -dy
#     return curr_y - new_dy
# def flip_next_y(next_y: float, curr_y: float):
#     dy = next_y - curr_y
#     new_dy = -dy
#     return curr_y + new_dy

# def mirror_path(path: dict):
#     for waypoint in path['waypoints']:
#         waypoint['anchor']['y'] = FIELD_Y_DIM - waypoint['anchor']['y']
#         if waypoint['prevControl'] != None:
#             waypoint['prevControl']['y'] = FIELD_Y_DIM - waypoint['prevControl']['y']
#         if waypoint['nextControl'] != None:
#             waypoint['nextControl']['y'] = FIELD_Y_DIM - waypoint['nextControl']['y']
#     for rot in path['rotationTargets']:
#         rot['rotationDegrees'] *= -1
#     path['idealStartingState']['rotation'] *= -1
#     path['goalEndState']['rotation'] *= -1

# # returns flipped name
# def read_mirror_and_write_path(path_name: str) -> str:
#     with open(f'{PATHPLANNER_PATHS}/{path_name}.path', 'r') as f:
#         path = json.loads(f.read())

#     mirror_path(path)
#     flipped_name = flip_name(path_name)

#     with open(f'{PATHPLANNER_PATHS}/{flipped_name}.path', 'w') as f:
#         f.write(json.dumps(path))
#     return flipped_name

# def process_command(command: dict):
#     if command['type'] == 'named':
#         if command['data']['name'].startswith('Align'):
#             command['data']['name'] = flip_align_command_name(command['data']['name'])
#     if command['type'] == 'path':
#         if name_is_directional(command['data']['pathName']):
#             flipped_name = read_mirror_and_write_path(command['data']['pathName'])
#             command['data']['pathName'] = flipped_name
#     if (command['type'] == 'sequential') or (command['type'] == 'parallel'):
#         for child_command in command['data']['commands']:
#             process_command(child_command)

# def read_mirror_and_write_auto(auto_name: str, new_name: str = '') -> str:
#     if new_name == '':
#         new_name = flip_name(auto_name)
    
#     with open(f'{PATHPLANNER_AUTOS}/{auto_name}.auto', 'r') as f:
#         auto = json.loads(f.read())
    
#     for command in auto['command']['data']['commands']:
#         process_command(command)
    
#     with open(f'{PATHPLANNER_AUTOS}/{new_name}.auto', 'w') as f:
#         f.write(json.dumps(auto))

# read_mirror_and_write_auto('ThreeCoralRight', 'ThreeCoralLeft')
# read_mirror_and_write_auto('TwoCoralRight',   'TwoCoralLeft')

