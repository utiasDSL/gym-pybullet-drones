import math
def manual_direction_setting (vx: float, vy: float, vz: float, intensity: float)->list:
    return [vx, vy, vz, intensity]

def diagonal_qube(intensity: float)->list:
    return [1, 1, 1, intensity]

def diagonal_base_square(intensity: float)->list:
    return [1,   1, 0, intensity]

def along_x_axis(intensity: float, ascending: bool)->list:
    return [1, 0, 0, intensity] if ascending else [-1, 0, 0, intensity]

def along_y_axis(intensity: float, ascending: bool)->list:
    return [0, 1, 0, intensity] if ascending else [0, -1, 0, intensity]

def along_z_axis(intensity: float, ascending: bool)->list:
    return [0, 0, 1, intensity] if ascending else [0, 0, -1, intensity]

def stand_still():
    return [0, 0, 0, 0]

def compare_two(first_list: list, second_list: list)->list:
    out = list(map(sum, zip(first_list, second_list)))
    out[3] = out[3]/2 # здесь можно отказаться от этой строки и вернуть сразу list(map(sum, zip(first_list, second_list)))
    return out

def compare_tree(first_list: list, second_list: list, third_list:list)->list:
    out = list(map(sum, zip(first_list, second_list, third_list)))
    out[3] = out[3]/3 # здесь можно отказаться от этой строки и вернуть сразу list(map(sum, zip(first_list, second_list, third_list)))
    return out

# Плохие функции углов -- их можно улучшить, добавив обертку приведения углов и уменьшив их количетсво вдвое (если получится)))
def angle_between_ox_oy(angle: float, intensity: float, ascending: bool)->list: # positive; 0 < angle < 90 in degrees
    if angle <= 0 or angle >= 90:
        raise(f"Angle gotta be 0 < angle < 90; now angle == {angle}")
    return [1, math.tan(math.radians(angle)), 0, 1] if ascending else [-1, -math.tan(math.radians(angle)), 0, 1]

def angle_between_oy_ox(angle: float, intensity: float, ascending: bool)->list: # positive; 0 < angle < 90 in degrees
    if angle <= 0 or angle >= 90:
        raise(f"Angle gotta be 0 < angle < 90; now angle == {angle}")
    return [math.tan(math.radians(angle)), 1, 0, 1] if ascending else [-math.tan(math.radians(angle)), -1, 0, 1]

def angle_between_ox_oz(angle: float, intensity: float, ascending: bool)->list: # positive; 0 < angle < 90 in degrees
    if angle <= 0 or angle >= 90:
        raise(f"Angle gotta be 0 < angle < 90; now angle == {angle}")
    return [1, 0, math.tan(math.radians(angle)), 1] if ascending else [-1, 0, -math.tan(math.radians(angle)), 1]

def angle_between_oz_ox(angle: float, intensity: float, ascending: bool)->list: # positive; 0 < angle < 90 in degrees
    if angle <= 0 or angle >= 90:
        raise(f"Angle gotta be 0 < angle < 90; now angle == {angle}")
    return [math.tan(math.radians(angle)), 0, 1, 1] if ascending else [-math.tan(math.radians(angle)), 0, -1, 1]

def angle_between_oy_oz(angle: float, intensity: float, ascending: bool)->list: # positive; 0 < angle < 90 in degrees
    if angle <= 0 or angle >= 90:
        raise(f"Angle gotta be 0 < angle < 90; now angle == {angle}")
    return [0, 1, math.tan(math.radians(angle)), 1] if ascending else [0, -1, -math.tan(math.radians(angle)), 1]

def angle_between_oz_oy(angle: float, intensity: float, ascending: bool)->list: # positive; 0 < angle < 90 in degrees
    if angle <= 0 or angle >= 90:
        raise(f"Angle gotta be 0 < angle < 90; now angle == {angle}")
    return [0, math.tan(math.radians(angle)), 1, 1] if ascending else [0, -math.tan(math.radians(angle)), -1, 1]
       