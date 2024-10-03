import math
import pyautogui

FIELD_IMAGE_WIDTH_PIXELS = 2601
FIELD_IMAGE_HEIGHT_PIXELS = 1291

FIELD_WIDTH_PIXELS = FIELD_IMAGE_WIDTH_PIXELS
FIELD_HEIGHT_PIXELS = FIELD_IMAGE_HEIGHT_PIXELS

FIELD_WIDTH_METERS = 16.59128
FIELD_HEIGHT_METERS = 8.211312

FIELD_WIDTH_OFFSET_PIXELS = 0
FIELD_HEIGHT_OFFSET_PIXELS = 0

WINDOW_WIDTH_PIXELS = 1920
WINDOW_HEIGHT_PIXELS = 1012

TASK_BAR_HEIGHT_PIXELS = 40
WINDOW_BAR_HEIGHT_PIXELS = 25

DISPLAY_SCALAR = 0.9418

ROBOT_WIDTH_METERS = 0.635
ROBOT_LENGTH_METERS = 0.7366



#   Front
# 2---x---1
# |   ^   |
# |y<-|   |
# |       |
# 3-------4

CORNER_1_OFFSET = (ROBOT_LENGTH_METERS / 2, -ROBOT_WIDTH_METERS / 2)
CORNER_2_OFFSET = (ROBOT_LENGTH_METERS / 2, ROBOT_WIDTH_METERS / 2)
CORNER_3_OFFSET = (-ROBOT_LENGTH_METERS / 2, ROBOT_WIDTH_METERS / 2)
CORNER_4_OFFSET = (-ROBOT_LENGTH_METERS / 2, -ROBOT_WIDTH_METERS / 2)

CORNER_1_ANGLE = math.atan2(CORNER_1_OFFSET[1], CORNER_1_OFFSET[0])
CORNER_2_ANGLE = math.atan2(CORNER_2_OFFSET[1], CORNER_2_OFFSET[0])
CORNER_3_ANGLE = math.atan2(CORNER_3_OFFSET[1], CORNER_3_OFFSET[0])
CORNER_4_ANGLE = math.atan2(CORNER_4_OFFSET[1], CORNER_4_OFFSET[0])

print(CORNER_1_ANGLE)
print(CORNER_2_ANGLE)
print(CORNER_3_ANGLE)
print(CORNER_4_ANGLE)

def get_dist(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))

CORNER_RADIUS = get_dist(0, 0, CORNER_1_OFFSET[0], CORNER_1_OFFSET[1])

def get_field_dimensions(scalar: float) -> tuple[float]:
    if (FIELD_IMAGE_WIDTH_PIXELS / WINDOW_WIDTH_PIXELS) > (FIELD_IMAGE_HEIGHT_PIXELS / (WINDOW_HEIGHT_PIXELS * scalar)):
        return (WINDOW_WIDTH_PIXELS, FIELD_IMAGE_HEIGHT_PIXELS / (FIELD_IMAGE_WIDTH_PIXELS / WINDOW_WIDTH_PIXELS))
    else:
        return (FIELD_IMAGE_WIDTH_PIXELS / (FIELD_IMAGE_HEIGHT_PIXELS / (WINDOW_HEIGHT_PIXELS * scalar)), WINDOW_HEIGHT_PIXELS * scalar)
    
FIELD_DIMENSIONS_PIXELS_IN_FRAME = get_field_dimensions(DISPLAY_SCALAR)
FIELD_WIDTH_PIXELS_IN_FRAME = FIELD_DIMENSIONS_PIXELS_IN_FRAME[0]
FIELD_HEIGHT_PIXELS_IN_FRAME = FIELD_DIMENSIONS_PIXELS_IN_FRAME[1]

def meters_to_pixels_x(meters_x: float) -> float:
    return meters_x * (FIELD_WIDTH_PIXELS_IN_FRAME / FIELD_WIDTH_METERS)

def pixels_to_meters_x(pixels_x: float) -> float:
    return pixels_x * (FIELD_WIDTH_METERS / FIELD_WIDTH_PIXELS_IN_FRAME)

def meters_to_pixels_y(meters_y: float) -> float:
    return meters_y * (FIELD_HEIGHT_PIXELS_IN_FRAME / FIELD_HEIGHT_METERS)

def pixels_to_meters_y(pixels_y: float) -> float:
    return pixels_y * (FIELD_HEIGHT_METERS / FIELD_HEIGHT_PIXELS_IN_FRAME)

def meters_to_pixels(meters: tuple[float]) -> tuple[float]:
    return (meters_to_pixels_x(meters[0]), meters_to_pixels_y(meters[1]))

def pixels_to_meters(pixels: tuple[float]) -> tuple[float]:
    return (pixels_to_meters_x(pixels[0]), pixels_to_meters_y(pixels[1]))

def get_cursor_pos_pixels() -> tuple[float]:
    x, y = pyautogui.position()
    y = WINDOW_HEIGHT_PIXELS - y + WINDOW_BAR_HEIGHT_PIXELS
    return (x, y)

def get_cursor_pos_meters() -> tuple[float]:
    return pixels_to_meters(get_cursor_pos_pixels())

def deg_to_rad(deg: float):
    return (deg * math.pi) / 180

def rad_to_deg(deg: float):
    return (deg * 180) / math.pi

def in_to_m(inches: float):
    return inches / 39.37

TAG_POSES = [
      [15.079502159004317, 0.2458724917449835, 1.3558547117094235, 2.0943951023931953],
      [16.18516637033274, 0.8836677673355348, 1.3558547117094235, 2.0943951023931953],  
      [16.57937515875032, 4.982727965455931, 1.4511049022098046, 3.141592653589793],    
      [16.57937515875032, 5.547879095758192, 1.4511049022098046, 3.141592653589793],    
      [14.700787401574804, 8.204216408432817, 1.3558547117094235, 4.71238898038469],    
      [1.841503683007366, 8.204216408432817, 1.3558547117094235, 4.71238898038469],     
      [-0.038100076200152405, 5.547879095758192, 1.4511049022098046, 0.0],
      [-0.038100076200152405, 4.982727965455931, 1.4511049022098046, 0.0],
      [0.35610871221742446, 0.8836677673355348, 1.3558547117094235, 1.0471975511965976],
      [1.4615189230378463, 0.2458724917449835, 1.3558547117094235, 1.0471975511965976], 
      [11.90474980949962, 3.713233426466853, 1.3208026416052834, 5.235987755982989],    
      [11.90474980949962, 4.4983489966979935, 1.3208026416052834, 1.0471975511965976],  
      [11.220218440436883, 4.105156210312421, 1.3208026416052834, 3.141592653589793],   
      [5.320802641605283, 4.105156210312421, 1.3208026416052834, 0.0],
      [4.641351282702566, 4.4983489966979935, 1.3208026416052834, 2.0943951023931953],  
      [4.641351282702566, 3.713233426466853, 1.3208026416052834, 4.1887902047863905]
    ]

BACK_CAMERA_POSE = [0, 0, 0, 0, 0, math.pi]
FRONT_CAMERA_POSE = [0.3683, -0.01905, 0.23495, 0, 0, 0]
LEFT_CAMERA_POSE = [0.0172, 0.3429, 0.23495, 0, 0, math.pi / 2]
RIGHT_CAMERA_POSE = [0.073025, -0.3429, 0.23495, 0, 0, 3 * math.pi / 2]
BACK_CAMERA_POSITION_POLAR = [get_dist(0, 0, BACK_CAMERA_POSE[0], BACK_CAMERA_POSE[1]), math.atan2(BACK_CAMERA_POSE[1], BACK_CAMERA_POSE[0])]
FRONT_CAMERA_POSITION_POLAR = [get_dist(0, 0, FRONT_CAMERA_POSE[0], FRONT_CAMERA_POSE[1]), math.atan2(FRONT_CAMERA_POSE[1], FRONT_CAMERA_POSE[0])]
LEFT_CAMERA_POSITION_POLAR = [get_dist(0, 0, LEFT_CAMERA_POSE[0], LEFT_CAMERA_POSE[1]), math.atan2(LEFT_CAMERA_POSE[1], LEFT_CAMERA_POSE[0])]
RIGHT_CAMERA_POSITION_POLAR = [get_dist(0, 0, RIGHT_CAMERA_POSE[0], RIGHT_CAMERA_POSE[1]), math.atan2(RIGHT_CAMERA_POSE[1], RIGHT_CAMERA_POSE[0])]