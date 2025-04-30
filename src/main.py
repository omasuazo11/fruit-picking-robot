# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       omar                                                         #
# 	Created:      4/16/2025, 8:29:05 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

from vex import *

# Brain should be defined by default
brain=Brain()
WHEEL_DIAMETER = 4.0
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.141596
GEAR_RATIO = 5.0
WHEEL_TRACK = 11.0 
MOTOR_DEGREES_PER_INCH = 360.0 * GEAR_RATIO / WHEEL_CIRCUMFERENCE

X_CENTER = 160
Y_CENTER = 120

K_P = 2
K_PR = 4
K_PS = 28
SET_LINE_FOLLOW_SPEED = 150 
FINDING_TREE_STATE = "FINDING TREE"
RAISING_ARM_STATE = "RAISING ARM"
ALIGNING_STATE = "ALIGNING ARM"
STRAFING_STATE = "STRAFING STATE"
GRABBING_STATE = "GRABBING FRUIT"
LOWERING_STATE = "LOWERING STATE"
SORTING_STATE = "SORTING STATE"
REVERSE_STATE = "REVERSE STATE"
GREEN = "GREEN"
ORANGE = "ORANGE"
YELLOW = "YELLOW"
NO_COLOR = "NO_COLOR"

# drive function - For negative values of direction, the robot turns right, and for positive
# values of direction, the robot turns left.  For values of direction with small magnitudes, the
# robot gradually turns.  For values of direction with large magnitudes, the robot turns more
# quickly.

def drive(speed, direction):
   left_motor.set_velocity(speed - direction, RPM)
   right_motor.set_velocity(speed + direction, RPM)
   left_motor.spin(FORWARD)
   right_motor.spin(FORWARD)

def within_range(value, center, threshold):
    return (center - threshold) < value and value < (center + threshold)

def strafe(speed):
    if (speed > 0):
        center_motor.spin(FORWARD, speed, RPM)
    else:
        center_motor.spin(REVERSE, abs(speed), RPM)

def brake():
    left_motor.stop(BRAKE)
    right_motor.stop(BRAKE)
    center_motor.stop(BRAKE)

# Function to turn BaseBot for some number of degrees
def turnInPlace(robotTurnInDegrees):
    # Reset the current rotation to 0
    left_motor.stop(BRAKE)
    right_motor.stop(BRAKE)
    inertial_sensor.reset_rotation()
    rotation = 0
    # Keep rotating until the amount turned is >= to the amount given
    while not within_range(rotation, robotTurnInDegrees, 0.25):
        left_motor.spin(REVERSE, (robotTurnInDegrees - rotation) * K_PR, RPM)
        right_motor.spin(FORWARD, (robotTurnInDegrees - rotation) * K_PR, RPM)
        rotation = inertial_sensor.rotation()
    # Stop ASAP once we get right rotation
    brake()

# Determines whether rotation angle should affect the final moving
def get_rotation_component():
    # If outside range, return the error. If in range, return 0,0 so it doesn't affect
    if not within_range(rotation, 0, 1.25):
        error = rotation * K_PR
        return (error, -error)
    return (0, 0)

# Determines how much to move based on the line trackers
def get_line_component():
    left_value = line_left.reflectivity()
    right_value = line_right.reflectivity()
    if (left_value >= 30 and right_value >= 30):
        return (SET_LINE_FOLLOW_SPEED, SET_LINE_FOLLOW_SPEED)
    # Right sensor off line, turn left
    elif (left_value >= 30 and right_value <= 30):
        return (SET_LINE_FOLLOW_SPEED + 5, SET_LINE_FOLLOW_SPEED - 5)
    # Left sensor off line, turn right
    elif (left_value <= 30 and right_value >= 30):
        return (SET_LINE_FOLLOW_SPEED - 5, SET_LINE_FOLLOW_SPEED + 5)
    # Fully off line, return 0,0 so as to not affect
    else:
        return (40, 40)

# Determines how much h-drive wheel should move to stay at 4 inches from wall
def get_wall_componnet():
    rotation = inertial_sensor.rotation()
    trig_distance = math.cos(rotation * math.pi/180) * distance_left
    # In the case we rotate too much, we jammed so have center wheel move so it unjams itself
    if not(within_range(rotation, 0, 7)):
        return -(4 - trig_distance)*33*rotation
    elif not(within_range(trig_distance, 4, 0.25)) and trig_distance < 15:
        return -(4 - trig_distance)*K_PS
    return 0

# Determines how robot should move based on distance to wall, line trackers and rotation angle given movement direction
def move(direction):
    left_speed_rot, right_speed_rot = get_rotation_component()
    left_speed_line, right_speed_line = get_line_component()
    center_speed = get_wall_componnet()
    if (direction == FORWARD):
        left_motor.set_velocity(left_speed_rot + left_speed_line, RPM)
        right_motor.set_velocity(right_speed_rot + right_speed_line, RPM)
    # If moving backwards, invert the speeds of the wheels
    else:
        left_motor.set_velocity(right_speed_rot + right_speed_line - 20, RPM)
        right_motor.set_velocity(left_speed_rot + left_speed_line - 20, RPM)
    center_motor.set_velocity(center_speed, RPM)
    left_motor.spin(direction)
    right_motor.spin(direction)
    center_motor.spin(FORWARD)

# Detects the fruit
def detect_fruit():
    # Check green, orange, and yellow fruits. Grab largest, make sure its horizontally centered to make sure it's the fruit
    # we're checking, and return it's center y to know when to stop climbing
    green = ai_vision_5.take_snapshot(ai_vision_5__Green)
    orange = ai_vision_5.take_snapshot(ai_vision_5__Orange)
    yellow = ai_vision_5.take_snapshot(ai_vision_5__Yellow)
    max_height = 50
    color = NO_COLOR
    max_fruit = None
    if green:
        for fruit in green:
            if fruit.height > max_height and fruit.height < 300 and fruit.centerY < 350:
                max_fruit = fruit
                max_height = fruit.height
                color = GREEN
    if orange:
        for fruit in orange:
            if fruit.height > max_height and fruit.height < 300 and fruit.centerY < 350:
                max_fruit = fruit
                max_height = fruit.height
                color = ORANGE
    if yellow:
        for fruit in yellow:
            if fruit.height > max_height and fruit.height < 300 and fruit.centerY < 350:
                max_fruit = fruit
                max_height = fruit.height 
                color = YELLOW
        
    return (color, max_fruit)

# Returns the id and x-center of the april tag it sees
def detect_tag():
    tag = ai_vision_5.take_snapshot(AiVision.ALL_TAGS)
    if (tag):
        center_x = tag[0].centerX
        id = tag[0].id
        return (id, center_x)
    else:
        return (-1, -1)
        

# Prints out: Forward sensor value, pitch, both line trackers, yaw/rotation, fruit color being seen, Fruit center x and y, current state, position of elevator,
# and tag id and x-center if tag is being seen 
def print_to_brain():
    brain.screen.print_at("D_F:  " + str(distance_front), x=30, y=30)
    brain.screen.print_at("P:    " + str(pitch), x=30, y=60)
    brain.screen.print_at("L_L:  " + str(left_value), x=30, y=90)
    brain.screen.print_at("L_R: " + str(right_value), x=30, y=120)
    brain.screen.print_at("Yaw:   " + str(rotation), x=30, y = 150)
    brain.screen.print_at("Color:      " + color, x=30, y=180)
    if (fruit):
        brain.screen.print_at("F_Y: " + str(fruit.centerY), x=30, y=210)
        brain.screen.print_at("F_X: " + str(fruit.centerX), x=250, y=90)
        brain.screen.print_at("F_H:" + str(fruit.height), x=250, y=120)
    brain.screen.print_at("State:      " + state, x=30, y=240)
    brain.screen.print_at("E_P: " + str(elevator_motor.position()), x=250, y=150)
    brain.screen.print_at("D_L: " + str(distance_left), x=250, y=60)
    if (tag):
        brain.screen.print_at("Tag id, X:  " + str(tag[0]) + " " + str (tag[1]), x=250, y = 180)

left_motor = Motor(Ports.PORT4, 18_1, True)
center_motor = Motor(Ports.PORT5, 18_1, True)
right_motor = Motor(Ports.PORT6, 18_1, False)
grabbing_motor = Motor(Ports.PORT3, 18_1, False)
elevator_motor = Motor(Ports.PORT2, 18_1, True)
rangeFinderFront = Sonar(brain.three_wire_port.a)
rangeFinderLeft = Sonar(brain.three_wire_port.e)
inertial_sensor = Inertial(Ports.PORT8)
line_left = Line(brain.three_wire_port.d)
line_right = Line(brain.three_wire_port.c)
ai_vision_5__Green = Colordesc(1, 44, 233, 120, 30, 0.57)
ai_vision_5__Yellow = Colordesc(2, 247, 189, 109, 14, 0.12)
ai_vision_5__Orange = Colordesc(3, 244, 122, 115, 10, 0.37)
# AI Vision Code Descriptions
ai_vision_5 = AiVision(Ports.PORT7, ai_vision_5__Green, ai_vision_5__Orange, ai_vision_5__Yellow, AiVision.ALL_TAGS)
#driveStraightInches(108)
inertial_sensor.calibrate()
# Print that the Inertial Sensor is calibrating while
# waiting for it to finish calibrating.
while inertial_sensor.is_calibrating(): 
    brain.screen.print_at("Inertial Sensor Calibrating", x=30, y=30)
    wait(50, MSEC)
brain.screen.clear_screen()

inertial_sensor.set_turn_type(LEFT)

pitch = inertial_sensor.orientation(OrientationType.ROLL)
distance_front = rangeFinderFront.distance(DistanceUnits.IN)
distance_left = rangeFinderLeft.distance(DistanceUnits.IN)
rotation = inertial_sensor.rotation()
left_value = line_left.reflectivity()
right_value = line_right.reflectivity()
color = NO_COLOR
fruit_center_x = -10
fruit = None
dir = -1
tag = detect_tag()
old_tag = -1
state = FINDING_TREE_STATE
print_to_brain()
wait(2000)
pitch = inertial_sensor.orientation(OrientationType.ROLL)
brain.screen.clear_screen()

# Go up the ramp
x=1
dir = 1
while(x<11000):
    print_to_brain()
    rotation = inertial_sensor.rotation()
    pitch = inertial_sensor.orientation(OrientationType.ROLL)
    distance_front = rangeFinderFront.distance(DistanceUnits.IN)
    distance_left = rangeFinderLeft.distance(DistanceUnits.IN)
    left_speed_rot, right_speed_rot = get_rotation_component()
    left_motor.spin(FORWARD, left_speed_rot + 150, RPM)
    right_motor.spin(FORWARD, right_speed_rot + 150, RPM)
    #center_motor.spin(FORWARD, -(5 - math.cos(rotation * math.pi/180) * distance_left)*10, RPM)
    center_motor.spin(FORWARD, dir*40, RPM)
    x+=1
    if (x % 300 == 0):
        dir *= 1
brake()
center_motor.spin_for(REVERSE, 2.2, TURNS, True)
turnInPlace(90 - inertial_sensor.rotation())
center_motor.spin_for(REVERSE, 7, TURNS, True)
wait(200)
turnInPlace(90 - inertial_sensor.rotation())
wait(300)
inertial_sensor.reset_rotation()
center_motor.stop(BRAKE)

while(True):
    pitch = inertial_sensor.orientation(OrientationType.ROLL)
    distance_front = rangeFinderFront.distance(DistanceUnits.IN)
    distance_left = rangeFinderLeft.distance(DistanceUnits.IN)
    rotation = inertial_sensor.rotation()
    left_value = line_left.reflectivity()
    right_value = line_right.reflectivity()
    print_to_brain()  
    # Finding tree state, move forward until april tag is at horizontal center, then stop
    if (state == FINDING_TREE_STATE):
        tag = detect_tag()
        move(FORWARD)
        if within_range(tag[1], X_CENTER + 60, 35) and (tag[0] == old_tag+1 or (old_tag == 1 and tag[0] == 3)):
            brake()
            elevator_direction = FORWARD
            state = RAISING_ARM_STATE
            old_tag = tag[0]
            turnInPlace(-rotation)
            inertial_sensor.reset_rotation()
            center_motor.spin_for(FORWARD, MOTOR_DEGREES_PER_INCH * (rangeFinderLeft.distance(DistanceUnits.IN) - 2), DEGREES, 150, RPM, True)
    # Raise arm until fruit is vertically centered
    elif (state == RAISING_ARM_STATE):
        (color, fruit) = detect_fruit()
        # raise arm
        if (elevator_motor.position() > 890):
            elevator_motor.spin_to_position(0)
            center_motor.spin_for(FORWARD, MOTOR_DEGREES_PER_INCH * (rangeFinderLeft.distance(DistanceUnits.IN) - 4), DEGREES, 80, RPM, True)
            left_motor.spin_for(REVERSE, MOTOR_DEGREES_PER_INCH*12, DEGREES, False)
            right_motor.spin_for(REVERSE, MOTOR_DEGREES_PER_INCH*12, DEGREES, True)
            old_tag -= 1
            state = FINDING_TREE_STATE
            continue

        elevator_motor.spin(FORWARD, 10, RPM)        
        if fruit and fruit.centerY > Y_CENTER + 20:
            elevator_motor.stop(BRAKE)
            state = ALIGNING_STATE
    # Vertically align with tallest fruit
    elif (state == ALIGNING_STATE):
        move(REVERSE)
        (color, fruit) = detect_fruit()
        if (fruit and within_range(fruit.centerX, X_CENTER, 25)):
            brake()
            state = STRAFING_STATE
    # Strafe towards tallest fruit
    elif (state == STRAFING_STATE):
        (color, fruit) = detect_fruit()
        if (fruit):
            center_motor.spin(REVERSE, (145-fruit.height)*0.5, RPM)
            if fruit.height > 138 and fruit.height < 300:
                center_motor.stop(BRAKE)
                elevator_motor.spin_for(FORWARD, 55, DEGREES, True)
                state = GRABBING_STATE
    # Close claw
    elif (state == GRABBING_STATE):
        grabbing_motor.spin(FORWARD, 5.5, VOLT)
        wait(1800)
        state = LOWERING_STATE
    # Lower elevator
    elif (state == LOWERING_STATE):
        elevator_motor.spin_to_position(0, DEGREES, wait=True)
        (color, fruit) = detect_fruit()
        if color == NO_COLOR or (fruit and not within_range(fruit.centerX, X_CENTER, 45) and not within_range(fruit.centerY, Y_CENTER, 35)):
            center_motor.spin_for(FORWARD, MOTOR_DEGREES_PER_INCH * (rangeFinderLeft.distance(DistanceUnits.IN) - 4), DEGREES, 80, RPM, True)
            left_motor.spin_for(REVERSE, MOTOR_DEGREES_PER_INCH*12, DEGREES, False)
            right_motor.spin_for(REVERSE, MOTOR_DEGREES_PER_INCH*12, DEGREES, True)
            old_tag -= 1
            grabbing_motor.spin_for(REVERSE, 230, DEGREES)
            state = FINDING_TREE_STATE
        else:
            state = SORTING_STATE
            grabbing_motor.spin(FORWARD, 1.5, VOLT)
    # Deposit fruit into bin at the end, go until bin is close enough
    elif (state == SORTING_STATE):
        move(FORWARD)
        color = detect_fruit()[0]
        if within_range(distance_front, 4, 0.75):
            brake()
            center_motor.spin_for(FORWARD, 30, DEGREES, True)
            turnInPlace(90 - inertial_sensor.rotation())
            center_motor.spin_for(REVERSE, 30, DEGREES, True)
            grabbing_motor.spin_for(REVERSE, 230, DEGREES, True)
            # Switch lanes
            if (tag[0] == 2):
                turnInPlace(-90)
                center_motor.spin_for(REVERSE, 5, TURNS, 100, RPM, True)
                turnInPlace(-90)
                center_motor.spin_for(FORWARD, 2, TURNS, 100, RPM, True)
                state = FINDING_TREE_STATE
                inertial_sensor.reset_rotation()
                old_tag = -1
            # Tag with id 3 means we're done
            elif (tag[0] == 3):
                brain.screen.clear_screen()
                brain.screen.print("DONEEEEE")
                sys.exit(0)
            # Go back 
            else:
                turnInPlace(-90)
                inertial_sensor.reset_rotation()
                state = REVERSE_STATE
            tag = (-1, -1)
    # Go back until next tree is found
    elif (state == REVERSE_STATE):
        tag = detect_tag()
        move(REVERSE)
        if (within_range(tag[1], X_CENTER + 60, 35) and tag[0] == old_tag + 1 or (old_tag == 1 and tag[0] == 3)):
            state = RAISING_ARM_STATE
            old_tag = tag[0]
            turnInPlace(-rotation)
            inertial_sensor.reset_rotation()
            center_motor.spin_for(FORWARD, MOTOR_DEGREES_PER_INCH * (rangeFinderLeft.distance(DistanceUnits.IN) - 2), DEGREES, 150, RPM, True)
            brake()
                



