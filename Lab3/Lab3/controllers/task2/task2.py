"""lab3_task2 controller"""

""" 
    The robot applies the best KP proportional gain control from Task 1 to the two side sensors
    to follow the maze provided. If the robot reaches any end wall it should make a 90-degree and
    continue navigation. If no 90-degree turns are possible, it should make a 180-degree turn, and
    continue wall following in the opposite direction. The robot should navigate no further away from
    any wall than the distance specified in Task 1. The robot can start at any grid cell with any 
    orientation and should run for no more than 1 minute. During evaluation, the TA will start the 
    robot at different initial position, orientation, and decide whether to follow left or right walls. 
"""

#######################################################
# Import classes
#######################################################
from controller import Robot
import numpy as np
import math



#######################################################
# Globals 
#######################################################
PI = np.pi
WHEEL_DIAMETER = 1.6 # 1.61417
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_D = 2.28 # 2.04724 
WHEEL_D_MID = WHEEL_D / 2
MAX_SPEED = 6.28

#######################################################
# Arguements
#######################################################
WALL_FOLLOW_DISTANCE = 3.0 #3.5
KP1 = .1  #(0.1, 0.5, 1.0, 2.0, 2.5, 5.0)
KP2 = .1
TILE = 10
R = 2.58 #3
v = 5
W = v / R

# INVERSE_WALL = 'l'
# WALL = 'r'
# side = 1
INVERSE_WALL = 'r'
WALL = 'l'
side = 0


#######################################################
# Helpers 
#######################################################
def corner():
    sensors = getDistanceSensors()
    if (sensors[0] > TILE or sensors[1] > TILE):
        return True
    return False

def deadend():
    sensors = getDistanceSensors()
    if (sensors[0] < TILE and sensors[1] < TILE):
        return True
    return False

def speed_correction(phi_l, phi_r):
    # set left to max
    if abs(phi_l) > abs(phi_r):
        phi_r = MAX_SPEED * (phi_r/abs(phi_l))   # proportional change to right
        phi_l = MAX_SPEED * (phi_l/phi_l)        # set to max maintain sign
    # set right to max
    elif abs(phi_l) < abs(phi_r):
        phi_l = MAX_SPEED * (phi_l/abs(phi_r))   # proportional change to right
        phi_r = MAX_SPEED * (phi_r/phi_r)        # set to max maintain sign
    # go straight at max
    else:
        phi_l = phi_r = MAX_SPEED

    return phi_l, phi_r

#######################################################
# Finds nearest degree 
#######################################################
def find_nearest(theta):
    # array
    arr = np.array([0, 90, 180, 270, 360])

    # calculate the difference array
    difference_array = np.absolute(arr-theta)

    # find the index of minimum element from the array
    index = difference_array.argmin()
    return arr[index]

#######################################################
# Rotates in place until desired degree 
#######################################################
def rotate(degree, dir):
    threshold = 10 # idealy, 0 but tends to overshoot
    curr_degrees = getIMUDegrees(imu.getRollPitchYaw()[2])

    # Time interval for extra turn protection
    start_time = robot.getTime()
    if degree == 90:
        actual_time = 1.9    # 90 deg - (1.64)
    else:
        actual_time = 4.14   # 180 dg

    # left turns increase degrees
    if dir == 'l':
        goal_degrees = curr_degrees + degree
        setSpeedsRPS(-MAX_SPEED, MAX_SPEED)

    # right turns decrease degrees
    elif dir == 'r':
        goal_degrees = curr_degrees - degree 
        setSpeedsRPS(MAX_SPEED, -MAX_SPEED)

    # Exceeded range correction
    if goal_degrees > 360: goal_degrees -= 360
    elif goal_degrees < 0: goal_degrees += 360

    # Dirve rotation    
    while robot.step(timestep) != -1:
        curr_degrees = getIMUDegrees(imu.getRollPitchYaw()[2])
        curr_time = robot.getTime()

        # stop once goal reached
        # added time interval to ensure stop
        if ( pow(curr_degrees - goal_degrees, 2) < threshold ) or (actual_time + start_time <= curr_time): 
            setSpeedsRPS(0,0)
            return

#######################################################
# Drives desired curve given a radius 
#######################################################
def curve(R, dir):
    threshold = 5 # idealy, 0 but tends to overshoot
    degree_change = 90
    curr_degrees = getIMUDegrees(imu.getRollPitchYaw()[2])
    omega = v/R

    # left turns increase degrees
    if dir == 'l':
        goal_degrees = curr_degrees + degree_change
        # print("curve(): turning left")        
        vr = omega * (R + WHEEL_D_MID)   # signs flipped for left curve
        vl = omega * (R - WHEEL_D_MID)

    # right turns decrease degrees
    elif dir == 'r':
        goal_degrees = curr_degrees - degree_change
        # print("curve(): turning right")
        vl = omega * (R + WHEEL_D_MID)
        vr = omega * (R - WHEEL_D_MID)

    # Exceeded range correction
    if goal_degrees > 360: goal_degrees -= 360
    elif goal_degrees < 0: goal_degrees += 360
    
    setSpeedsIPS(vl, vr)

    # Dirve curve    
    while robot.step(timestep) != -1:
        curr_degrees = getIMUDegrees(imu.getRollPitchYaw()[2])

        # stop once goal reached
        if ( pow(curr_degrees - goal_degrees, 2) < threshold ): 
            setSpeedsRPS(0,0)
            return

#######################################################
# Drives desired distance 'D'
#######################################################
def drive(D, V=0):
    start_position = leftposition_sensor.getValue()

    setSpeedsRPS(MAX_SPEED,MAX_SPEED)

    while robot.step(timestep) != -1:
        # Checks if wheel distance is larger than D
        if WHEEL_RADIUS*abs(leftposition_sensor.getValue() - start_position) >= D+0.02:
            setSpeedsRPS(0,0)
            break

#######################################################
# Sets motor velocities to radians/sec 
#######################################################
def setSpeedsRPS(phi_l, phi_r):
    # speed too high correct
    if abs(phi_l) > MAX_SPEED or abs(phi_r) > MAX_SPEED:
        # print("Correcting Speed")
        phi_l, phi_r = speed_correction(phi_l, phi_r)

    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

#######################################################
# Sets motor velocities to radians/sec 
#######################################################
def setSpeedsIPS(vl, vr):
    phi_l = vl/WHEEL_RADIUS
    phi_r = vr/WHEEL_RADIUS    

    if abs(phi_l) > MAX_SPEED or abs(phi_r) > MAX_SPEED:
        # print("Correcting Speed")
        phi_l, phi_r = speed_correction(phi_l, phi_r)

    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

#######################################################
# IMU value in degrees corresponds to current Pose
#######################################################
def getIMUDegrees(theta):
    if theta < 0:
        theta = theta + ( 2 * PI )     # negative correction

    degrees = math.degrees(theta)      # convert
    return degrees

#######################################################
# Gets distance sensors in inches converted from meters
#######################################################
def getDistanceSensors():
    return [leftDistanceSensor.getValue()*39.3701, rightDistanceSensor.getValue()*39.3701, frontDistanceSensor.getValue()*39.3701]

#######################################################
# Wall follow algorithm
#######################################################
def wallFollow(wall, distance, kp):
    # Side sensors stay 3 - 7 off walls

    v = 5 # speed, can be changed

    if(wall == 'l'):
        error = getDistanceSensors()[0] - distance
        if(error>0):
            setSpeedsIPS(v - abs(error)*kp, v)  # turn away from right wall
        else:
            setSpeedsIPS(v, v - abs(error)*kp)  # turn towards right wall

    elif(wall == 'r'):
        error = getDistanceSensors()[1] - distance
        if(error>0):
            setSpeedsIPS(v, v - abs(error)*kp)  # turn away from left wall
        else:
            setSpeedsIPS(v - abs(error)*kp, v)  # turn towards left wall

#######################################################
# Creates Robot
#######################################################
robot = Robot()

#######################################################
# Sets the time step of the current world
#######################################################
timestep = int(robot.getBasicTimeStep())

#######################################################
# Gets Robots Distance Sensors
#######################################################
frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)

#######################################################
# Gets Robots Camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#######################################################
# Gets Robots Motors
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)


#######################################################
# Gets Robot's the position sensors
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

pose = imu.getRollPitchYaw()


# Main loop:
while robot.step(timestep) != -1:
    sensors = getDistanceSensors()

    # In open (No Walls) drive forward
    if (sensors[0] > TILE and sensors[1] > TILE and sensors[2] > TILE):
        # print("In the open...")
        drive(TILE/2)

    # Drove past wall being followed (right by default)
    elif (sensors[side] > TILE):
        # print("Past wall...")
        setSpeedsIPS(0, 0)  
        curve(R, WALL)   
        if ((sensors[side] > TILE)):
            drive(0.5)

    # Check if there's something blocking path (ie "end wall")
    elif (sensors[2] < TILE/3):
        # print("Ima hit this wall!") 
        setSpeedsIPS(0, 0)  

        if (corner()):
            rotate(90, INVERSE_WALL)
        elif (deadend()):
            rotate(180, WALL)

    # Maintains a following distance from wall 
    wallFollow(WALL, WALL_FOLLOW_DISTANCE, KP1)   