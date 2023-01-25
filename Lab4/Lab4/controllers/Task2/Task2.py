from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math


#######################################################
# Globals 
#######################################################
PI = np.pi
WHEEL_DIAMETER = 1.6   # inch
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_D = 2.28   # inch
WHEEL_D_MID = WHEEL_D / 2
MAX_SPEED = 6.28

#######################################################
# Arguements
#######################################################
TILE = 10
CYLINDER_OFFSET = 4.2
TARGET = 5 + CYLINDER_OFFSET
Kp = .1
V = 5
R = 5   #3, 2.58
leave = False   # True when leaving from curve (MG)
goal_reached = False
FOLLOW_DISTANCE = 5

left_rate_of_change = 0
right_rate_of_change = 0
front_rate_of_change = 0

# Left turns, follow wall on right
WALL = 'r'
SIDE = 1

# Right turns, follow wall on left
# WALL = 'l' 
# SIDE = 0

#######################################################
# Return True/False if at a corner
#######################################################
def corner():
    # at a corner
    sensors = getDistanceSensors()
    if (sensors[0] > TILE or sensors[1] > TILE):
        return True
    return False

#######################################################
# Proportionally corrects speeds over max
#######################################################
def speedCorrection(phi_l, phi_r):
    # set left to max
    if abs(phi_l) > abs(phi_r):
        phi_r = MAX_SPEED * (phi_r/abs(phi_l))   # proportional change to right
        phi_l = MAX_SPEED * (phi_l/phi_l)        # set to max maintain sign
    # set right to max
    elif abs(phi_l) < abs(phi_r):
        phi_l = MAX_SPEED * (phi_l/abs(phi_r))   # proportional change to left
        phi_r = MAX_SPEED * (phi_r/phi_r)        # set to max maintain sign
    # go straight at max
    else:
        phi_l = phi_r = MAX_SPEED * (phi_l/phi_l)

    return phi_l, phi_r

#######################################################
# Returns nearest degree 
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
# Gets camera distance in inches converted from meters
#######################################################
def getObjectXY(object):
    return [round(object.getPosition()[0]*39.3701, 4), round(object.getPosition()[1]*39.3701, 4)]

#######################################################
# Gets robots Pose in inches converted from meters
#######################################################
def getRobotsXY():
    return [round(imu.getRollPitchYaw()[0]*39.3701, 4), round(imu.getRollPitchYaw()[1]*39.3701, 4)]

#######################################################
# IMU value in degrees corresponds to current Pose
#######################################################
def getDegrees(theta):
    if theta < 0:
        theta = theta + ( 2 * PI )     # negative correction

    degrees = math.degrees(theta)      # convert
    return round(degrees, 4)

#######################################################
# Gets distance sensors in inches converted from meters
#######################################################
def getDistanceSensors():
    return [leftDistanceSensor.getValue()*39.3701, rightDistanceSensor.getValue()*39.3701, frontDistanceSensor.getValue()*39.3701]
    
#######################################################
# Sets motor velocities to radians/sec 
#######################################################
def setSpeedsRPS(phi_l, phi_r):
    # speed too high correct
    if abs(phi_l) > MAX_SPEED or abs(phi_r) > MAX_SPEED:
        # print("Correcting Speed")
        phi_l, phi_r = speedCorrection(phi_l, phi_r)

    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

#######################################################
# Sets IPS motor velocities to radians/sec 
#######################################################
def setSpeedsIPS(vl, vr):
    phi_l = vl/WHEEL_RADIUS
    phi_r = vr/WHEEL_RADIUS    

    if abs(phi_l) > MAX_SPEED or abs(phi_r) > MAX_SPEED:
        # print("Correcting Speed")
        phi_l, phi_r = speedCorrection(phi_l, phi_r)

    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

#######################################################
# Uses camera to find and face the target object
#######################################################
def locateObject():
    while robot.step(timestep) != -1:
        recognition_arr = camera.getRecognitionObjects()
        setSpeedsRPS(1,-1)

        # if object found
        if (len(recognition_arr) > 0):
            targetXY = getObjectXY(recognition_arr[0])
            X, Y = targetXY
        
            # centers robot on object
            if (targetXY) and ( (-.5<=X<=.5) or (-.5<=Y<=.5) ):
                setSpeedsRPS(0,0)
                break

    return [X, Y, recognition_arr[0].getOrientation()]

#######################################################
# Drives desired curve given a radius 
# Will cancel turn if it can perform Motion to Goal
#######################################################
def curve(R, dir):
    global leave
    threshold = 2 # idealy, 0 but tends to overshoot
    degree_change = 90
    curr_degrees = getDegrees(imu.getRollPitchYaw()[2])
    omega = 1.5/R # v/r 

    # left turns increase degrees
    if dir == 'l':
        goal_degrees = curr_degrees + degree_change
        vr = omega * (R + WHEEL_D_MID)   # signs flipped for left curve
        vl = omega * (R - WHEEL_D_MID)

    # right turns decrease degrees
    elif dir == 'r':
        goal_degrees = curr_degrees - degree_change
        vl = omega * (R + WHEEL_D_MID)
        vr = omega * (R - WHEEL_D_MID)

    # Exceeded range correction
    if goal_degrees > 360: goal_degrees -= 360
    elif goal_degrees < 0: goal_degrees += 360

    setSpeedsIPS(vl, vr)

    # Dirve curve    
    while robot.step(timestep) != -1:
        curr_degrees = getDegrees(imu.getRollPitchYaw()[2])

        # can the camera see the goal
        recognition_arr = camera.getRecognitionObjects()   
        if (len(recognition_arr) > 0):
            targetXY = getObjectXY(recognition_arr[0])
            X, Y = targetXY
            # stop once robot centers object
            if (targetXY) and ( (-.4<=X<=.4) or (-.4<=Y<=.4) ):
                setSpeedsRPS(0,0)
                leave = True
                return leave

        # stop once turn goal reached
        if ( pow(curr_degrees - goal_degrees, 2) < threshold ): 
            setSpeedsRPS(0,0)
            return False

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
# Spin in place
#######################################################
def turn(phi_l, phi_r):
    global leave
    threshold = 10 # idealy, 0 but tends to overshoot
    curr_degrees = getDegrees(imu.getRollPitchYaw()[2])

    # Time interval for extra turn protection
    start_time = robot.getTime()
    actual_time = 1.6   # 90 deg time

    # left turns increase degrees
    if phi_l < phi_r:
        goal_degrees = curr_degrees + 90

    # right turns decrease degrees
    elif phi_r < phi_l:
        goal_degrees = curr_degrees - 90

    # Exceeded range correction
    if goal_degrees > 360: goal_degrees -= 360
    elif goal_degrees < 0: goal_degrees += 360

    # On leave not necassarily a 90 degree turn
    if leave:
        goal_degrees = find_nearest(curr_degrees)

    # Dirve rotation    
    setSpeedsRPS(phi_l, phi_r)
    while robot.step(timestep) != -1:
        curr_degrees = getDegrees(imu.getRollPitchYaw()[2])
        curr_time = robot.getTime()

        # turn until nothing infront and coming from WF -> Curve
        if leave:
            sensors = getDistanceSensors()
            if sensors[2] > TILE and sensors[SIDE] > FOLLOW_DISTANCE:
                setSpeedsRPS(0,0)
                leave = False # now in WF procedure
                break

        # stop once goal reached
        # added time interval to ensure stop
        if ( pow(curr_degrees - goal_degrees, 2) < threshold ) or (actual_time + start_time <= curr_time): 
            setSpeedsRPS(0,0)
            break

#######################################################
# Gradually slows to a stop
#######################################################
def pidStop(targetDistance, Kp):
    # stops at 5 inches (targetDistance)
    x, y = getObjectXY(camera.getRecognitionObjects()[0])
    cur_dist = max(x, y)
    error = cur_dist - targetDistance

    # As error goes to zero slow down proportionally
    if(error < targetDistance):
        v = error * Kp           # PID velocity 
        if (error < 0):
            setSpeedsIPS(v, v)   # reverse
            if ( targetDistance *.9 <= cur_dist <= targetDistance*1.01):
                return True
        setSpeedsIPS(v, v)          # slow down

    return False

#######################################################
# Main Algorithms
# MG - drive toward goal 
# WF - follow wall until can perform MG()
#######################################################
def Motion_To_Goal(distance, orientation):
    global leave
    # print("MG()...")
    setSpeedsRPS(MAX_SPEED,MAX_SPEED)

    # moving toward/away walls
    left_rate_of_change, right_rate_of_change, front_rate_of_change = getDistanceSensors()

    while robot.step(timestep) != -1:
        # if lost sight of goal
        camera_arr = camera.getRecognitionObjects()
        if len(camera_arr) == 0:
            break

        # Returns true when target distance to goal reached
        kp = 1
        if (pidStop(TARGET, kp)):
            return True
            
        # MG unless 5" ( FOLLOW_DISTANCE ) from a wall
        leftDS, rightDS, frontDS = getDistanceSensors()
        if (frontDS <= FOLLOW_DISTANCE) or (leftDS <= FOLLOW_DISTANCE) or (rightDS <= FOLLOW_DISTANCE):
            # Moving away from a wall and at a leave point
            if leave and (right_rate_of_change < rightDS) and (left_rate_of_change < leftDS):
                print("Moving away from wall")
            # Moving toward a wall (WF)
            else:
                setSpeedsRPS(0,0)
                break
    return False

def Wall_Follow(wall):
    # print("WF()...")
    phi_l = -5
    phi_r = 5

    # switch turn direction
    if (wall == 'l'):  
        phi_l = phi_r
        phi_r = -phi_l

    # turn 90 to wall being followed
    turn(phi_l, phi_r)

    # drive wall follw
    while robot.step(timestep) != -1:
        sensors = getDistanceSensors()

        # Check if there's something blocking path (ie "end wall")
        if (sensors[2] <= FOLLOW_DISTANCE):
            # print("WF() Ima hit this wall!")
            setSpeedsIPS(0, 0)  
            turn(phi_l, phi_r)

        # Drove past wall being followed (right by default)
        elif (sensors[SIDE] > TILE):
            # print("WF() Past wall...")
            setSpeedsIPS(0, 0) 
            
            # if during curve can perform MG()
            if ( curve(R, WALL) ):
                break
            if ((sensors[SIDE] > TILE)):
                drive(TILE/2)

        if (wall == 'l'):
            error = sensors[0] - FOLLOW_DISTANCE
            if(error>0):
                setSpeedsIPS(V - abs(error)*Kp, V)  # turn away from right wall
            else:
                setSpeedsIPS(V, V - abs(error)*Kp)  # turn towards right wall
        else:
            error = sensors[1] - FOLLOW_DISTANCE
            if(error>0):
                setSpeedsIPS(V, V - abs(error)*Kp)  # turn away from left wall
            else:
                setSpeedsIPS(V - abs(error)*Kp, V)  # turn towards left wall

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

# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # turn toward yellow object
    x, y, orientation = locateObject()

    # distance to drive toward object
    distance = max(x, y)

    # Motion to Goal 
    if not goal_reached and (Motion_To_Goal(distance, orientation)):
        print("Goal Achived!")
        goal_reached = True

    # Wall follow using Bug 0 
    elif not goal_reached:
        Wall_Follow(WALL)