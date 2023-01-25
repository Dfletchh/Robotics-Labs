# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
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
corner_flag = False   # flag for edge case @ nextCellVisited()
row = 0               # init row
col = 0               # init column
x = 0                 # init x
y = 0                 # init y
n = 16                #! init cell
# n = 12              # test for cell #12
n = 6               # test for cell #7
world = [['.','.','.','.'],['.','.','.','.'],['.','.','.','.'],['.','.','.','.']]

# Webots world updated
# facing North now 90 degrees
N = 90 #0
E = [0, 360] #270
S = 270 #180  
W = 180 #90


#######################################################
# Helpers
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

def face(dir='', change=0, heading=0):
    threshold = 6.3
    
    if dir == 'N': goal_degrees = N      #90
    elif dir == 'E': goal_degrees = E[0] #0
    elif dir == 'S': goal_degrees = S    #270  
    elif dir == 'W': goal_degrees = W    #180
    else: 
        goal_degrees = heading+change
        # Exceeded range correction
        if goal_degrees > 360: goal_degrees -= 360
        elif goal_degrees < 0: goal_degrees += 360

    while robot.step(timestep) != -1:
        curr_degrees = getDegrees(imu.getRollPitchYaw()[2])
        # stop once goal reached
        if ( pow(curr_degrees - goal_degrees, 2) < threshold ):
            setSpeedsRPS(0,0)
            break
        setSpeedsRPS(-MAX_SPEED*0.2, MAX_SPEED*0.2)

def startingXY():
    global x, y
    face('N')   # algorithm depends on starting in a Northern direction
    leftDS, rightDS, frontDS  = getDistanceSensors()   # in inches

    # init x, y
    x = 20 - round(rightDS)
    y = 20 - round(frontDS)

    # x, y correction logic 
    if (frontDS >= 20):     # negative y
        y = round(frontDS - 20) * -1
    if (rightDS >= 20):     # negative x
        x = round(rightDS - 20) * -1

def updateCell(heading):
    global n
    if heading == N: n -= 4                         # north
    if heading == S: n += 4                         # south
    if heading == E[0] or heading == E[1]: n += 1   # east 
    if heading == W: n -= 1                         # west
    return n

def promptWorld(world):
    line = []
    print("+---------------+") 
    for row in world:
        line.append(' '.join('| '+str(i) for i in row)+' | ')
    print(' \n'.join(line))
    print("+---------------+") 

def nextCellVisited():
    global world, row, col, corner_flag
    MAX_INDEX = 3
    heading = findNearest(getDegrees(imu.getRollPitchYaw()[2]))
    col_look_ahead = col
    row_look_ahead = row

    if heading == N: row_look_ahead -= 1
    elif heading == S: row_look_ahead += 1
    elif heading == W: col_look_ahead -= 1
    elif heading == E[0] or heading == E[1]: col_look_ahead += 1

    if row_look_ahead > MAX_INDEX or col_look_ahead > MAX_INDEX: 
        corner_flag = True # out of bounds encountered at corners
        return True        # out of bounds
    if world[row_look_ahead][col_look_ahead] != 'X': return False
    return True

def localization(HEADING):
    updateCell(HEADING)   # current cell on map
    visitCell(n)          # mark this cell with an 'X'
    updateRobotsXY(TILE, HEADING)   # Current pose (x, y)

#######################################################
# Attempts to put wall on right
#######################################################
def findWall():
    leftDS, rightDS, frontDS = getDistanceSensors()
    if rightDS <= 8: return True   # wall on right 
    elif leftDS <= 8: face('S')    # wall on left put on right
    elif frontDS <= 8: face('W')   # wall in front put on right
    else: face("S")                # face south to check behind robot 

    if getDistanceSensors()[2] <= 8:   # frontDS after first orientation
        face('E')
    else:
        return False
    return True

#######################################################
# Returns nearest degree 
#######################################################
def findNearest(theta):
    # array
    arr = np.array([0, 90, 180, 270, 360])

    # calculate the difference array
    difference_array = np.absolute(arr-theta)

    # find the index of minimum element from the array
    index = difference_array.argmin()
    return arr[index]

#######################################################
# Based on distance and heading update pose (x, y)
#######################################################
def updateRobotsXY(DISTANCE, HEADING):
    global x, y
    if HEADING == N: y += DISTANCE                        # North
    if HEADING == S: y -= DISTANCE                        # South
    if HEADING == E[0] or HEADING == E[1]: x += DISTANCE  # East
    if HEADING == W: x -= DISTANCE                        # West

#######################################################
# IMU value in degrees corresponds to current Pose
#######################################################
def getDegrees(theta):
    if theta < 0:
        theta = theta + ( 2 * PI )            # negative correction
    degrees = round(math.degrees(theta), 4)   # convert and round
    if (degrees >= 360): degrees -= 360
    return degrees

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
        phi_l, phi_r = speedCorrection(phi_l, phi_r)

    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

#######################################################
# Spin in place
#######################################################
def turn(dir, heading=1):
    left = False
    left_turn_speed = [-MAX_SPEED*0.985, MAX_SPEED*0.985]
    right_turn_speed = [MAX_SPEED*0.95, -MAX_SPEED*0.95]
    threshold = 7.5 # idealy, 0 but tends to overshoot
    curr_degrees = getDegrees(imu.getRollPitchYaw()[2])

    # Time interval for extra turn protection
    start_time = robot.getTime()
    actual_time = 2.1   # 90 deg time

    if dir == 'W':
        # Instance: 'N' to 'W'
        if heading == 0 or heading == 360:
            goal_degrees = curr_degrees + 90   # CCW degrees increase
            left = True
        # Instance: 'S' to 'W'
        elif heading == 180:
            goal_degrees = curr_degrees - 90   # CW degrees decrease
    # Instance: 'W' to 'N'
    elif dir == 'N':
        goal_degrees = curr_degrees - 90
    # Instance: 'W' to 'S'
    elif dir == 'S':
        goal_degrees = curr_degrees + 90
        left = True

    # Exceeded range correction
    if goal_degrees > 360: goal_degrees -= 360
    elif goal_degrees < 0: goal_degrees += 360

    if left: 
        setSpeedsRPS(left_turn_speed[0], left_turn_speed[1])
        threshold = 5   # better for left turns
    else: setSpeedsRPS(right_turn_speed[0], right_turn_speed[1])

    goal_degrees = findNearest(goal_degrees)
    # Dirve rotation
    while robot.step(timestep) != -1:
        curr_degrees = getDegrees(imu.getRollPitchYaw()[2])
        curr_time = robot.getTime()

        # stop once goal reached & time interval to ensure stop
        if ( pow(curr_degrees - goal_degrees, 2) < threshold ):#! or (actual_time + start_time <= curr_time): 
            setSpeedsRPS(0,0)
            break

#######################################################
# Drives desired distance 'D'
#######################################################
def drive(DISTANCE):
    global cell_count
    start_position = leftposition_sensor.getValue()

    setSpeedsRPS(MAX_SPEED,MAX_SPEED)

    while robot.step(timestep) != -1:
        # wall infront
        if getDistanceSensors()[2] <= 5:
            break

        # Checks if wheel distance is larger than Distance
        if WHEEL_RADIUS*abs(leftposition_sensor.getValue() - start_position) >= DISTANCE+0.02:
            break

#######################################################
# Mark cell visited & return updated world map
#######################################################
def visitCell(cell):
    global world, row, col
    if cell <= 4: row = 0
    elif cell <= 8: row = 1
    elif cell <= 12: row = 2
    elif cell <= 16: row = 3

    col = (cell % 4) - 1
    if col < 0: col = 3
    world[row][col] = 'X'   # mark current cell as visited ( "X" )
    promptWorld(world)

#######################################################
# Prompts Robots Pose (x, y, n, theta)
#######################################################
def robotState():
    global x, y, n
    theta = getDegrees(imu.getRollPitchYaw()[2])
    # print("S = (", str(x)+",", str(y)+",", str(n)+",", round(theta,2), ")")
    print(f"S = ( {x}, {y}, {n}, {round(theta)} )")

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
# Gets Robots Cameras 
# (4 cameras facing front, right, rear, and left)
#######################################################
cameras = []
cameras.append(robot.getDevice('front camera'))
# cameras.append(robot.getDevice('right camera'))
# cameras.append(robot.getDevice('rear camera'))
# cameras.append(robot.getDevice('left camera'))

# Enables all cameras
for camera in cameras:
    camera.enable(timestep)
    camera.recognitionEnable(timestep)

#######################################################
# cameras array map
#   [0] -> front
#   [1] -> right
#   [2] -> rear
#   [3] -> left
#######################################################
# default referance to front camera
camera = cameras[0]

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

#######################################################
# World setup
#######################################################
startingXY()
visitCell(n)
robotState()
follow_wall = findWall()   # T/F if wall on right

#######################################################
# Main loop:
#######################################################
while robot.step(timestep) != -1:
    # NOTE robot starts in cell 16 facing north
    # | . . . . | |  1  2  3  4 |
    # | . . . ^ | |  5  6  7  8 |
    # | . . . ^ | |  9 10 11 12 |
    # | . . . x | | 13 14 15 16 |
    
    NO_CELLS_TO_VISIT = False
    count = 0
    # Drive South until wall found then put on right
    if not follow_wall:
        while (getDistanceSensors()[2] >= 7):
            drive(TILE)
            localization(S)   # South degrees
            robotState()      # prompts x, y, cell, theata
        face('E')
        follow_wall = True

    # When wall infront
    leftDS, rightDS, frontDS = getDistanceSensors()
    if(frontDS <= 6):
        heading = findNearest(getDegrees(imu.getRollPitchYaw()[2]))
        face('',90, heading)   # turn left

    # True when next cell marked 'X'
    while nextCellVisited():
        heading = findNearest(getDegrees(imu.getRollPitchYaw()[2]))
        face('',90, heading)   # turn left
        count+=1
        if count >=4: 
            NO_CELLS_TO_VISIT = True
            break
    
    if NO_CELLS_TO_VISIT:
        if not corner_flag: break   #* Goal complete
        else: corner_flag = False   # else reset flag

    # Free to move cells
    heading = findNearest(getDegrees(imu.getRollPitchYaw()[2]))
    drive(TILE)   # Drive a tile per iter
    localization(heading)
    robotState()   # prompts x, y, cell, theata

# Clean up code
print("All grid cells have been navigated!")
setSpeedsRPS(0,0)