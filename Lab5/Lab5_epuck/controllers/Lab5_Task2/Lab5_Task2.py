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
cell_known = False
corner_flag = False   # flag for edge case @ nextCellVisited()
row = 0               # init row
col = 0               # init column
x = 0                 # init x
y = 0                 # init y
n = 0                 #* cell unkown
# n = 13              #* cell known
heading = 'X'         #* dirction of robot
if (n > 0): cell_known = True
world = [['.','.','.','.'],['.','.','.','.'],['.','.','.','.'],['.','.','.','.']]

# Webots world updated
# facing North now 90 degrees
N = 90 #0
E = [0, 360] #270
S = 270 #180  
W = 180 #90

class Cylinder:
  def __init__(self, x, y, rgb, color):
    self.x = x
    self.y = y
    self.rgb = rgb
    self.color = color
    self.scanned = False
    self.distance = 0
# RGB for camera color recognition
RGB_YELLOW = [1.0, 1.0, 0.0]
RGB_RED = [1.0, 0.0, 0.0]
RGB_GREEN = [0.0, 1.0, 0.0]
RGB_BLUE = [0.0, 0.0, 1.0]
# create cylinder instances => Cylinder(X, Y, RBG)
yellow_cylinder = Cylinder(-20.0, 20, RGB_YELLOW, 'Y')
red_cylinder = Cylinder(20.0, 20, RGB_RED, 'R')
green_cylinder = Cylinder(-20.0, -20, RGB_GREEN, 'G')
blue_cylinder = Cylinder(20.0, -20, RGB_BLUE, 'B')


#######################################################
# Helpers
#######################################################
def setup():
    global n, x, y
    if n == 0: trilateration() # starting cell unkown
    else: x, y = getRobotXY()

    follow_wall = findHeading()   # T/F if wall on right
    visitCell(n)                  # mark cell with 'X'
    robotState()
    return follow_wall   # if in cell near wall turns toward it

def move():
    drive(TILE)   # updates x,y
    updateCell()  # updates n
    visitCell(n)
    robotState()  # prompts x, y, cell, theata

def checkAllCells():
    for row in world:
        for cell in row:
            if cell == '.':
                return True
    return False

def wallPID(V, Kp, dist):
    error = getDistanceSensors()[1] - dist
    if(-1.7<error<0):
        setSpeedsRPS(V, V - abs(error)*Kp)  # turn away from left wall
    else:
        setSpeedsRPS(V - abs(error)*Kp, V)  # turn towards left wall

def speedCorrection(phi_l, phi_r):
    # set left to max
    if abs(phi_l) > abs(phi_r):
        phi_r = MAX_SPEED * (phi_r/abs(phi_l))   # proportional change to right
        phi_l = MAX_SPEED * (phi_l/abs(phi_l))   # set to max maintain sign
    # set right to max
    elif abs(phi_l) < abs(phi_r):
        phi_l = MAX_SPEED * (phi_l/abs(phi_r))   # proportional change to left
        phi_r = MAX_SPEED * (phi_r/abs(phi_r))   # set to max maintain sign
    # go straight at max
    else:
        phi_l = phi_r = MAX_SPEED * (phi_l/abs(phi_l))

    return phi_l, phi_r

def findNearest(theta):
    # degree array
    arr = np.array([0, 90, 180, 270, 360])

    # calculate the difference array
    difference_array = np.absolute(arr-theta)

    # find the index of minimum element from the array
    index = difference_array.argmin()
    return arr[index]

def promptWorld(world):
    line = []
    print("+---------------+") 
    for row in world:
        line.append(' '.join('| '+str(i) for i in row)+' | ')
    print(' \n'.join(line))
    print("+---------------+") 

def findHeading():
    global n 
    if 2 <= n <= 4: face('W')       # Top row
    elif 14 <= n < 16: face('E')    # Bottom row
    elif n%4 == 0 : face('N')       # Right column 
    else:                           # Left column or in middle
        face('S')
        if n == 6 or n == 7 or n == 10 or n == 11: return False
    return True

def calculateCell():
    global x, y
    x_sign = 1
    y_sign = 1
    if x < 0: x_sign = -1
    if y < 0: y_sign = -1
    grid = [[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]]

    if abs(x) < 10:
        if x_sign < 0: col = 1  # negative x < 10
        else: col = 2           # positive x < 10
    else:
        if x_sign < 0: col = 0  # negative x < 20
        else: col = 3           # positive x < 20
    
    if abs(y) < 10:
        if y_sign < 0: row = 2  # negative y < 10
        else: row = 1           # positive y < 10
    else:
        if y_sign < 0: row = 3  # negative y < 20
        else: row = 0           # positive y < 20

    return grid[row][col]

def getTime(cylinder, direction):
    # Turn time to orientate direction from specified cylinder
    # Time to turn left [N, E, S, W]
    Y_TIME = [2.8, 1.9, 1.2, 0.35]#[2.7, 1.9, 1.024, 0.35] 
    R_TIME = [0.352, 2.784, 1.9, 1.024]
    G_TIME = [1.9, 1.024, 0.4, 2.784]#[1.9, 1.024, 0.35, 2.7]
    B_TIME = [1.024, 0.35, 2.805, 1.9]#[1.024, 0.35, 2.7, 1.9]

    # setup proper time array
    if cylinder == 'Y':
        target = Y_TIME
    elif cylinder == 'R':
        target = R_TIME
    elif cylinder == 'G':
        target = G_TIME
    elif cylinder == 'B':
        target = B_TIME

    # return direction time
    if direction == 'N':
        return target[0]
    elif direction == 'E':
        return target[1]
    elif direction == 'S':
        return target[2]
    elif direction == 'W':
        return target[3]
    else:
        print("Error: Direction unkown")
    return 0
#######################################################
# update cell (n) whether known or not
#######################################################
def updateCell():
    global heading, n, cell_known
    # starting cell known logic
    if cell_known: 
        if heading == 'N': n -= 4
        if heading == 'S': n += 4
        if heading == 'E': n += 1
        if heading == 'W': n -= 1
    else:
        cell_known = True
        n = calculateCell() 

#######################################################
# Center robot on cylinder diagonal from quadrant
#######################################################
def centerCylinder():    
    timed_turn = 7.2   # total 360 time

    # based on (x, y) determine cylinder to center on
    if x >= 0 and y >= 0:     # Quadrant I (+X, +Y), center on Green
        target = green_cylinder
    elif x <= 0 and y >= 0:   # Quadrant II (-X, +Y), center on Blue
        target = blue_cylinder
    elif x <= 0 and y <= 0:   # Quadrant III (-X, -Y), center on Red
        target = red_cylinder
    elif x >= 0 and y <= 0:   # Quadrant IV (+X, -Y), center on Yellow
        target = yellow_cylinder
    else: 
        target = red_cylinder

    spin_stop_time = robot.getTime() + timed_turn   # turn time
    while robot.step(timestep) != -1:
        # begin spin
        setSpeedsRPS(-MAX_SPEED*.2, MAX_SPEED*.2)   # CCW turn

        # center on target cylinder
        if camera.hasRecognition():
            for landmark in camera.getRecognitionObjects(): 
                targetXY = landmark.getPosition()[0], landmark.getPosition()[1]
                X, Y = targetXY
                # centers robot on object
                if (targetXY) and ( (-.1<=X<=.1) or (-.1<=Y<=.1 ) ):
                    # if centered on correct color
                    if landmark.getColors() == target.rgb: 
                        setSpeedsRPS(0, 0)
                        return target.color

        # Should not occur
        if (robot.getTime() >= spin_stop_time):
            setSpeedsRPS(0, 0)
            print("Error: Turned 360 and target was not centered")
            return 'N'

#######################################################
# rotate robot to face direction 
#######################################################
def face(dir):
    global heading
    heading = dir   #* update global heading
    goal_time = 0
    cylinder_color = centerCylinder()
    
    # calc time to turn from cylinder centered on
    if dir == 'N': goal_time = getTime(cylinder_color, 'N')
    elif dir == 'E': goal_time = getTime(cylinder_color, 'E')
    elif dir == 'S': goal_time = getTime(cylinder_color, 'S')
    elif dir == 'W': goal_time = getTime(cylinder_color, 'W')

    goal_time += robot.getTime()   # offset w/ current time
    while robot.step(timestep) != -1:
        # stop once goal reached
        if ( robot.getTime() >= goal_time ):
            setSpeedsRPS(0,0)
            break
        setSpeedsRPS(-MAX_SPEED*0.5, MAX_SPEED*0.5)   # CCW
    return True

#######################################################
# Based on heading update pose (x, y)
#######################################################
def updateRobotsXY(DISTANCE):
    global x, y, heading
    if heading == 'N': y += DISTANCE    # North
    if heading == 'S': y -= DISTANCE    # South
    if heading == 'E': x += DISTANCE    # East
    if heading == 'W': x -= DISTANCE    # West

#######################################################
# Is next cell visited and catchs if in a corner 
#######################################################
wall = 0
def nextCellVisited():
    global world, row, col, corner_flag, heading, wall
    MAX_INDEX = 3
    MIN_INDEX = 0
    col_look_ahead = col
    row_look_ahead = row

    if heading == 'N': row_look_ahead -= 1
    elif heading == 'S': row_look_ahead += 1
    elif heading == 'W': col_look_ahead -= 1
    elif heading == 'E': col_look_ahead += 1

    # messy corner logic
    if (MIN_INDEX > row_look_ahead or row_look_ahead > MAX_INDEX) or (MIN_INDEX > col_look_ahead or col_look_ahead > MAX_INDEX): 
        wall+=1
        return True
    if wall >= 2: 
        wall = 0
        corner_flag = True # out of bounds encountered at corners
        return True        # out of bounds

    if world[row_look_ahead][col_look_ahead] != 'X': return False   #* next cell not visited
    return True

#######################################################
# Rotate robot 360 and scan all cylinder distances 
#######################################################
def scanCylinders():
    timed_turn = 7.2

    spin_stop_time = robot.getTime() + timed_turn   # turn time
    while robot.step(timestep) != -1:
        # begin spin
        setSpeedsRPS(-MAX_SPEED*.2, MAX_SPEED*.2)   # CCW turn

        # setup objects seen for trilateration
        if camera.hasRecognition():
            for landmark in camera.getRecognitionObjects(): 

                cylinder_distance = landmark.getPosition()[0]*39.3701

                if landmark.getColors() == yellow_cylinder.rgb: 
                    if cylinder_distance > yellow_cylinder.distance:
                        yellow_cylinder.distance = cylinder_distance
                        yellow_cylinder.scanned = True

                elif landmark.getColors() == red_cylinder.rgb:
                    if cylinder_distance > red_cylinder.distance:
                        red_cylinder.distance = cylinder_distance
                        red_cylinder.scanned = True

                elif landmark.getColors() == green_cylinder.rgb:
                    if cylinder_distance > green_cylinder.distance:
                        green_cylinder.distance = cylinder_distance
                        green_cylinder.scanned = True

                elif landmark.getColors() == blue_cylinder.rgb:
                    if cylinder_distance > blue_cylinder.distance:
                        blue_cylinder.distance = cylinder_distance
                        blue_cylinder.scanned = True

        if (robot.getTime() >= spin_stop_time):
            setSpeedsRPS(0,0)
            break

#######################################################
# Find x, y using Trilateration
#######################################################
def getRobotXY():
    x = [1]   # init arrays for cylinders
    y = [1]
    r = [1]
    robots_x = 0
    robots_y = 0

    # Gets all cylinder distances
    scanCylinders()

    if yellow_cylinder.scanned:
        r.append(yellow_cylinder.distance)
        x.append(yellow_cylinder.x)
        y.append(yellow_cylinder.y)

    if red_cylinder.scanned: 
        r.append(red_cylinder.distance)
        x.append(red_cylinder.x)
        y.append(red_cylinder.y)

    if green_cylinder.scanned:
        r.append(green_cylinder.distance)
        x.append(green_cylinder.x)
        y.append(green_cylinder.y)

    if blue_cylinder.scanned:
        r.append(blue_cylinder.distance)
        x.append(blue_cylinder.x)
        y.append(blue_cylinder.y)

    # reset flags
    yellow_cylinder.scanned = False  
    red_cylinder.scanned = False
    green_cylinder.scanned = False
    blue_cylinder.scanned = False
    # reset distance
    yellow_cylinder.distance = 50
    red_cylinder.distance = 50
    green_cylinder.distance = 50
    blue_cylinder.distance = 50

    while(True):
        # approximate robot location
        A = ( -2 * x[1] ) + ( 2 * x[2] )
        B = ( -2 * y[1] ) + ( 2 * y[2] )
        C = ( r[1]**2 - r[2]**2 - x[1]**2 ) + ( x[2]**2 - y[1]**2 + y[2]**2 )
        D = ( -2 * x[2] ) + ( 2 * x[3] )
        E = ( -2 * y[2] ) + ( 2 * y[3] )
        F = ( r[2]**2 - r[3]**2 - x[2]**2 ) + ( x[3]**2 - y[2]**2 + y[3]**2 )

        # check for divide by zero and attempt to correct
        if ((E*A - B*D) == 0 or (B*D - E*A) == 0):
            D = ( -2 * x[2] ) + ( 2 * x[4] )
            E = ( -2 * y[2] ) + ( 2 * y[4] )
            F = ( r[2]**2 - r[4]**2 - x[2]**2 ) + ( x[4]**2 - y[2]**2 + y[4]**2 )

        # calculate (x, y) 
        if not (E*A == B*D):
            robots_x = ( C*E - F*B ) / ( E*A - B*D )
            robots_y = ( C*D - A*F ) / ( B*D - E*A )
            break
        else: 
            print("Divide by ZERO error")
            continue

    return robots_x, robots_y

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
# Drives desired distance 'D'
#######################################################
def drive(DISTANCE):
    rps = MAX_SPEED*.5

    IPS = rps * WHEEL_RADIUS    # convert rps to ips
    stop_time = robot.getTime() + ( DISTANCE / IPS )
    setSpeedsRPS(rps, rps)

    while robot.step(timestep) != -1:
        # keep off wall
        if getDistanceSensors()[2] <= 5:
            break
        if getDistanceSensors()[1] < 10:
            wallPID(rps, .1, 5)
        elif getDistanceSensors()[1] < 20:
            wallPID(rps, .1, 15)

        # Checks calculated distance reached
        if robot.getTime() >= stop_time:
            break
    updateRobotsXY(DISTANCE)

#######################################################
# Mark cell visited & prompt updated world map
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
    global x, y, n, heading
    x_rounded = round(x, 3)
    y_rounded = round(y, 3)
    theta = getDegrees(imu.getRollPitchYaw()[2])
    print(f"S = ( {x_rounded}, {y_rounded}, {n}, {heading}, {round(theta)} )")

#######################################################
#* Localization from Colored Landmarks
#######################################################
def trilateration():
    global x, y
    x, y = getRobotXY() # scan and calc x, y
    n = updateCell()    # with x, y known update n

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
follow_wall = setup()

#######################################################
# Main loop:
#######################################################
while robot.step(timestep) != -1:
    # NOTE robot starts in cell 13 facing north
    # |  1  2  3  4 |
    # |  5  6  7  8 |
    # |  9 10 11 12 |
    # | 13 14 15 16 |
    
    NO_CELLS_TO_VISIT = False
    moreCells = True
    count = 0
    # Drive South until in cell w/ wall then put on right
    if not follow_wall:
        while (n == 6 or n == 7 or n == 10 or n == 11): move()
        face('E')
        follow_wall = True

    # When wall infront on corners or coming from middle
    if n == 16 and heading == 'E': face('N')
    if n == 1 and heading == 'W': face('S')
    if ( 2 <= n <= 4 ) and heading == 'N': face('W')
    if ( 13 <= n <= 15 ) and heading == 'S': face('E')

    # True when next cell marked 'X'
    while nextCellVisited():
        if (heading == 'N'): face('E')
        elif (heading == 'E'): face('S')
        elif (heading == 'S'): face('W')
        elif (heading == 'W'): face('N')
        
        count+=1
        if count >= 4:
            moreCells = checkAllCells()
            NO_CELLS_TO_VISIT = True
            break
    
    if NO_CELLS_TO_VISIT:
        if not corner_flag and not moreCells: break   #* Goal complete
        else: corner_flag = False   # else reset flag

    # Free to move cells
    if moreCells:
        move()

# Clean up code
print("All grid cells have been navigated!")
setSpeedsRPS(0,0)