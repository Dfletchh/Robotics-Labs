from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math


#######################################################
# Creates Robot
#######################################################
robot = Robot()

#######################################################
# Globals 
#######################################################
PI = np.pi
WHEEL_DIAMETER = 1.6   # inch
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_D = 2.28   # inch
WHEEL_D_MID = WHEEL_D / 2
MAX_SPEED = 6.28
TILE = 10
V = 3
WALL_FOLLOW_DISTANCE = 5
need_to_search = False        # flag nextCellVisited()
look_left = []
look_right = []
setup = True
heading = ''
row = 0               # init row
col = 0               # init column
x = 0                 # init x
y = 0                 # init y
HEADING = ['E', 'N', 'W', 'S', 'E']   # [0, 90, 180, 270, 360]
world = [['.','.','.','.'],['.','.','.','.'],['.','.','.','.'],['.','.','.','.']]
wall_config = [['X','X','X','X'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O'],['O','O','O','O']]
#######################################################
#! Starting Cell
n = 16
#######################################################

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
# Helpers
#######################################################
def complete():
    global need_to_search
    for row in world:
        for cell in row:
            if cell == '.':
                if nextCellVisited(): need_to_search = True
                return False
    return True

def drive(D):
    start = leftposition_sensor.getValue()
    while robot.step(timestep) != -1:
        curr = leftposition_sensor.getValue()
        if getDistanceSensors()[2] <= WALL_FOLLOW_DISTANCE-1.5: break   # Front wall
        if getDistanceSensors()[1] <= TILE: wallFollow()
        if WHEEL_RADIUS*abs(curr - start) >= D+0.02: break   # Wheel distance to stop

def findNearest(theta):
    arr = np.array([0, 90, 180, 270, 360])      # degree array
    difference_array = np.absolute(arr-theta)   # calculate the difference array
    index = difference_array.argmin()           # find the index of minimum element from the array
    return arr[index],index

def findXY():
    global x, y, heading, n
    # correct (x, y) with sensors 
    LEFT, RIGHT, FRONT, REAR = getDistanceSensors()
    CELL_XY = {1:[-15,15], 2:[-5,15], 3:[5,15], 4:[15,15], 
               5:[-15,5], 6:[-5,5], 7:[5,5], 8:[15,5], 
               9:[-15,-5], 10:[-5,-5], 11:[5,-5], 12:[15,-5], 
               13:[-15,-15], 14:[-5,-15], 15:[5,-15], 16:[15,-15]}

    x, y = CELL_XY[n]   # init x, y
 
def getDegrees(theta):
    if theta < 0: theta = theta + ( 2 * PI )  # negative correction
    degrees = round(math.degrees(theta), 4)   # convert and round
    if (degrees >= 360): degrees -= 360
    return degrees

def getDistanceSensors():
    return [leftDistanceSensor.getValue()*39.3701, rightDistanceSensor.getValue()*39.3701, frontDistanceSensor.getValue()*39.3701, rearDistanceSensor.getValue()*39.3701]

def nextCellVisited():
    global world, row, col, heading, look_left, look_right
    col_look_ahead = col
    row_look_ahead = row

    if heading == 'N': row_look_ahead -= 1
    elif heading == 'S': row_look_ahead += 1
    elif heading == 'W': col_look_ahead -= 1
    elif heading == 'E': col_look_ahead += 1

    # setup for search if next cell visited 'X' 
    left_right = {'N':[[row-1, col-1], [row-1, col+1]],
                  'S':[[row+1, col+1], [row+1, col-1]],
                  'E':[[col+1, row-1], [col+1, row+1]],
                  'W':[[col-1, row+1], [col-1, row-1]]}

    if 0 <= col_look_ahead <= 3 and 0 <= row_look_ahead <= 3:           # bounds check
        if world[row_look_ahead][col_look_ahead] != 'X': return False   #* next cell not visited  
    else: return False  

    look_left = left_right[heading][0]
    look_right = left_right[heading][1]
    return True

def outsideWall():
    global heading, n
    if getDistanceSensors()[1] > 8:     # Wall not on Right
        if 1 < n <= 4: turn('W')        # Top row (1, 4]
        elif 13 <= n < 16: turn('E')    # Bottom row [13, 16)
        elif n%4 == 0: turn('N')        # Right column (8, 12, 16)
        elif n%4 == 1: turn('S')        # Left column or in middle
        else: turn('N')

def promptWalls():
    wall_config.pop(0)
    count = 0
    line = []
    print("+-------------------------+") 
    print("|  Cell  |  Walls (NWSE)  |")
    print("+-------------------------+") 
    for cell in wall_config:
        count+=1
        if count < 10: line.append(''.join('|   '+str(count)+'    |   '))
        else: line.append(''.join('|   '+str(count)+'   |   '))
        line.append(''.join(' '+str(i) for i in cell))
        line.append(''.join('     |\n'))
    print(''.join(line))
    print("+-------------------------+") 

def promptWorld():
    global world
    line = []
    print("+---------------+") 
    for row in world:
        line.append(' '.join('| '+str(i) for i in row)+' | ')
    print(' \n'.join(line))
    print("+---------------+") 
    robotState()   # pose(x,y,n,theta)

def robotState():
    global x, y, n, heading
    x_rounded = round(x, 3)
    y_rounded = round(y, 3)
    theta = getDegrees(imu.getRollPitchYaw()[2])
    print(f"( {x_rounded}, {y_rounded}, {n}, {heading}, {round(theta)} )")

def search():
    global look_left, look_right, world, heading
    if heading == 'N': direction = ['W', 'E']
    elif heading == 'E': direction = ['N', 'S']
    elif heading == 'S': direction = ['E', 'W']
    elif heading == 'W': direction = ['S', 'N']

    if 0 <= look_left[0] <= 3 and 0 <= look_left[1] <= 3:   # bounds check
        if world[look_left[0]][look_left[1]] == '.':        # visited check
            return direction[0]                         # left turn direction

    if 0 <= look_right[0] <= 3 and 0 <= look_right[1] <= 3: # bounds check
        if world[look_right[0]][look_right[1]] == '.':      # visited check
            return direction[1]                         # right turn direction
    return 'X'  # return a flag otherwise
        
def setSpeedsIPS(vl, vr):
    phi_l = vl/WHEEL_RADIUS
    phi_r = vr/WHEEL_RADIUS    

    if abs(phi_l) > MAX_SPEED or abs(phi_r) > MAX_SPEED:
        phi_l, phi_r = speedCorrection(phi_l, phi_r)

    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

def setSpeedsRPS(phi_l, phi_r):
    if abs(phi_l) > MAX_SPEED or abs(phi_r) > MAX_SPEED:    # speed too high correct
        phi_l, phi_r = speedCorrection(phi_l, phi_r)

    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

def speedCorrection(phi_l, phi_r):
    if abs(phi_l) > abs(phi_r):                  # set left to max
        phi_r = MAX_SPEED * (phi_r/abs(phi_l))   # proportional change to right
        phi_l = MAX_SPEED * (phi_l/abs(phi_l))   # set to max maintain sign
    elif abs(phi_l) < abs(phi_r):                # set right to max
        phi_l = MAX_SPEED * (phi_l/abs(phi_r))   # proportional change to left
        phi_r = MAX_SPEED * (phi_r/abs(phi_r))   # set to max maintain sign
    else:                                        # go straight at max
        phi_l = phi_r = MAX_SPEED * (phi_l/abs(phi_l))
    return phi_l, phi_r

def turn(target=''):
    global heading
    DEGREE = {'N':90,'E':0,'S':270, 'W':180}
    LEFT, RIGHT, FRONT, REAR = getDistanceSensors()
    goal, dir = findNearest(getDegrees(imu.getRollPitchYaw()[2]))

    if heading == target: return   #  already facing heading

    if target == '':
        if RIGHT > TILE: goal -= 90    # CW 90 degree turn
        else:                          # CCW 90 degree turn
            goal += 90
            if (LEFT <= TILE and RIGHT <= TILE): goal += 90   # Deadend 180 degree turn

        # Exceeded range correction
        if goal >= 360: goal -= 360
        elif goal < 0: goal += 360
        goal, dir = findNearest(goal)   # sanity check
    else: 
        goal = DEGREE[target]


    setSpeedsRPS(-MAX_SPEED*0.4, MAX_SPEED*0.4)   # CCW
    while robot.step(timestep) != -1:
        current = getDegrees(imu.getRollPitchYaw()[2])
        if ( pow(current - goal, 2) < 7 ):        # go straight once goal reached
            setSpeedsIPS(V,V)
            break
    updateHeading()

def updateCell():
    global n, heading
    # starting cell known
    if heading == 'N': n -= 4
    elif heading == 'S': n += 4
    elif heading == 'E': n += 1
    elif heading == 'W': n -= 1

def updateHeading():
    global heading
    HEADING = ['E', 'N', 'W', 'S', 'E']   #[0, 90, 180, 270, 360]
    degree, dir = findNearest(getDegrees(imu.getRollPitchYaw()[2]))
    heading = HEADING[dir]

def updateRobotsXY(DISTANCE=0):
    global x, y, heading
    if not DISTANCE: findXY()
    else:
        if heading == 'N': y += DISTANCE      # North
        elif heading == 'S': y -= DISTANCE    # South
        elif heading == 'E': x += DISTANCE    # East
        elif heading == 'W': x -= DISTANCE    # West

def visitCell(cell):
    global world, row, col
    if cell <= 4: row = 0
    elif cell <= 8: row = 1
    elif cell <= 12: row = 2
    elif cell <= 16: row = 3

    col = (cell % 4) - 1
    if col < 0: col = 3

    if world[row][col] != 'X':
        world[row][col] = 'X'   # mark current cell as visited ( "X" )
        promptWorld()

def wallFollow():
    # PID following right wall 
    Kp = 0.1
    error = getDistanceSensors()[1] - WALL_FOLLOW_DISTANCE
    if(-1.7<error<0):
        setSpeedsIPS(V, V - abs(error)*Kp)  # turn towards right wall
    else:
        setSpeedsIPS(V - abs(error)*Kp, V)  # turn away from right wall

#######################################################
# localize will visit cell marking it 'X' and prompt world, Pose
#######################################################
def localize():
    global n, setup, TILE
    updateHeading()
    if setup: 
        updateRobotsXY()
        setup = False
    else:
        updateCell()
        updateRobotsXY(TILE)
    scanWalls()     # sets 'W' flags for maps walls
    visitCell(n)    # marks cell visited 'X'

#######################################################
# Marks wall configuration for current cell
#######################################################
def scanWalls():
    global n
    degree, dir = findNearest(getDegrees(imu.getRollPitchYaw()[2]))
    LEFT, RIGHT, FRONT, REAR = getDistanceSensors()
    walls = ['O','O','O','O']   # North West South East
    heading = HEADING[dir]      # [N:90, W:180, S:270, E:0/360]

    # wall index setup
    if heading == 'N': walls_sensed = [FRONT, LEFT, REAR, RIGHT]   # North West South East
    elif heading == 'W': walls_sensed = [RIGHT, FRONT, LEFT, REAR]
    elif heading == 'S': walls_sensed = [REAR, RIGHT, FRONT, LEFT]
    elif heading == 'E': walls_sensed = [LEFT, REAR, RIGHT, FRONT]

    # walls sensed
    i = 0
    for wall in walls_sensed:
        if wall < 8:
            walls[i] = 'W'
        i+=1

    # uses cell (n) to index worlds wall configuration
    wall_config[n] = walls



#######################################################
# Main loop:
#######################################################
while robot.step(timestep) != -1:
    index = {'N':0,'W':1,'S':2,'E':3}

    # world logic
    localize()

    # wall infront or wall passed
    LEFT, RIGHT, FRONT, REAR = getDistanceSensors()
    if FRONT <= WALL_FOLLOW_DISTANCE-1 or RIGHT > TILE: turn()
    
    # has robot visited every cell
    if complete(): break   

    # move to next cell
    drive(TILE)   #* can remove for the if search else drive

    # wall followed but maze not complete
    # search for remaining cells
    if need_to_search:
        turn_direction = search()
        if turn_direction != 'X': 
            # localize()
            if wall_config[n][index[turn_direction]] != 'W':   # wall check
                turn(turn_direction)
                drive(TILE)
        need_to_search = False
    # else: drive(TILE)

# Clean up code
setSpeedsRPS(0,0)
promptWalls()