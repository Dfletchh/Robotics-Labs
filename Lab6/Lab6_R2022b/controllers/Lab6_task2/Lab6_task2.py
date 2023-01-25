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
setup = True            # flag inital setup
heading = ''            # init NWES
row = 0                 # init row
col = 0                 # init column
x = 0                   # init x
y = 0                   # init y
planned_path = {}
world = [['.','.','.','.'],['.','.','.','.'],['.','.','.','.'],['.','.','.','.']]
nbrs = {1:[],2:[],3:[],4:[],5:[],6:[],7:[],8:[],9:[],10:[],11:[],12:[],13:[],14:[],15:[],16:[]}
visited = {1:'N',2:'N',3:'N',4:'N',5:'N',6:'N',7:'N',8:'N',9:'N',10:'N',11:'N',12:'N',13:'N',14:'N',15:'N',16:'N'}
wave_config = {1:0,2:0,3:0,4:0,5:0,6:0,7:0,8:0,9:0,10:0,11:0,12:0,13:0,14:0,15:0,16:0}
wall_config = {1:['W','W','O','W'],2:['O','W','O','W'],3:['O','W','O','O'],4:['O','W','W','O'],
               5:['W','W','W','O'],6:['W','W','W','O'],7:['W','O','W','O'],8:['W','O','W','O'],
               9:['W','O','W','O'],10:['W','O','O','W'],11:['O','O','W','W'],12:['W','O','W','O'],
               13:['W','O','O','W'],14:['O','W','O','W'],15:['O','W','O','W'],16:['O','O','W','W']}
#######################################################
n = 5               #! Starting Cell
goal_cell = 6       #! Ending Cell
wave_cell = goal_cell   # init wave count
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
def drive(D):
    start = leftposition_sensor.getValue()
    while robot.step(timestep) != -1:
        curr = leftposition_sensor.getValue()
        if getDistanceSensors()[2] <= WALL_FOLLOW_DISTANCE-1.5: break   # Front wall
        if getDistanceSensors()[1] <= TILE: wallFollow()
        if WHEEL_RADIUS*abs(curr - start) >= D+0.02: break   # Wheel distance to stop

def findXY():
    global x, y, n
    CELL_XY = {1:[-15,15], 2:[-5,15], 3:[5,15], 4:[15,15], 
               5:[-15,5], 6:[-5,5], 7:[5,5], 8:[15,5], 
               9:[-15,-5], 10:[-5,-5], 11:[5,-5], 12:[15,-5], 
               13:[-15,-15], 14:[-5,-15], 15:[5,-15], 16:[15,-15]}

    x, y = CELL_XY[n]   # init x, y

def findNearest(theta):
    arr = np.array([0, 90, 180, 270, 360])      # degree array
    difference_array = np.absolute(arr-theta)   # calculate the difference
    index = difference_array.argmin()           # find the index of minimum element from the array
    return arr[index],index

def getDegrees(theta):
    if theta < 0: theta = theta + ( 2 * PI )  # negative correction
    degrees = round(math.degrees(theta), 4)   # convert and round
    if (degrees >= 360): degrees -= 360
    return degrees

def getDistanceSensors():
    return [leftDistanceSensor.getValue()*39.3701, rightDistanceSensor.getValue()*39.3701, frontDistanceSensor.getValue()*39.3701, rearDistanceSensor.getValue()*39.3701]

def promptPlan(plan):
    # cellList = [key for key in plan]
    print(f'------------------------------')
    print(f'Planned Path: {[key for key in plan]}')
    print(f'------------------------------')
    print(f' \n')

def promptWorld():
    global world
    line = []
    print("+---------------+") 
    for row in world:
        line.append(' '.join('| '+str(i) for i in row)+' | ')
    print(' \n'.join(line))
    print("+---------------+") 
    robotState()   # pose(x,y,n,theta)

def reachable(curr, next):
    # checks if next cell is a valid neighbor
    for nbr in nbrs[curr]:
        if nbr == next: return True
    return False

def robotState():
    global x, y, n, heading
    x_rounded = round(x, 3)
    y_rounded = round(y, 3)
    theta = getDegrees(imu.getRollPitchYaw()[2])
    print(f"( {x_rounded}, {y_rounded}, {n}, {heading}, {round(theta)} )")

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

    else: phi_l = phi_r = MAX_SPEED * (phi_l/abs(phi_l))   # go straight at max
    return phi_l, phi_r

def turn(target):
    global heading
    if heading == target: return   # no turn necessary
    heading = target               # update global heading
    DEGREE = {'N':90,'E':0,'S':270, 'W':180}
    goal = DEGREE[target]

    setSpeedsRPS(-MAX_SPEED*0.4, MAX_SPEED*0.4)   # CCW
    while robot.step(timestep) != -1:
        current = getDegrees(imu.getRollPitchYaw()[2])
        if ( pow(current - goal, 2) < 6 ):        # go straight once goal reached
            setSpeedsIPS(V,V)
            break

def updateCell():
    global n, heading
    # starting cell known
    if heading == 'N': n -= 4
    elif heading == 'S': n += 4
    elif heading == 'E': n += 1
    elif heading == 'W': n -= 1

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
# Gets the cost of possible moves from goal to start
#######################################################
def findCost(nbr, wave):
    global nbrs, visited, wave_config, n
    cost = wave_config[nbr]

    if wave >= 16: return   # base case

    if visited[nbr] == 'V':
        if wave < cost: wave_config[nbr] = wave   # found shorter route
    else: 
        visited[nbr] = 'V'   # mark myself visited
        wave_config[nbr] = wave

    for next in nbrs[nbr]: findCost(next, wave+1)

#######################################################
# Gets neightbors with 4-Point 
#######################################################
def getNeighbors(cell=1):
    global wall_config, nbrs
    if cell > 16: return   # base case

    # This waves 4-Point cell numbers
    north = cell - 4  # up
    south = cell + 4  # down
    west = cell - 1   # left
    east = cell + 1   # right

    if 1 <= west <= 16 and wall_config[cell][0] != 'W': nbrs[cell].append(west)      # West    
    if 1 <= north <= 16 and wall_config[cell][1] != 'W': nbrs[cell].append(north)    # North
    if 1 <= east <= 16 and wall_config[cell][2] != 'W':                              # East
        if cell%4 != 0:  nbrs[cell].append(east)   # index wraps to begining of next row here
    if 1 <= south <= 16 and wall_config[cell][3] != 'W': nbrs[cell].append(south)    # South

    getNeighbors(cell+1)

#######################################################
# localize will visit cell marking it 'X' and prompt world, Pose
#######################################################
def localize():
    global heading, n, setup, TILE
    HEADING = ['E', 'N', 'W', 'S', 'E']   #[0, 90, 180, 270, 360]
    degree, dir = findNearest(getDegrees(imu.getRollPitchYaw()[2]))
    heading = HEADING[dir]
    if setup: 
        updateRobotsXY()
        setup = False
    else:
        updateCell()
        updateRobotsXY(TILE)
    visitCell(n)    # marks cell visited 'X'

#######################################################
# Plan path after Wavefront Planner 
#######################################################
def planPath():
    global n, wave_config
    route = dict(reversed(sorted(wave_config.items(), key=lambda item: item[1])))
    prev_cell = n
    prev_cost = route[n]
    path = {n: route[n]}   # init to starting cell

    # for every cell closer to goal
    for cell in route:
        if route[cell] < prev_cost:          # follows decreasing costs
            if reachable(prev_cell, cell):   # next cell is reachable from prev cell (ie a valid neighbor)
                path[cell] = route[cell]     # add to path
                prev_cell = cell             # upate prevs
                prev_cost = route[cell]
    promptPlan(path)
    return path

#######################################################
# Run path planned after Wavefront Planner 
#######################################################
def runPlan(path):
    global n, goal_cell
    c = 0
    cells = [cell for cell in path]       # upack dict
    while robot.step(timestep) != -1:
        localize()
        if n == goal_cell: return True    #* Done
        curr, next = cells[c], cells[c+1]
        if abs(next - curr) == 1:                   # left or rigth a column (+/- x)
            if next > curr: turn('E')
            else: turn('W')
        else:                                       # up or down row  (+/- y)
            if next > curr: turn('S')
            else: turn('N')
        drive(TILE)
        c+=1
    return False

#######################################################
# 4-Point Connectivity Wavefront Planner Algorithm
#######################################################
def waveFront():
    global goal_cell, planned_path
    getNeighbors()
    findCost(goal_cell, 2)
    planned_path = planPath()


#######################################################
# Main loop:
#######################################################
while robot.step(timestep) != -1:
    waveFront()                       # path planning algorithm
    if runPlan(planned_path): break   # path success, done

# Clean up code
setSpeedsRPS(0,0)
print(' \n')
print('GOAL !!!')
print(' \n')