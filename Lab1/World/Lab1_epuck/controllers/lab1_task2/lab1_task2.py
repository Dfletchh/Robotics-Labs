"""lab1_task2 controller"""

# Task 2: Double Circle Traversal
# 1. Robot travels the circles (10 points) 
# 2. Robot travels the circles at given “R1” and “R2” values (10 points) 
# 3. Robot travels at the specified “V” velocity (10 points) 
# 4. Program computes and prints correct stopping time “T” (10 points) 
# 5. Output message if robot cannot complete the motion (5 points)

# VIDEO TASK:
# 1) Record a video preforming the task with R1=5 in, W=10 in, and V=5in/sec. Answer the following:
#   - Is this motion possible?
#   - How did you calculate T?
# 2) Record a video preforming the task with R2=0 in, W=10 in, and V=2 in/sec. Answer the following:
#   - Is this motion possible? 
#   - What Motion is preformed when the Radius of the Circle is 0?

from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import numpy as np


### physical constraints ###
WHEEL_DIAMETER = 1.6
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * np.pi
TICKS_PER_ROTATION = 52
WHEEL_D = 2.28
WHEEL_D_MID = WHEEL_D / 2
MAX_SPEED = 6.28

### global procedures ###
left_velocity = 0
right_velocity = 0
left_ticks = 0
right_ticks = 0
curr_ticks = 0
prev_ticks = 0
distance_traveled = 0
current_duration = 0
start_time = 0


################################################################################
############################## Given ###########################################
################################################################################
# R = 5
# W = 10
# V = 5
R = 0
W = 10
V = 2
################################################################################
################################################################################

# return the left and right tick counts since the last call to resetCounts
def getCounts():
    global left_ticks
    global right_ticks
    
    left_ticks = leftposition_sensor.getValue()
    right_ticks = rightposition_sensor.getValue()
    wheel_tick_counts = ( left_ticks, right_ticks )
    return wheel_tick_counts

# set the speed of the motors in RPS
def setSpeedsRPS (rpsLeft, rpsRight):    
    global left_velocity
    global right_velocity

    left_velocity = rpsLeft
    right_velocity = rpsRight

# robot moves in a circle of radius “R” at a constant linear velocity 
# of “V” inches per second
def circleMotionRV(R, V):
    # Known: R, V, d_mid
    # Find: vl, vr

    # solving for vr fisrt
    if (R > 0):
        vr = (V * (R - WHEEL_D_MID)) / R
    else:
        vr = 0
    # sub vr to get vl
    vl = 2 * V - vr
    setSpeedsRPS(vl, vr)

def setSpeedsVW(V, W):
    global left_velocity
    global right_velocity
    # Known: V, W "Omega", d_mid
    # Find: vl, vr

    # find left & right wheel velocity    
    # solving for vr fisrt
    if (R > 0):
        right_velocity = (V * (R - WHEEL_D_MID)) / R
    else:
        right_velocity = 0
    # sub vr to get vl
    left_velocity = 2 * V - right_velocity

################################################################################
################################################################################

# create the Robot instance.
robot = Robot()

# get the time step of the current world in msec.
timestep = int(robot.getBasicTimeStep())

# Motor Instance 
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# getting the position sensors
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

################################################################################
################################################################################

start_time = robot.getTime()

# V = distance / time
slip_error = 1.10
circ = (R * 2) * (2 * np.pi)
if (circ == 0):
    total_time = W / V * slip_error
else:
    total_time = (circ / V) * slip_error

#* Work
while robot.step(timestep) != -1:

    current_time = robot.getTime()

    # circleMotionRV(R, V)
    setSpeedsVW(V, W)
    
    if (current_time - start_time >= total_time):
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        print("Stopping time: " + str(total_time))
        break
    elif (left_velocity < MAX_SPEED and right_velocity < MAX_SPEED):
        leftMotor.setVelocity(left_velocity)
        rightMotor.setVelocity(right_velocity)
    else:
        print("Maneuver not possible!")
        break