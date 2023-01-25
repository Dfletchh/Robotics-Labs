"""lab1_task1 controller"""

# This criterion is linked to a Learning Outcome Task 1 Rectangle Traversal
# 1. Robot travels the rectangle (10 points) 
# 2. Robot travels the rectangle at given height “H” and width “W” values (10 points) 
# 3. Robot travels at the specified “V” velocity (10 points) 
# 4. Program computes and prints correct stopping time “T” (10 points) 
# 5. Output message if robot cannot complete the motion (5 points) 

# VIDEO TASK:
# 1) Record a video preforming the task with H=10 in, W=20 in, and V=5in/sec. Answer the following:
#   - Is this motion possible?
#   - How did you calculate T?
# 2) Record a video preforming the task with H=15 in, W=10 in, and V=10in/sec. Answer the following:
#   - Is this motion possible?

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
HEIGHT = 10
WIDTH = 20
V = 5
# HEIGHT = 15
# WIDTH = 10
# V = 10
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

# set the speed of the motors in inches per second (IPS)
def setSpeedsIPS(ipsLeft, ipsRight):
    global left_velocity
    global right_velocity
    
    # convert IPS to RPS
    # V = WR => W = V / R
    left_velocity = ipsLeft / WHEEL_RADIUS
    right_velocity = ipsRight / WHEEL_RADIUS

# robot moves in a straight line at a linear velocity of “V” inches 
# per second for a distance of “X” inches
def straightlineMotionVX(V, X):
    global distance_traveled
    global curr_ticks
    TARGET_DISTANCE = X * 0.0254

    setSpeedsIPS(V, V)

    if distance_traveled < TARGET_DISTANCE:
        curr_ticks = ( getCounts()[0] + getCounts()[1] ) / 2.0
        distance_traveled = ( curr_ticks / TICKS_PER_ROTATION ) * WHEEL_CIRCUMFERENCE
    else:
        setSpeedsRPS(0, 0)

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

# compute turn 
rotation_turn_time = np.pi / 2.33 # pi / 2 is a 90 degree turn

# compute duration on sides
# inches divided by in/s leaves seconds
height_duration = HEIGHT / V
width_duration = WIDTH / V

current_duration = width_duration
start_time = robot.getTime()

# total time to make turn
turn_start_time = start_time + current_duration
turn_end_time = turn_start_time + rotation_turn_time

# drive time
total_time = (4 * rotation_turn_time) + (2 * height_duration) + (2 * width_duration)
instance_time = 0

#* Work
while robot.step(timestep) != -1:

    current_time = robot.getTime()

    setSpeedsIPS(V, V) # set global speeds

    # turn in this interval
    if turn_start_time < current_time < turn_end_time:
        left_velocity = -V
        right_velocity = V
    
    # if turn happend reset interval and change sides
    elif current_time > turn_end_time:
        if current_duration % width_duration:
            current_duration = width_duration
        else:
            current_duration = height_duration
        turn_start_time = current_time + current_duration
        turn_end_time = turn_start_time + rotation_turn_time
    
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