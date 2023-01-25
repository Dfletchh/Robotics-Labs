from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import numpy as np
from EncoderFunctions import getCounts, resetCounts
from cmath import inf, pi

### physical constraints ###
WHEEL_DIAMETER = 1.6
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * pi
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
distance_traveled = 0
current_duration = 0
start_time = 0



###################################################################################################
###################################################################################################

# robot moves in a straight line at a linear velocity of “V” IPS
def straightlineMotionV(V):
    setSpeedsIPS(V,V)


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


# robot moves in a straight line at a linear velocity of “V” inches 
# per second for “T” seconds.
def straightlineMotionVT(V, T):
    global start_time
    global current_duration
    TARGET_DURATION = T + start_time

    setSpeedsIPS(V, V)

    if (current_duration < TARGET_DURATION):
        current_duration = robot.getTime() - start_time
    else:
        setSpeedsRPS(0, 0)

###################################################################################################
###################################################################################################

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


# Robot moves in a circle of radius “R” at a constant linear velocity
# of “V” inches per second for a distance of “X” inches
def circleMotionRVX(R, V, X):
    global distance_traveled
    global curr_ticks
    TARGET_DISTANCE = X
    
    circleMotionRV(R, V)

    if (distance_traveled < TARGET_DISTANCE):
        curr_ticks = ( getCounts()[0] + getCounts()[1] ) / 2.0
        distance_traveled = ( curr_ticks / TICKS_PER_ROTATION ) * WHEEL_CIRCUMFERENCE
    else:
        setSpeedsRPS(0, 0)


# robot moves in a circle of radius “R” at a constant linear velocity 
# of “V” inches per second for “T” seconds.
def circleMotionRVT(R, V, T):
    global start_time
    global current_duration
    TARGET_DURATION = T + start_time

    circleMotionRV(R, V)

    if (current_duration < TARGET_DURATION):
        current_duration = robot.getTime() - start_time
    else:
        setSpeedsRPS(0, 0)


###################################################################################################
###################################################################################################

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


# sets the speed of the robot, in which the robot will move with the given
# linear velocity ‘V’ (in inches per second) and with an angular 
# velocity ‘Omega - W’ (in radians per second). Positive angular velocities 
# should make the robot spin counterclockwise
def setSpeedsVW(V, W):
    global left_velocity
    global right_velocity
    # Known: V, W "Omega", d_mid
    # Find: vl, vr

    # solve for radious
    R = V / W
    # find left & right wheel velocity    
    # solving for vr fisrt
    if (R > 0):
        right_velocity = (V * (R - WHEEL_D_MID)) / R
    else:
        right_velocity = 0
    # sub vr to get vl
    left_velocity = 2 * V - right_velocity