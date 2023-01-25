"""lab2_task1 controller."""

""" 
Forward Kinematics:

In this task, starting at “C” with orientation north, the TA will give you a set of four left and right
wheel velocities (“VL1”, “VR1”, “VL2”, “VR2”, “VL3”, “VR3”, “VL4”, “VR4”) for certain amounts of
time (“T1”, “T2”, “T3”, “T4”). The robot needs to follow the precise path computed from these
values. The program should calculate and print the corresponding values for “D1”, “D2”, “R1”,
“R2”, as well as of all intermediate and final poses (“P1”, “P2”, “P3”, “P4”). During evaluation,
the TA will test different values of “VL1”, “VR1”, “VL2”, “VR2”, “VL3”, “VR3”, “VL4”, “VR4” and
“T1”, “T2”, “T3”, “T4”.

Implement the following function (called 4 times):
   forwardKinematics(P, VL, VR, T) - motion function making the robot move starting from
   current waypoint P with individual left and right velocities VL, VR, for time T.

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
WHEEL_DIAMETER = 1.61417 # 1.6
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_D = 2.04724 # 2.28
WHEEL_D_MID = WHEEL_D / 2
MAX_SPEED = 6.28
pCount = 0
rCount = 1
dCount = 1
#######################################################
# Arguements
#######################################################
Vl1 = 5
Vr1 = 3.143
Vl2 = 4
Vr2 = 4
Vl3 = 5
Vr3 = 3.143
Vl4 = 4
Vr4 = 4
T1 = 1.929
T2 = 3
T3 = 3.857
T4 = 3

#######################################################
# Kinematics to calculate next pose
#######################################################
def calcPose(P, VL, VR, T):

    X = P[0]
    Y = P[1]
    THETA = P[2]

    # Drive strainght line
    if (VL == VR):
        theta_new = THETA
        x_new = X + VL * T * np.cos(THETA)
        y_new = Y + VL * T * np.sin(THETA)

    # Drive a curve
    else:
        # Find the radius
        R = abs( ( WHEEL_D_MID * (VL + VR) ) / (VL - VR)  )

        # Find coords to curves center
        ICC_x = X - R * np.sin(THETA)
        ICC_y = Y + R * np.cos(THETA)

        # Find angular velocity
        W = (VR - VL) / WHEEL_D

        # Find the change in angle
        thetaC = W * T

        # Based on above calculate find change in x, y, theta
        x_new = (X-ICC_x) * np.cos(thetaC) - (Y-ICC_y) * np.sin(thetaC) + ICC_x 
        y_new = (X-ICC_x) * np.sin(thetaC) + (Y-ICC_y) * np.cos(thetaC) + ICC_y 
        theta_new = THETA + thetaC
    
    return x_new, y_new, theta_new

#######################################################
# Motion function making the robot move starting from current waypoint 
# P with individual left and right velocities VL, VR, for time T
#######################################################
def forwardKinematics(P, VL, VR, T):

    # find motion type
    if VL != VR: 
        circle(VL, VR, T)

    elif ( (VL != 0) and (VR == VL) ):
        line(VL, T)

    else: 
        print("Motion not possible...")

    return calcPose(P, VL, VR, T)

#######################################################
# IMU value in degrees corresponds to current Pose
#######################################################
def getIMUDegrees(flag=True, theta=0):
    global imu

    if flag:
        theta = imu.getRollPitchYaw()[2]   # current pose

    if theta < 0:
        theta = theta + ( 2 * PI )     # negative correction

    degrees = math.degrees(theta)      # convert
    return degrees

#######################################################
# Circle Function
#######################################################
def circle(VL, VR, T):
    global rCount

    # Calculate radius R
    R = abs( (WHEEL_D_MID * (VL + VR)) / (VL - VR) ) 
        
    # Wheel angular velocities
    PHI_L = VL / WHEEL_RADIUS
    PHI_R = VR / WHEEL_RADIUS

    # Wheel velocity range check
    if abs(PHI_L) > MAX_SPEED or abs(PHI_R) > MAX_SPEED:
        print("Speed is beyond robots capabilities")
        return 
    else:
        leftMotor.setVelocity(PHI_L)
        rightMotor.setVelocity(PHI_R)
    

    # Get Distance to turn
    error = 1.3
    D = (( (VL + VR) / 2 ) * T) * error

    # Setup drive interval 
    start_position = rightposition_sensor.getValue()

    while robot.step(timestep) != -1:
        # Checks if wheel distance greater then calculated distance
        if WHEEL_RADIUS * abs(rightposition_sensor.getValue() - start_position) >= D:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            print(f"R[{rCount}]: ", R)
            rCount+=1
            return

#######################################################
# Straight Line Functoin 
#######################################################
def line(V, T):
    global dCount

    # Calculate Distance
    D = V * T

    # Wheel angular velocities
    PHI = V / WHEEL_RADIUS

    # Wheel velocity range check
    if abs(PHI) > MAX_SPEED:
        print("Speed is beyond robots capabilities")
        return 
    else:
        leftMotor.setVelocity(PHI)
        rightMotor.setVelocity(PHI)
    
    # Setup drive interval
    TARGET_DURATION = T + robot.getTime()

    # # Drive to pose
    while robot.step(timestep) != -1:
        if (TARGET_DURATION < robot.getTime()):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            print(f"D[{dCount}]: ", D)
            dCount+=1
            return 

#######################################################
# Create the Robot instance.
#######################################################
robot = Robot()

#######################################################
# Get the time step of the current world.
#######################################################
timestep = int(robot.getBasicTimeStep())

#######################################################
# Setup
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#######################################################
# Getting the position sensors
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

#######################################################
# Main loop
#######################################################
while robot.step(timestep) != -1:

    P0 = (0, 0, 0)

    P1 = forwardKinematics(P0, Vl1, Vr1, T1)
    print("P[1]: ", P1)
    P2 = forwardKinematics(P1, Vl2, Vr2, T2)
    print("P[2]: ", P2)
    P3 = forwardKinematics(P2, Vl3, Vr3, T3)
    print("P[3]: ", P3)
    P4 = forwardKinematics(P3, Vl4, Vr4, T4)
    print("P[4]: ", P4)

    break