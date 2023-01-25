"""lab2_task1 controller."""

""" 
Inverse Kinematics:

In this task, the TA will give you the initial pose “C”, final pose “F”, and a constant velocity
“V”. The program needs to compute a path that reaches “F” from “C” by assigning values larger
than 0 to “D1”, “D2”, “R1”, “R2”. No additional segments or partial circles should be added to
the trajectory. The robot must follow the computed path, and the program should print the values
for “D1”, “D2”, “R1”, “R2” and intermediate poses (“P1”, “P2”, “P3”, “P4”).

Implement the following function:
    inverseKinematics(C, F, V) - main task function making the robot move following the 
    trajectory shown in Figure 5 at a constant linear velocity of “V” inches per second
    starting from “C” = “P0” and ending at “F” = “P5”.

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
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI
TICKS_PER_ROTATION = 52
MAX_SPEED = 6.28
#######################################################
# Arguements
#######################################################
# F1 = (5, -5, -PI)
F1 = (15, -2.072, -PI/4)
C = (0, 0, 0)
V = 3

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
        x_new = X + VL * T * np.sin(THETA)
        y_new = Y + VL * T * np.cos(THETA)

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
# main task function making the robot move following the
# trajectory shown in Figure 5 at a constant linear velocity of 
# “V” IPS starting from “C” = “P0” and ending at “F” = “P4”
#######################################################
def inverseKinematics(C, F, V):
    # start at C go to F = (5, -5, -PI) && (15, -2.072, -PI/4)
    # Given Linear V = 3

    #######################################################
    #! Radius to P1 
    #######################################################   
    pCount = 1
    R1 = 2 # Calculate the change in angle to get R

    # Prompt Radius
    print("R[1]: ", R1)

    # Drive
    degrees = getIMUDegrees(False, F[2])
    leftover = degrees - 90
    theta_remaining = math.radians(leftover)
    theta = -PI / 2
    circle(abs(theta), V, -R1)

    # Calculate params to find pose
    T1 = circleTime(theta, R1, V)
    w = V / R1   
    vl = (V - WHEEL_D_MID * w)
    vr = (V + WHEEL_D_MID * w)
    P1 = calcPose(C, vl, vr, T1)

    # Prompt Pose   
    print(f"P[{pCount}]: ", P1)
    pCount+=1

    #######################################################
    #! Distance to P2
    ####################################################### 
    D1 = abs(F[0])   # X distance to get in the neighborhood
 
    # X correction
    if ( (F[2] != 0) and (abs(F[2]) != PI) ):
        D1 *= 1.19

    # Prompt Distance
    print("D[1]: ", D1)

    # Drive
    line(V, D1)

    # Calculate params to find pose
    T2 = D1 / V      
    P2 = calcPose(P1, V, V, T2)

    # Prompt Pose    
    print(f"P[{pCount}]: ", P2)
    pCount+=1

    #######################################################
    #! Radius to P3
    ####################################################### 

    # Find difference of Current and Final Theta
    target = getIMUDegrees(False, F[2])     # Get final pose in degrees
    current = getIMUDegrees(False, P2[2])   # Get current pose in degrees
    degrees = abs(target - current)         # Based on provided path, target will always be the larger
    theta = math.radians(degrees)

    # Get final radius
    R2 = 2

    # Prompt Radius
    print("R[2]: ", R2)

    # Turn to Final pose angle
    circle(theta_remaining, V, -R2)

    # Calculate params to find pose
    T3 = circleTime(-theta, R2, V)
    w = V / R2     
    vl = (V - WHEEL_D_MID * w)
    vr = (V + WHEEL_D_MID * w)
    P3 = calcPose(P2, vl, vr, T3)

    # Prompt Pose   
    print(f"P[{pCount}]: ", P3)
    pCount+=1

    #######################################################
    #! Distance to P4 
    ####################################################### 
    # Distance formula to for Current XY to Final XY
    D2 = math.sqrt( ( F[0] - P3[0] )**2 + ( F[1] - P3[1] )**2 )   # Remaining distance

    # Distance correction
    if ( (F[2] != 0) and (abs(F[2]) != PI) ):
        D2 *= .67

    T4 = D2 / V

    # Prompt Distance
    print("D[2]: ", D2)

    # Drive
    line(V, D2)

    # Calculate Pose
    P4 = calcPose(P3, V, V, T4)

    # Prompt Pose
    print(f"P[{pCount}]: ", P4)

#######################################################
# Straight Line Functoin 
#######################################################
def line(V, D):
    global dCount

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
    T = (D / V)
    TARGET_DURATION = T + robot.getTime()

    # Drive to pose
    while robot.step(timestep) != -1:
        if (TARGET_DURATION < robot.getTime()):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            return 

#######################################################
# Circle Function
#######################################################
def circle(theta, V, R):
    global rCount

    # Get Distance to turn
    error = 0.232
    D = theta * (abs(R) + WHEEL_D_MID * error)
        
    # get omega
    W = V/R
    PHI_L, PHI_R = getSpeedsVW(V, W)

    # Wheel velocity range check
    if abs(PHI_L) > MAX_SPEED or abs(PHI_R) > MAX_SPEED:
        print("Speed is beyond robots capabilities")
        return 
    else:
        leftMotor.setVelocity(PHI_L)
        rightMotor.setVelocity(PHI_R)
    
    # Setup drive interval 
    start_position = rightposition_sensor.getValue()

    while robot.step(timestep) != -1:
        # Checks if wheel distance is larger than D
        if WHEEL_RADIUS * abs(rightposition_sensor.getValue() - start_position) >= D:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break

#######################################################
# Positive angular velocities makes the robot spin counterclockwise
#######################################################
def getSpeedsVW(V, W):
    # Given V = wR AND v_l = w(R + d/2) AND phi = V / r
    # Flip sign for rotation direction
    return (V - WHEEL_D_MID * W) / WHEEL_RADIUS, (V + WHEEL_D_MID * W) / WHEEL_RADIUS

#######################################################
# Calculates curve time to drive
#######################################################
def circleTime(theta, R, V):
    D = theta * R   # S = aR
    T = D / V            # Time
    return abs(T)

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

    inverseKinematics(C, F1, V)
    break

# Enter here exit cleanup code.