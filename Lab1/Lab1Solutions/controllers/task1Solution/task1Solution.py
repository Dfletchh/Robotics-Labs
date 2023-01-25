
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math

# Robot Deminsions in inch
wheel_radius = 1.6/2
axel_length = 2.28

# Rectangle Parameters
H = 10 # in
W = 20 # in
V = 5 # in/sec

#######################################################
# General driving forward a distance of D function.
# will update the pose of the robot based on D, and the
# current heading of the robot.
#######################################################            
def driveD(robot,D,V):
    
    start_position = leftposition_sensor.getValue()
    
    # Calculates velocity of each motor and the robot
    phi = V / wheel_radius                # rad/sec

    # Calculates Time need to move a distance D
    T   = D/V               # sec

    # Sets motor speeds and sets start time
    t_start=robot.getTime()
    leftMotor.setVelocity(phi)
    rightMotor.setVelocity(phi)

    while robot.step(timestep) != -1:
        # Checks if wheel distance is larger than D
        if wheel_radius*abs(leftposition_sensor.getValue() - start_position) >= D-0.01:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
   
#######################################################
# General circular motion function that drives the robot.
# around a circle with radius R
# R < 0 counter clockwise
# R > 0 clockwise
#######################################################            
def circleR(robot,R,V,direction='right'):
    
    # Determines direction and sets proper speeds
    omega = V/abs(R)
    if R < 0 :
        sign = -1
    else:
        sign = 1
    
    # Right and Clockwise
    if direction == 'right' and sign > 0:
        vl = omega*(abs(R) + sign*(axel_length/2))
        vr = omega*(abs(R) - sign*(axel_length/2))
        D = 2*math.pi*(abs(R) + sign*(axel_length/2))
    # Right and Counter Clockwise
    elif direction == 'right' and sign < 0:
        vl = -omega*(abs(R) - sign*(axel_length/2))
        vr = -omega*(abs(R) + sign*(axel_length/2))
        D = 2*math.pi*(abs(R) - sign*(axel_length/2))
    # Left and Clockwise
    elif direction == 'left' and sign > 0:
        vl = -omega*(abs(R) - sign*(axel_length/2))
        vr = -omega*(abs(R) + sign*(axel_length/2))
        D = 2*math.pi*(abs(R) - sign*(axel_length/2))
    # Left and Counter Clockwise
    elif direction == 'left' and sign < 0:
        vl = omega*(abs(R) + sign*(axel_length/2))
        vr = omega*(abs(R) - sign*(axel_length/2))
        D = 2*math.pi*(abs(R) + sign*(axel_length/2))
    
    
    phi_l = vl/wheel_radius
    phi_r = vr/wheel_radius

    # Checks to see if speed is to high 
    if abs(phi_l) > leftMotor.getMaxVelocity() or abs(phi_r) > rightMotor.getMaxVelocity():
        print("Speed is too great for robot")
        return

    start_position = leftposition_sensor.getValue()

    # Sets motor speeds and sets start time
    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

    while robot.step(timestep) != -1:
        # Checks if wheel distance is larger than D
        if wheel_radius*abs(leftposition_sensor.getValue() - start_position) >= D-0.02:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break

#######################################################
# General function to rotate the robot by degree
#######################################################       
def rotate(robot,degree):
    
    # Determines Rotation and sets proper speeds
    if degree < 0 :
        sign = -1
    else:
        sign = 1
    phi = sign*1

    start_position = leftposition_sensor.getValue()

    leftMotor.setVelocity(phi)
    rightMotor.setVelocity(-phi)

    # Arch length of wheel needs to travel
    D = math.radians(abs(degree))*(axel_length/2)
    
    while robot.step(timestep) != -1:
        
        # Checks to see if left wheel traveled past distance of arch lendth D
        if wheel_radius*(abs(leftposition_sensor.getValue() - start_position)) >= D -.01:
            
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
        

#######################################################
# Cleans the IMU readings so that the are in degrees and in the
# range of [0,359]
#######################################################
def imu_cleaner(imu_reading):
    rad_out = imu_reading
    if rad_out < 0:
        rad_out = rad_out + 2*math.pi
    return math.degrees(rad_out)

#######################################################
# Calculated the time needed to complete the Rectangle
# motion
#######################################################
def calcTimeOfMotion(motion_type,paramaters):
    if motion_type == 'rectangle':
        if len(paramaters) != 3:
            print("Incorrect Paramaters")
            return
        V = paramaters[0]
        H = paramaters[1]
        W = paramaters[2]
        T_H = H/V
        T_W = W/V 
        # Hardcoded speed for rotations
        omega = 2*1*wheel_radius / axel_length
        T_omega = 2*math.pi / (omega)
        T = 2*T_H + 2*T_W + T_omega
        return T

    elif motion_type == 'circle':
        if len(paramaters) != 2:
            print("Incorrect Paramaters")
            return
        V = paramaters[0]
        R = paramaters[1]
        D = 2*math.pi*R
        T = D/V
        return T


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
    
    # Checks to see if speed is to high
    phi = V/wheel_radius
    if phi > leftMotor.getMaxVelocity():
        print("Speed to fast for robot")
        break

    # Calculate Time
    T = calcTimeOfMotion('rectangle',[V,H,W])
    print("Robot will take " + str(T) + " sec")
    start_time = robot.getTime()

    # Preforms the Rectangle Motion
    driveD(robot,H/2,V)
    rotate(robot,90)
    driveD(robot,W,V)
    rotate(robot,90)
    driveD(robot,H,V)
    rotate(robot,90)
    driveD(robot,W,V)
    rotate(robot,90)
    driveD(robot,H/2,V)
    print("Robot took " + str(robot.getTime() - start_time) + " sec to complete the motion")
    break
    
    

# Enter here exit cleanup code.
