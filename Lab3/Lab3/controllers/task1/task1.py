"""lab3_task1 controller"""

""" 
    Webots robot should stop at the exact 10-inch mark from the end wall using the front
    sensor and should keep a distance between 3 and 7 inches from side walls using the two
    side sensors. See Fig 2 (right). Task should run for no more than 30 sec
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
WHEEL_DIAMETER = 1.6 # 1.61417
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_D = 2.28 # 2.04724 
WHEEL_D_MID = WHEEL_D / 2
MAX_SPEED = 6.28

#######################################################
# Arguements
#######################################################
DISTANCE = 3
TARGET = 10
KP1 = .1  #(0.1, 0.5, 1.0, 2.0, 2.5, 5.0)
KP2 = 1

v = 5 # speed


#######################################################
# Set to Max and Proportional decrease to max  
#######################################################
def speed_correction(phi_l, phi_r):
    # set left to max
    if abs(phi_l) > abs(phi_r):
        phi_r = MAX_SPEED * (phi_r/abs(phi_l))   # proportional change to right
        phi_l = MAX_SPEED * (phi_l/phi_l)        # set to max maintain sign
    # set right to max
    elif abs(phi_l) < abs(phi_r):
        phi_l = MAX_SPEED * (phi_l/abs(phi_r))   # proportional change to right
        phi_r = MAX_SPEED * (phi_r/phi_r)        # set to max maintain sign
    # go straight at max
    else:
        phi_l = phi_r = MAX_SPEED

    return phi_l, phi_r

#######################################################
# Sets motor velocities to radians/sec 
#######################################################
def setSpeedsIPS(vl, vr):
    phi_l = vl/WHEEL_RADIUS
    phi_r = vr/WHEEL_RADIUS    

    if abs(phi_l) > MAX_SPEED or abs(phi_r) > MAX_SPEED:
        # print("Correcting Speed")
        phi_l, phi_r = speed_correction(phi_l, phi_r)

    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

#######################################################
# Gets distance sensors in inches converted from meters
#######################################################
def getDistanceSensors():
    return [leftDistanceSensor.getValue()*39.3701, rightDistanceSensor.getValue()*39.3701, frontDistanceSensor.getValue()*39.3701]

#######################################################
# Gradually slows to a stop
#######################################################
def pidStop(targetDistance, Kp):
    # Front sensor stops at 10 (targetDistance)
    front_sensor = getDistanceSensors()[2]
    error = front_sensor - targetDistance

    # As error goes to zero slow down proportionally
    if(error < targetDistance):
        v = error * Kp           # PID velocity 
        print(front_sensor)
        if (error < 0):
            setSpeedsIPS(v, v)   # reverse
            if ( targetDistance *.999 <= front_sensor <= targetDistance*1.001):
                setSpeedsIPS(0, 0)  # stop
                return True
        setSpeedsIPS(v, v)          # slow down

    return False

#######################################################
# Wall follow algorithm
#######################################################
def wallFollow(wall, distance, kp):
    # Side sensors stay 3 - 7 off walls

    if(wall == 'l'):
        error = getDistanceSensors()[0] - distance
        if(error<0):
            setSpeedsIPS(v - abs(error)*kp, v)  # turn away from right wall
        else:
            setSpeedsIPS(v, v - abs(error)*kp)  # turn towards right wall
    elif(wall == 'r'):
        error = getDistanceSensors()[1] - distance
        if(error<0):
            setSpeedsIPS(v, v - abs(error)*kp)  # turn away from left wall
        else:
            setSpeedsIPS(v - abs(error)*kp, v)  # turn towards left wall
    
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
    sensors = getDistanceSensors()

    # print("Front DS Reading: " + str(frontDistanceSensor.getValue() * 39.3701))

    # Returns true when Target distance reached
    error = sensors[2] - TARGET
    if(error < TARGET):
        if (pidStop(TARGET, KP2)):
            print("Goal Achived...")
            break

    # Picks closest wall
    else:
        if (sensors[0] < sensors[1]):
            wallFollow('l', DISTANCE, KP1)      
        else:
            wallFollow('r', DISTANCE, KP1)

