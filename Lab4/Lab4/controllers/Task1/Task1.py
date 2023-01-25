from controller import Robot
# import numpy as it may be used in future labs
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
TARGET = 5
CYLINDER_OFFSET = 4
TARGET += CYLINDER_OFFSET
Kp = 1



#######################################################
# Proportionally corrects speeds over max
#######################################################
def speedCorrection(phi_l, phi_r):
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
# Rotate until object is in cameras field of view
#######################################################
def locateObject():
    while robot.step(timestep) != -1:
        recognition_arr = camera.getRecognitionObjects()
        setSpeedsRPS(2,-2)

        # if object found
        if (len(recognition_arr) > 0):
            targetXY = getObjectXY(recognition_arr[0])
            X, Y = targetXY
        
            # centers robot on object
            if (targetXY) and ( (-.5<=X<=.5) or (-.5<=Y<=.5) ):
                setSpeedsRPS(0,0)
                break

    return [X, Y, recognition_arr[0].getOrientation()]

#######################################################
# Gradually slows to a stop
#######################################################
def pidStop(targetDistance, Kp):
    # stops at 5 inches (targetDistance)
    x, y = getObjectXY(camera.getRecognitionObjects()[0])
    cur_dist = max(x, y)
    error = cur_dist - targetDistance

    # As error goes to zero slow down proportionally
    if(error < targetDistance):
        v = error * Kp           # PID velocity 
        if (error < 0):
            setSpeedsIPS(v, v)   # reverse
            if ( targetDistance *.99 <= cur_dist <= targetDistance*1.01):
                setSpeedsIPS(0, 0)  # stop
                # print(cur_dist)
                return True
        setSpeedsIPS(v, v)          # slow down

    return False

#######################################################
# Gets camera distance in inches converted from meters
#######################################################
def getObjectXY(object):
    return [round(object.getPosition()[0]*39.3701, 4), round(object.getPosition()[1]*39.3701, 4)]

#######################################################
# Gets robots Pose in inches converted from meters
#######################################################
def getRobotsXY():
    return [round(imu.getRollPitchYaw()[0]*39.3701, 4), round(imu.getRollPitchYaw()[1]*39.3701, 4)]

#######################################################
# IMU value in degrees corresponds to current Pose
#######################################################
def getDegrees(theta):
    if theta < 0:
        theta = theta + ( 2 * PI )     # negative correction

    degrees = math.degrees(theta)      # convert
    return round(degrees, 4)

#######################################################
# Sets motor velocities to radians/sec 
#######################################################
def setSpeedsRPS(phi_l, phi_r):
    # speed too high correct
    if abs(phi_l) > MAX_SPEED or abs(phi_r) > MAX_SPEED:
        # print("Correcting Speed")
        phi_l, phi_r = speedCorrection(phi_l, phi_r)

    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

#######################################################
# Sets motor velocities to radians/sec 
#######################################################
def setSpeedsIPS(vl, vr):
    phi_l = vl/WHEEL_RADIUS
    phi_r = vr/WHEEL_RADIUS    

    if abs(phi_l) > MAX_SPEED or abs(phi_r) > MAX_SPEED:
        # print("Correcting Speed")
        phi_l, phi_r = speedCorrection(phi_l, phi_r)

    leftMotor.setVelocity(phi_l)
    rightMotor.setVelocity(phi_r)

#######################################################
# Main Algorithm
#######################################################
def Motion_To_Goal(distance, orientation):
    setSpeedsRPS(MAX_SPEED,MAX_SPEED)

    while robot.step(timestep) != -1:
        # object moved
        if len(camera.getRecognitionObjects()) == 0:
            # print("lost object...")
            break

        # Returns true when Target distance reached
        if (pidStop(TARGET, Kp)):
            return True
    return False

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

    # turn toward yellow object
    x, y, orientation = locateObject()

    # distance to drive toward object
    distance = max(x, y)

    if Motion_To_Goal(distance, orientation):
        print("Goal Achived!")
        break

