from MotorFunctions import left_velocity, right_velocity


# reset the number of ticks counted to zero.
def resetCounts():
    global left_ticks
    global right_ticks

    left_ticks = 0
    right_ticks = 0


# return the left and right tick counts since the last call to resetCounts
def getCounts():
    global left_ticks
    global right_ticks
    
    left_ticks = leftposition_sensor.getValue()
    right_ticks = rightposition_sensor.getValue()
    wheel_tick_counts = ( left_ticks, right_ticks )
    # wheel_tick_counts = (leftposition_sensor.getValue(), rightposition_sensor.getValue())
    return wheel_tick_counts


# returns the instantaneous left and right wheel speeds (measured in 
# revolutions per second)
def getSpeeds():
    global left_velocity
    global right_velocity
    speeds = (left_velocity, right_velocity)
    return speeds

# contains all code necessary for initialization.
def initEncoders():
    resetCounts()
