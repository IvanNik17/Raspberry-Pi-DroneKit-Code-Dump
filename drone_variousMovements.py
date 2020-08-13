import math
import numpy

from input_forDrone import send_ned_velocity_heading, condition_yaw, send_ned_velocity_global

def fixDistanceToBlade(safeDistance, distDtoB,vehicle):

    diffDist = safeDistance - distDtoB

    maxDiff = 50

## forward - x = 2, backwards - x = -2, 2 is just how fast the velocity added should be
## right - y = 2, left - y = -2
## down - z = 2, up - z = -2    

    if math.fabs(diffDist) > maxDiff:

        if diffDist > 0:
            send_ned_velocity_heading(-2, 0, 0, vehicle)
            print('backwards')
        elif diffDist < 0:
            send_ned_velocity_heading(2, 0, 0, vehicle)
            print('forward')
            
##        reversedAngle = (90 - meanAngle + 180) % 360
##        reversedDist = math.fabs(diffDist - maxDiff)
##
##        moveBackX = dronePos[0] + reversedDist * math.acos(math.radians(reversedAngle))
##        moveBackY = dronePos[1] + reversedDist * math.asin(math.radians(reversedAngle))
##
##        reversedDistX = moveBackX - dronePos[0]
##        reversedDistY = moveBackY - dronePos[1]
##
##        reversedPos = numpy.array((reversedDistX, reversedDistY))
##
##        currPos = numpy.array((dronePos[0],dronePos[1]))
##        distBetweenCurrAndFinal = numpy.linalg.norm(reversedPos - currPos)
##
##        return distBetweenCurrAndFinal

def moveDroneInLocalCoord(vehicle, movementVector, speed):

##    right = +y, left = -y, forward = +x, backwards = -x
    movementVector = numpy.multiply(movementVector,speed)
    send_ned_velocity_heading(movementVector[0], movementVector[1], movementVector[2], vehicle)


def movementToDistancePosition(distX, distY, speed):
    stopper = False

    ## x is +forward, -backward, y is +right, -left 


    signX = numpy.sign(distX)
    signY = numpy.sign(distY)
    counterX = abs(distX)
    counterY = abs(distY)
    while stopper is False:

##      speed =  0.5
        
        if counterX > 0:
            xMove = 1
            counterX-=speed 
        else:
            xMove = 0

        if counterY > 0:
            yMove = 1
            counterY-=speed
        else:
            yMove = 0

        if yMove == 0 and xMove == 0:
            stopper = True
            
        moveDroneInLocalCoord(vehicle, [signX*xMove,signY*yMove,0], [speed,speed,speed])
        

def movementToPosition(posX, posY, currentPos, vehicle):
    stopper = False
    
    ## x is +forward, -backward, y is +right, -left 
    distX = posX - currentPos[0]
    distY = posY - currentPos[1]

    signX = numpy.sign(distX)
    signY = numpy.sign(distY)

    moveDroneInLocalCoord(vehicle, [signX,signY,0], [1,1,1])

def rotateDrone(angle, isRelative, vehicle):
    condition_yaw(45,vehicle,relative=isRelative)
    send_ned_velocity_global(0,0,0,vehicle)
            
##def moveToNewLocation(vehicle,currPosition, nextPosition, stopThresh):
##
####    THIS NEEDS FIXING !!!!!
##    if abs(currPosition[0] - nextPosition[0]) < stopThresh and abs(currPosition[1] - nextPosition[1]) < stopThresh:
##        
##        return -1
##
##    else:
##        movementVector = [0,0]
##
##        if abs(currPosition[0] - nextPosition[0]) < stopThresh:
##            
##            if currPosition[0] - nextPosition[0] > 0:
##                
##                movementVector[1] = 2
##                print("go right")
##            else:
##                
##                movementVector[1] = -2
##                print("go left")
##                
##        if abs(currPosition[1] - nextPosition[1]) < stopThresh:
##
##            if currPosition[1] - nextPosition[1] > 0:
##                
##                movementVector[0] = -2
##                print("go backwards")
##            else:
##                
##                movementVector[0] = 2
##                print("go forwards")
##        print(movementVector)
##        send_ned_velocity_heading(movementVector[0], movementVector[1], 0, vehicle)
##
