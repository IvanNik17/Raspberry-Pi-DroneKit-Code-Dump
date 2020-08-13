# -*- coding: utf-8 -*-
"""
Created on Wed Apr  5 08:20:05 2017

@author: ivan
"""

##import scipy.io
import math
import numpy
##from scipy.spatial.distance import pdist
##import matplotlib.pyplot as plt
from ICP_python import fit
from importRadii import radiiImport


##test = scipy.io.loadmat('C:/Users/ivan/Documents/MATLAB/testComparison/clausTest/bladePos_01.mat')
##angleDistance = test['angleDistance']
##dronePos = test['dronePos']
##
##allRadii = scipy.io.loadmat('C:/Users/ivan/Documents/MATLAB/3DReconstructionPipelineTest/fullHeightRadii_proper.mat')
##radiiLongList = allRadii['long_interp']
##radiiShortList = allRadii['short_interp']

#heightMeasured = 1100
#angleOffsetMeasured = 0
#
#isArmed = False
#armedAngle = 0
#
#
#
#calculateAngles = True


def testEllipse(distance1,distance2, orientation, center):
#    ellipseInfo = collections.namedtuple('ellipseInfo', ['radiusAnglesEllipse', 'ellipsePointsPos'])
    numberOfPoints = 360
    centerX = center[0]
    centerY = center[1]
    orientation = -orientation
    theta = numpy.linspace(0, 2*math.pi, numberOfPoints)
    orientation=orientation*math.pi/180
    
    radiusDist = []
    xx2 = []
    yy2 = []

    for i in range(0,numberOfPoints):
        xx = -(distance1/2) * math.sin(theta[i]) + centerX
        yy = -(distance2/2) * math.cos(theta[i]) + centerY
    
        xx2_temp = (xx-centerX)*math.cos(orientation) - (yy-centerY)*math.sin(orientation) + centerX
        yy2_temp = (xx-centerX)*math.sin(orientation) + (yy-centerY)*math.cos(orientation) + centerY

        xx2.append(xx2_temp)
        yy2.append(yy2_temp)
    
        radiusDist.append(math.sqrt(xx2_temp**2 + yy2_temp**2))


    degrees = range(0,numberOfPoints)
    
    radiusAngles = numpy.column_stack((radiusDist,degrees))
    ellipsePos = numpy.column_stack((xx2,yy2))

    return radiusAngles, ellipsePos


def cart2pol(x,y):
    rho = numpy.sqrt(x**2 + y**2)
    phi = numpy.arctan2(x,y)
    return (rho,phi)

def findNearest(array,value):
    idx = (numpy.abs(array - value)).argmin()
    return idx

def circularMean(angles,weight):
    
##    sinAngles = []
##    cosAngles = []
    
##    for i in range(0,len(weight)):
##        sinAngles.append( math.sin(math.radians(angles[i])))
##        cosAngles.append( math.cos(math.radians(angles[i])))

    sinAngles = numpy.sin(numpy.radians(angles))
    cosAngles = numpy.cos(numpy.radians(angles))
    
#    for i in range(0,len(weight)):
    meanSin = numpy.dot(weight,sinAngles)
    meanCos = numpy.dot(weight,cosAngles)

    
    if (meanSin > 0 and meanCos > 0):
        meanAngle = math.atan(meanSin/meanCos)
        
    elif (meanCos < 0):
        meanAngle = math.atan(meanSin/meanCos) + math.radians(180)
    elif (meanSin <0 and meanCos> 0):
        meanAngle = math.atan(meanSin/meanCos) + math.radians(360)
    
    return math.degrees(meanAngle)
    
def circle2cart_drone(center,angle, distance):
   
    xC = center[0] - distance * math.sin(math.radians(angle))
    yC = center[1] - distance * math.cos(math.radians(angle))

    return [xC,yC]


def circle2cart_points(circleCenter,measurement,addDegree):

    xC = []
    yC = []
    for i in range(0,len(measurement)):
        xC.append( circleCenter[0] + measurement[i,1] * math.sin(math.radians(measurement[i,0] + addDegree)) )
        yC.append( circleCenter[1] + measurement[i,1] * math.cos(math.radians(measurement[i,0] + addDegree)) )
    
    return numpy.column_stack((xC,yC))


def intersectionLineCurve(lineFirst, lineSecond, curvePoints):

    b = (lineSecond[1] - lineFirst[1]) / (lineSecond[0] - lineFirst[0]) # gradient
    a = lineFirst[1] - b * lineFirst[0] # intercept
    B = (a + curvePoints[:,0] * b) - curvePoints[:,1] # distance of y value from line
    ix = numpy.where(B[1:] * B[:-1] < 0)[0] # index of points where the next point is on the other side of the line
    d_ratio = B[ix] / (B[ix] - B[ix + 1]) # similar triangles work out crossing points
    cross_points = numpy.zeros((len(ix), 2)) # empty array for crossing points
    cross_points[:,0] = curvePoints[ix,0] + d_ratio * (curvePoints[ix+1,0] - curvePoints[ix,0]) # x crossings
    cross_points[:,1] = curvePoints[ix,1] + d_ratio * (curvePoints[ix+1,1] - curvePoints[ix,1]) # y crossings

    
    distToLidar_1 = numpy.linalg.norm(lineSecond - cross_points[0,:])
    distToLidar_2 = numpy.linalg.norm(lineSecond - cross_points[1,:])


    if(distToLidar_1 > distToLidar_2):
        outputInters = cross_points[1,:]
    else:
        outputInters = cross_points[0,:]
    
    return outputInters


def getProperRadii(pix_height, radiiLong, radiiShort, pix_angleOffset):
    
    allHeght_diff = []
    for j in range(0, len(radiiLong[:,1])):
        allHeght_diff.append(math.fabs(radiiLong[j,1] - pix_height))

    
    minIndex = allHeght_diff.index(min(allHeght_diff))
    radiusLong = radiiLong[minIndex,0]
    radiusShort = radiiShort[minIndex,0]

    radiusLong = radiusLong
    
    radiusAnglesEllipse, ellipsePointsPos = testEllipse(radiusShort,radiusLong, pix_angleOffset, [0,0])
    
    return radiusAnglesEllipse, ellipsePointsPos, [radiusLong, radiusShort]


def calculateAngles(angleDistance,heightMeasured, radiiLongList, radiiShortList):

##    weight = numpy.ones(len(angleDistance))/len(angleDistance)        
##    meanAngle = circularMean(angleDistance[:,0],weight)    
##    meanDist = sum(angleDistance[:,1])/len(angleDistance[:,1])

    meanAngle,meanDist = calculateMeans(angleDistance)

    dronePos = circle2cart_drone([0,0],meanAngle, meanDist)
    bladePointsPos = circle2cart_points(dronePos,angleDistance, 0)
    radiusAnglesEllipse, ellipsePointsPos = getProperRadii(heightMeasured, radiiLongList, radiiShortList, 0)
    
    ICP_points,angleOffsetMeasured = fit( bladePointsPos , ellipsePointsPos, M=30, N=360, threshold=1e-3 )
    #armedAngle = angX
    
    return angleOffsetMeasured

def calculateAnglesPCA(angleDistance):

##    weight = numpy.ones(len(angleDistance))/len(angleDistance)        
##    meanAngle = circularMean(angleDistance[:,0],weight)    
##    meanDist = sum(angleDistance[:,1])/len(angleDistance[:,1])

    meanAngle,meanDist = calculateMeans(angleDistance)

    dronePos = circle2cart_drone([0,0],meanAngle, meanDist)
    bladePointsPos = circle2cart_points(dronePos,angleDistance, 0)
	
    cov_mat = numpy.cov([bladePointsPos[:,0],bladePointsPos[:,1]])
    
    eig_val_cov, eig_vec_cov = numpy.linalg.eig(cov_mat)
    
    maxIndEigenval = numpy.argmax(eig_val_cov)
    maxEigenvec = eig_vec_cov[:,maxIndEigenval]
    
    angleOffsetMeasured = math.degrees(numpy.arctan2( maxEigenvec[1],maxEigenvec[0]))
##    normally this is 90 not  270
    angleOffsetMeasured = 90 - angleOffsetMeasured 
	
    #radiusAnglesEllipse, ellipsePointsPos = getProperRadii(heightMeasured, radiiLongList, radiiShortList, 0)
    
    #ICP_points,angleOffsetMeasured = fit( bladePointsPos , ellipsePointsPos, M=30, N=360, threshold=1e-3 )
    #armedAngle = angX
    
    return angleOffsetMeasured

def calculateMeans(angleDistList):
##    weight = numpy.ones(len(angleDistList))/len(angleDistList)
    dists = angleDistList[:,1]
    weight = dists/sum(dists)
    
    meanAngleCalc = circularMean(angleDistList[:,0],weight)    
##    meanDistCalc = sum(angleDistList[:,1])/len(angleDistList[:,1])
    meanDistCalc = numpy.average(angleDistList[:,1],weights= weight)

    return meanAngleCalc, meanDistCalc


def lowPass2DFilter(inputPos, outputPos, threshold):

##    if outputPos is None:
##        return inputPos
    alpha = 0.5
    if math.fabs(inputPos - outputPos) < threshold:
        outputPos = outputPos + alpha * (inputPos - outputPos)
    else:
        outputPos = inputPos
    
    

    return outputPos

def findNewPoint(bladePoints, averageAngle, averageDist, sideStart):
    averagePointPos = [sum(bladePoints[:,0])/len(bladePoints[:,0]),sum(bladePoints[:,1])/len(bladePoints[:,1])]
    x_center = averagePointPos[0]
    y_center = averagePointPos[1]
    
    r = averageDist
##  0 is Pressure side, 1 is Suction   
    if sideStart is 0:
        step = 25
    elif sideStart is 1:
        step = -25
    
##    if averageAngle>=0 and averageAngle < 180:
##        step = -35
##    elif averageAngle>= 180 and averageAngle < 360:
##        step = -35

    nextX = -r*math.sin(math.radians(int(averageAngle) + step)) + x_center
    nextY = -r*math.cos(math.radians(int(averageAngle) + step)) + y_center

##    nextPoint = numpy.array((nextX,nextY))

    return nextX, nextY
                       

def mainAlgorithm (angleDistance,angX,heightMeasured,ellipsePointsPos,radiusAnglesEllipse,angleOffsetMeasured, armedAngle, radiiLongList, radiiShortList, isArmed, prevDronePos,distCorrLidar):
    

    distDtoB = 0
    
    
    if isArmed is False:
    
##        meanAngle = circularMean(angleDistance[:,0],weight)    
##        meanDist = sum(angleDistance[:,1])/len(angleDistance[:,1])

        meanAngle,meanDist = calculateMeans(angleDistance)
        
##        print(angleDistance[:,0])
        dronePos = circle2cart_drone([0,0],meanAngle, meanDist)
        bladePointsPos = circle2cart_points(dronePos,angleDistance, 0)

##Here the code can be optimized by only calling getProperRadii when there is a change in the height and angleOffsetMeasured       
##        radiusAnglesEllipse, ellipsePointsPos = getProperRadii(heightMeasured, radiiLongList, radiiShortList, angleOffsetMeasured)
        
    else:
        compensateYaw = angX - armedAngle
        
        angleDistance[:,0] = angleDistance[:,0] + compensateYaw 


##        print(weight)
##        meanAngle = circularMean(angleDistance[:,0],weight)    
##        meanDist = sum(angleDistance[:,1])/len(angleDistance[:,1])

        meanAngle,meanDist = calculateMeans(angleDistance)
        
        dronePos = circle2cart_drone([0,0],meanAngle, meanDist)
        bladePointsPos = circle2cart_points(dronePos,angleDistance, 0)
        
        

        
##        radiusAnglesEllipse, ellipsePointsPos = getProperRadii(heightMeasured, radiiLongList, radiiShortList, angleOffsetMeasured)
        
        #Ellipse correction
        
        averagePointPos=[sum(bladePointsPos[:,0])/len(bladePointsPos[:,0]),sum(bladePointsPos[:,1])/len(bladePointsPos[:,1])]
##        distCentToPoint = numpy.array([ [0,0],  [averagePointPos[0],averagePointPos[1]] ])
##        distAveragePointToZero = pdist(distCentToPoint,'euclidean');
        pCenter=numpy.array((0,0))
        pAvg = numpy.array((averagePointPos[0],averagePointPos[1]))
        distAveragePointToZero = numpy.linalg.norm(pCenter-pAvg)
        
        intersectPointOnCurve = intersectionLineCurve([0,0], dronePos, ellipsePointsPos)

        ##    Find correction using angle correspondance    
##        radEll, anglesEll = cart2pol(ellipsePointsPos[:,0],ellipsePointsPos[:,1])
##        anglesEll = numpy.degrees(anglesEll)
##
##        anglesEll = anglesEll%360
##
##        indexClosest = findNearest(anglesEll,meanAngle)
##
##        correctionDist = radiusAnglesEllipse[indexClosest,0]

        ##        
        
        correctionDist = math.sqrt(intersectPointOnCurve[0]**2 + intersectPointOnCurve[1]**2)


##        Add additional lidar corrections
        
##        minIndex = numpy.argmin(numpy.absolute(distCorrLidar[0,:] - meanDist))
##
##        correctionValue = distCorrLidar[1,minIndex]
        

        newCenterPos = circle2cart_drone([0,0],meanAngle, correctionDist -distAveragePointToZero) ## +correctionValue
        dronePos = circle2cart_drone(newCenterPos,meanAngle, meanDist)
        bladePointsPos = circle2cart_points(dronePos,angleDistance, 0)

##Low-pass filtering of drone position    
##    threshold = 30
####    if prevDronePos is not None:
##    if prevDronePos:
##        dronePos[0] =  lowPass2DFilter(dronePos[0], prevDronePos[0], threshold)
##        dronePos[1] =  lowPass2DFilter(dronePos[1], prevDronePos[1], threshold)

        distDtoB = meanDist + correctionDist -distAveragePointToZero
        
    return  dronePos, bladePointsPos, meanAngle, distDtoB

     
    
    
    
##    #Plot stuff
##    plt.plot(ellipsePointsPos[:,0],ellipsePointsPos[:,1])   
##    plt.plot(dronePos[0],dronePos[1],'ro')
##    plt.plot(bladePointsPos[:,0],bladePointsPos[:,1],'ro')
##    plt.axes().set_aspect('equal', 'datalim')
    
    
##mainAlgorithm(angleDistance,0,0,0,0)
