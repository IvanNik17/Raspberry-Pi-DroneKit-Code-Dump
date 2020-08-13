# -*- coding: utf-8 -*-
"""
Created on Wed Aug  9 13:44:09 2017

@author: ivan
"""
import scanseSweep as ss
from sweeppy import Sweep
import numpy as np
import math
import matplotlib.pyplot as plt

from checkUSBs import selectSerialPort


def circularMean(angles,weight):
    
    sinAngles = []
    cosAngles = []
    
    for i in range(0,len(weight)):
        sinAngles.append( math.sin(math.radians(angles[i])))
        cosAngles.append( math.cos(math.radians(angles[i])))
    
    meanSin = np.dot(weight,sinAngles)
    meanCos = np.dot(weight,cosAngles)
    
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
    
    return np.column_stack((xC,yC))
    

    
def getInputAndVisualize(angleDistance,angleDistanceBad):
    
#    Not used here- only if a weighted mean is needed
    weight = np.ones(len(angleDistance))/len(angleDistance)
    meanAngle = circularMean(angleDistance[:,0],weight)    
    
    # meanDist = np.average(angleDistance[:,1], weights=weight)
    meanDist = 0;
#    Circular to Cartesian coordinates
    lidarPoint = circle2cart_drone([0,0],meanAngle, meanDist)
    frontPoint = [[0.0,0.0], circle2cart_drone([0,10000],meanAngle, meanDist)]
    
    environmentPoints = circle2cart_points(lidarPoint,angleDistance, 0)
    environmentPointsBad = circle2cart_points(lidarPoint,angleDistanceBad, 0)
    
#    Visualize
    plt.ion()

    # Add drone locations
    plt.scatter(lidarPoint[0],lidarPoint[1],marker='o', c='r')

    # Lidar Front
    plt.plot(frontPoint[:][0], frontPoint[:][1], 'g-')

    # Add the threshhold circles
    circle2 = plt.Circle((0,0), distanceMax, color='r', fill=False)
    plt.gca().add_artist(circle2)

    plt.scatter(environmentPoints[:,0],environmentPoints[:,1],marker='o', c='b', s=0.5)

    plt.scatter(environmentPointsBad[:,0],environmentPointsBad[:,1],marker='o', c='r', s=0.5)
    # plt.axes().set_aspect('equal', 'datalim')
    plt.axes().set_xlim([-4000,4000])
    plt.axes().set_ylim([-4000,4000])
    plt.show()
    plt.pause(0.05)
    plt.gcf().clear()
    
def useSweep(comPort, motorSpeed, sampleRate):
    # Connect to the Sweep Lidar
    print('Connecting to the Sweep sensor on COM port \"{}\"'.format(comPort))
    with Sweep(comPort) as sweep:
        print('Connected to the Sweep sensor\n')
        print('Changing settings to\nMotor Speed\t{} Hz\nSample Rate\{} Hz'.format(motorSpeed,sampleRate))
        ss.changeSettings(sweep, motorSpeed, sampleRate)
        print('Settings injected')
        print('Starting scanning')
        sweep.start_scanning()
        # Infinit loop, the loop i executed everytime a new scan is present from the LIDAR
        try:
            for scan in sweep.get_scans():
                lidarSamples = ss.scanToNpArray(scan)
                lidatSamplesFiltered, lidarSamplesBad = ss.filterSignalStrength(lidarSamples,signalStrengthValue)
                lidatSamplesFiltered, _ = ss.filterDistance(lidatSamplesFiltered,distanceMin,distanceMax)
                lidarSamplesBad = np.concatenate((lidarSamplesBad,_))
                print('{}\\{}\\{}'.format(len(lidarSamples),len(lidatSamplesFiltered),len(lidarSamplesBad)))
                getInputAndVisualize(lidatSamplesFiltered[:,:2],_)
                
        
        except KeyboardInterrupt:
            print("exit")
            plt.close()

        
signalStrengthValue = 60
distanceMin = 200
distanceMax = 40000

if __name__ == '__main__':

    portDictionary = selectSerialPort()
    useSweep(portDictionary['4'],5,500)
