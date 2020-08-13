# -*- coding: utf-8 -*-
"""
Created on Mon Sep 11 10:15:21 2017

@author: ivan
"""
import numpy
import math
  

  
        
def scanningPattern_leftToRight(rotationAngle, circleCent, radius, startAngle, endAngle, startingHeight, scanDiffHeight, deltaScannedPoints):
    x_list = []
    y_list = []
    z_list = []
    x = numpy.array(x_list)
    y = numpy.array(y_list)
    z = numpy.array(z_list)
    
    x_center = circleCent[0]
    y_center = circleCent[1]
    
    for h in range(0,3):
        
        for n in range(rotationAngle+ startAngle,rotationAngle+ endAngle,deltaScannedPoints):
        
            x = numpy.append(x,radius*math.cos(math.radians(n)) + x_center)
            y = numpy.append(y,radius*math.sin(math.radians(n)) + y_center)
            z = numpy.append(z,startingHeight)
        startingHeight += scanDiffHeight
        deltaScannedPoints = -deltaScannedPoints
        temp = startAngle + deltaScannedPoints
        startAngle = endAngle + deltaScannedPoints
        endAngle = temp
        
    
    
    return x,y,z


def scanningPattern_downToUp(rotationAngle,circleCent, radius, startAngle, endAngle, startingHeight, scanDiffHeight, deltaScannedPoints):
    x_list = []
    y_list = []
    z_list = []
    x = numpy.array(x_list)
    y = numpy.array(y_list)
    z = numpy.array(z_list)
    
    x_center = circleCent[0]
    y_center = circleCent[1]
    
    for n in range(rotationAngle+ startAngle,rotationAngle+ endAngle,deltaScannedPoints):
        
        for h in range(0,3):
            x = numpy.append(x,radius*math.cos(math.radians(n)) + x_center)
            y = numpy.append(y,radius*math.sin(math.radians(n)) + y_center)
            z = numpy.append(z,startingHeight)
            startingHeight += scanDiffHeight
        scanDiffHeight = -scanDiffHeight
        startingHeight +=scanDiffHeight
    
    
        
    return x,y,z
    

def scanningPattern_fullBladeScan(rotationDir, circleCent, radius, endAngle,  startingX, startingY, heightTop, heightBottom, heightScanDelta, deltaScannedPoints):

    x_list = []
    y_list = []
    z_list = []
    x = numpy.array(x_list)
    y = numpy.array(y_list)
    z = numpy.array(z_list)
    
    x_center = circleCent[0]



#    heightTop = 10000
#    heightBottom = 1000
#    heightScanDelta = 500
    
#    startingX = 2000
#    startingY = 0
    
    for u in range(heightBottom, heightTop, heightScanDelta):
        x = numpy.append(x, startingX)
        y = numpy.append(y, startingY)
        z = numpy.append(z,u)
#        startingHeight+=heightScanDelta
    
    startingAngle = int(math.degrees(math.acos((startingX - x_center)/radius)))
    print(startingAngle)
    centerNewX = startingX - radius * math.cos(math.radians(startingAngle))
    centerNewY = startingY - radius * math.sin(math.radians(startingAngle))
    
    if rotationDir is 'r':
        for n in range(startingAngle,endAngle + startingAngle+10, deltaScannedPoints):
            x = numpy.append(x,radius*math.cos(math.radians(n)) + centerNewX)
            y = numpy.append(y,radius*math.sin(math.radians(n)) + centerNewY)
            z = numpy.append(z,u)
    elif rotationDir is 'l':
        for n in range(startingAngle + 360,endAngle + startingAngle-10, -deltaScannedPoints):
            x = numpy.append(x,radius*math.cos(math.radians(n)) + centerNewX)
            y = numpy.append(y,radius*math.sin(math.radians(n)) + centerNewY)
            z = numpy.append(z,u)
    
    endingX = x[-1]
    endingY = y[-1]
        
    for d in range( heightTop - heightScanDelta, heightBottom - heightScanDelta, -heightScanDelta):
        x = numpy.append(x, endingX)
        y = numpy.append(y, endingY)
        z = numpy.append(z,d)
#        startingHeight-=heightScanDelta   
    
       

    return x,y,z
            
if __name__ == '__main__': 
    ax = initializeFigurePlot()
#    x,y,z = scanningPattern_leftToRight(180, [0,0], 2000, 0, 190, 0, 400, 10)
#    plot3D_animation(x,y,z,2000,2000,2000)
    x,y,z = scanningPattern_fullBladeScan('l',[0,0], 2000, 180,  2000, 0, 10000, 1000, 500, 10)

    drawBlade(ax, 10000, 110, 660, 0, 0)
    
#    for j in range(1,10):
        
#    x,y,z = scanningPattern_downToUp(180, [0,0], 2000, 0, 190, 1000, 400, 10)
    
    plot3D_animation(ax,x,y,z,2000,2000,10000)

#    plot3D_animation(x,y,z,2000,2000,10000)
    

