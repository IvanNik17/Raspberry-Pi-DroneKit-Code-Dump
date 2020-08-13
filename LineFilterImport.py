# -*- coding: utf-8 -*-
"""
Created on Thu Jun 29 12:41:41 2017

@author: ivan
"""

import numpy

def difference_matrix(a):
    x = numpy.reshape(a, (len(a), 1))
    return x - x.transpose()


def sunFilterV2(angles,distances, distThreshA, distThreshD):
    
    x = angles.T
    
    sinAngles = numpy.sin(numpy.radians(x))
    cosAngles = numpy.cos(numpy.radians(x)) 
    
    
    
    diffMatSin = difference_matrix(sinAngles.T)
    diffMatSinAbs = numpy.absolute(diffMatSin)
    
    diffMatCos = difference_matrix(cosAngles.T)
    diffMatCosAbs = numpy.absolute(diffMatCos)
    
    diffMatSinAbs[numpy.logical_and(diffMatSinAbs > distThreshA, diffMatSinAbs != 0)] = -1
    diffMatCosAbs[numpy.logical_and(diffMatCosAbs > distThreshA, diffMatCosAbs != 0)] = -1
                
                
    occurancesInSin = (diffMatSinAbs == -1).sum(axis=1)
    occurancesInSin[occurancesInSin > numpy.average(occurancesInSin)] = -1
                     
    occurancesInCos = (diffMatCosAbs == -1).sum(axis=1)
    occurancesInCos[occurancesInCos > numpy.average(occurancesInCos)] = -1
                    
    diffMatDist = difference_matrix(distances.T)
    diffMatDistAbs = numpy.absolute(diffMatDist)       
    diffMatDistAbs[numpy.logical_and(diffMatDistAbs > distThreshD, diffMatDistAbs != 0)] = -1
    occurancesInDist = (diffMatDistAbs == -1).sum(axis=1)
    occurancesInDist[occurancesInDist > numpy.average(occurancesInDist)] = -1
                    
                  
    compareMat = numpy.zeros([numpy.size(occurancesInSin)])
    compareMat[numpy.logical_and(occurancesInSin == -1, occurancesInCos == -1, occurancesInDist == -1)] = 1
    
    problemIndices = numpy.where(compareMat==1)
    
    return problemIndices
    

def lineFilter(angles,distances, distThreshA, distThreshD):

    diffMatA = difference_matrix(angles.T)
    
    diffMatAAbs = numpy.absolute(numpy.triu(diffMatA))
    
    diffMatAAbs[numpy.logical_and(diffMatAAbs < distThreshA, diffMatAAbs != 0)] = -1
    
    
    diffMatD = difference_matrix(distances.T)
    
    diffMatDAbs = numpy.absolute(numpy.triu(diffMatD))
    
    diffMatDAbs[numpy.logical_and(diffMatDAbs > distThreshD, diffMatDAbs != 0)] = -1
    
    compareMat = numpy.zeros([numpy.size(diffMatDAbs,1), numpy.size(diffMatDAbs,1)])
    compareMat[numpy.logical_and(diffMatDAbs == -1, diffMatAAbs == -1)] = 1
    
    indecesArr = numpy.where(compareMat==1)
    
    problemIndices = numpy.unique( indecesArr )
    
    return problemIndices
