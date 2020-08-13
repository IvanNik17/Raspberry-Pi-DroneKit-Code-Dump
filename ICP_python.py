import numpy as np
##import scipy.stats
##from pylab import *
##import matplotlib.pyplot as plt
##import scipy.io
import math
import time

t = time.time()

 
def T( x, T0, T1, k=1.0 ):
    # apply an affine transformation to `x`
    y = x * T0.T
    y[:,0] += T1[0,0]
    y[:,1] += T1[1,0]
    return y*k
 

def translate( X, Y, errorfct ):
    # translate to align the centers of mass
    mx = np.mean( X, axis=0 ).T
    my = np.mean( Y, axis=0 ).T
    translation = mx - my
    I = np.matrix( np.eye( 2 ) )
    Yp = T( Y, I, translation )
    return errorfct( X, Yp ), translation
 
def randrot( X, Y, errorfct, angle ):
    # perform a random rotation
#    theta = scipy.stats.uniform( 0.0, 2.0*np.pi ).rvs()
    theta = angle
    c = np.cos( theta )
    s = np.sin( theta )
    rotation = np.matrix( [[c,-s],[s,c]] )
    Z = np.matrix( np.zeros((2,1)) )
    Yp = T( Y, rotation, Z )
    return errorfct( X, Yp ), rotation
 
##def randscale( X, Y, errorfct ):
##    # perform a random scaling
##    k = scipy.stats.uniform( 0.5, 1.0 ).rvs()
##    scaling = k * np.matrix( np.eye(2) )
##    Z = np.matrix( np.zeros((2,1)) )
##    Yp = T( Y, scaling, Z )
##    return errorfct( X, Yp ), scaling

def SSE( X, Y ): 
    return np.sum( np.array( X - Y )**2.0 )
 
def ptSSE( pt, X ):
    '''
    Point-wise smallest squared error.
    This is the distance from the point `pt`
    to the closest point in `X`
    '''
    difference = pt - X
    # x and y columns
    xcol = np.ravel( difference[:,0] )
    ycol = np.ravel( difference[:,1] )
    # sum of the squared differences btwn `pt` and `X`
    sqr_difference = xcol**2.0 + ycol**2.0
    # nearest squared distance
    distance = np.min( sqr_difference )
    # index of the nearest point to `pt` in `X`
    nearest_pt = np.argmin( sqr_difference )
    return distance
 
def NSSE( X, Y ):
    '''
    Nearest sum squared error.
    This is the sum of the squares of the
    nearest differences between the points
    of `X` and the points of `Y`
    '''
    err = 0.0
    for x in X:
        err += ptSSE( x, Y )
    return err
    
def fit( X, Y, M, N, threshold=1e-5 ):
    T0 = list()
    T1 = list()
    errorfct = NSSE
    errors = list()
    errors.append( errorfct( X, Y ) )
    print (errors[-1])
    Yp = Y.copy()
    for iter in range( M ):
 
        err, translation = translate( X, Yp, errorfct )
        if err < threshold:
            break
        elif err < errors[-1]:
            errors.append( err )
            print (errors[-1])
            T1.append( translation )
            I = np.matrix( np.eye( 2 ) )
            Yp = T( Yp, I, T1[-1] )
             
        rot = [ randrot( X, Yp, errorfct, i ) for i in range(0, N,6 ) ] 
        rot.sort()
        err, rotation = rot[0]
        if err < threshold:
            break
        elif err < errors[-1]:
            errors.append( err )
            print (errors[-1])
            T0.append( rotation )
            Z = np.matrix( np.zeros((2,1)) )
            Yp = T( Yp, T0[-1], Z )
            
             
#        scale = [ randscale( X, Yp, errorfct ) for i in range( N ) ]
#        scale.sort()
#        err, scaling = scale[0]
#        if err < threshold:
#            break
#        elif err < errors[-1]:
#            errors.append( err )
#            print (errors[-1])
#            T0.append( scaling )
#            Z = np.matrix( np.zeros((2,1)) )
#            Yp = T( Yp, T0[-1], Z )
    finalRot = T0[0]
    angleRot= math.degrees(math.asin(finalRot[1,0]))
    return Yp,angleRot   
    
# Example   
#Yp,angle = fit( testPoints , ellipsePointsPos, M=30, N=360, errorfct=NSSE, threshold=1e-3 )


elapsed = time.time() - t
print(elapsed)
