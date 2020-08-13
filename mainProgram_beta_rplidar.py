
## Default libraries
import numpy
import serial
import math
import json
import timeit

## Additionally installed libraries
from rplidar import RPLidar
from rplidar import RPLidarException

## Custom libraries
from importRadii import radiiImport, errorCorrectionImport
from pythonMatlabForRP import mainAlgorithm, calculateAngles,calculateAnglesPCA,getProperRadii,findNewPoint
from outputs_fromDrone import InitializeConnectDrone, ReturnDroneAngle
from imuFunctions import getDataFromIMU, calibrateIMU
from drone_variousMovements import fixDistanceToBlade, movementToPosition, rotateDrone
from input_forDrone import droneArm, droneTakeOff, condition_yaw
from IOFunctions import waitForResponse, sendDataToGround
from LineFilterImport import lineFilter, sunFilterV2
from checkUSBs import selectSerialPort
from CreateMovementPattern import scanningPattern_downToUp, scanningPattern_leftToRight


def mainAlgorithm_beta_rpLidar(serial_IO):

    ## Variables

    tryCalculate = False
    angleOffsetMeasured = 0
    armedAngle = 0
    isArmed = False
    yawAngle = 0
    heightMeasured = 1100
    prevHeightMeasured = 0
    distance_min_threshold = 200
    distance_max_threshold = 800
    angleDistanceR = []

    dronePPrev = []
    countSend = 1
    stopAttempts = False
    attempt = 0

    nextX = 0
    nextY = 0

    sideToStart = -1

    streamData = False

    distThreshA = 0.1 #threshold for line filter for the angle in sin/cos space
    distThreshD = 10 #threhsold for line filter for distances

    isScanning = False
    scanPattern = -1
    
    ## Drone movement variables

    goToNewLocation = True

    safeDistance = 2000

    ## Save angle/distances
    saveLidar = False
    saveAngleDist = numpy.array([[0,0,0]])
    countSaves = 1
    
    print("Initializing serial ports...")

    portDictionary = selectSerialPort()

    ## Initialize serial for Lidar
    serial_lidar = RPLidar(portDictionary['2'])

    ## Initliaze serial for Arduino and IMU
    serial_IMU = serial.Serial(portDictionary['3'],57600)
    arduinoInitial = numpy.zeros(8)


    print("Importing radii...")
    ## Import radii and correction parameters
    largeRArray, smallRArray = radiiImport()

    distCorrLidar = errorCorrectionImport()

    print("Starting Lidar...")
    ## Start Lidar and get's info
    while True:
        try:
            
            info = serial_lidar.get_health()
            
            
            break
        except RPLidarException:
            
            print("Lidar error retry")
        finally:
            serial_lidar.stop()

    print(info)


    ##print("Starting IMU calibration...")
    #### Calibrate IMU
    ##try:
    ##    
    ##    calibrateIMU(serial_IMU)
    ##    
    ##except KeyboardInterrupt:
    ##    print("Stopped calibration")


    ## Initialize connection to drone
    ##vehicle = InitializeConnectDrone()


    ##print("Arm the drone...")
    ## Arm drone
    ##droneArm(vehicle)


    ##print("Drone take off...")
    ## Drone take off - test take of to 10 meters
    ##droneTakeOff(10, vehicle)

    ## Wait until start command is send
    countZ = 100
    radiusAnglesEllipse, ellipsePointsPos, bigSmallRadius = getProperRadii(heightMeasured,largeRArray, smallRArray, angleOffsetMeasured)
    counterS = 0    


    print("Starting!")

    while attempt < 20 and stopAttempts is not True:
        
        try:
            serial_lidar.stop()
            for i, scan in enumerate(serial_lidar.iter_scans(1000,5,distance_min_threshold,distance_max_threshold)):
                attempt = 0
                
                if len(scan) <= 1:
                    continue
                
                start_time = timeit.default_timer()
                
    ## Angle/Distance numpy array                
                angleDistanceR = numpy.array(scan)
##                angleDistanceR[:,0] *=  -1
##                print(len(angleDistanceR))
    ## Line filter for filtering the sunlight
                problemIndices = sunFilterV2(angleDistanceR[:,0],angleDistanceR[:,1], distThreshA, distThreshD)
                
                angleDistanceR = numpy.delete(angleDistanceR, problemIndices, 0)
    ## Save angle/distances
                if saveLidar is True:
                    
                    saveNum = numpy.ones([len(angleDistanceR),1]) * countSaves
                    angleDistanceR_added = numpy.append(angleDistanceR, saveNum, axis=1)
                    
                    saveAngleDist = numpy.concatenate([saveAngleDist, angleDistanceR_added], axis = 0)
                    countSaves += 1


                    
##                print(len(angleDistanceR))
##                print('--------')
    ## Get output from Arduino - IMU Pitch/Roll/Yaw calibrations and values and height from ultrasound
                
                arduinoOutput = getDataFromIMU(serial_IMU,arduinoInitial)
                arduinoInitial = arduinoOutput
                yawAngle = arduinoOutput[4]
    ##                heightMeasured = arduinoOutput[7]
                
    ## Get output from drone - Yaw value and height from height meter
    ##                yawAngle, test = ReturnDroneAngle(vehicle)
    ##                yawAngle = yawAngle+180
    ## Does the ellipse need to be recalculated
                if (math.fabs(heightMeasured - prevHeightMeasured) > 100 or prevHeightMeasured == 0) and isArmed == True:
                    print("HERE")
                    radiusAnglesEllipse, ellipsePointsPos, bigSmallRadius = getProperRadii(heightMeasured,largeRArray, smallRArray, angleOffsetMeasured)
                

    ## Calculate the angle between the blade and the Lidar
                if len(angleDistanceR[:,0]) >= 8 and tryCalculate is True:
                    angleOffsetMeasured = calculateAnglesPCA(angleDistanceR)
                    armedAngle = yawAngle

                    radiusAnglesEllipse, ellipsePointsPos, bigSmallRadius = getProperRadii(heightMeasured,largeRArray, smallRArray, angleOffsetMeasured)
                    tryCalculate = False

                    print(angleOffsetMeasured)
                    
                elif len(angleDistanceR[:,0]) < 8 and tryCalculate is True:
                    tryCalculate = False


    ## Main calculation algorithm                
                droneP, bladePointsP, meanAngle_compensated, distDroneToBlade =  mainAlgorithm (angleDistanceR,yawAngle,heightMeasured,ellipsePointsPos,radiusAnglesEllipse,angleOffsetMeasured, armedAngle, largeRArray, smallRArray, isArmed,dronePPrev,distCorrLidar)
                dronePPrev = droneP
##                print(droneP)
    ##                print(angleDistanceR)
                
    ## Get data from ground station and process it
                pointsAll_ahr = ""
                
                str_return = waitForResponse(serial_IO)
##                print(str_return)
                if str_return == 'ArmP':
                    isArmed = True
                    sideToStart = 0
                    print("Arming for Presure side")
                elif str_return == 'ArmS':
                    isArmed = True
                    sideToStart = 1
                    print("Arming for Suction side")
                elif str_return == 'Set':
                    tryCalculate = True
                elif str_return == 'Save':
##                    saveLidar = not saveLidar
                    
                    strName = "position_suction_" + str(counterS)  +".txt"
                    numpy.savetxt(strName, droneP, "%5.3f")
                    print("Saved")
                    counterS +=1
                    
                elif str_return == 'Stop':
                    print("Stopped")
                    stopAttempts = True
                    break
                elif str_return == 'LtoR':
                    scanPattern = 1
                elif str_return == 'DtoU':
                    scanPattern = 2
                    
                elif str_return == 'Scan':
                    print("Start Scanning")
                    isScanning = True
                    
                elif str_return == 'Send':
                    ## Send data to ground station
                    
    ##                    testingScenario REMOVE
    ##                    randomValues = numpy.random.rand(40,2)*1000
    ##                    countZ +=10
                    
    ##           bladePointsP , heightMeasured        
                    pointsAll = numpy.concatenate([numpy.array([droneP]),bladePointsP])
                    pointsAll_ah = numpy.concatenate([numpy.array([[angleOffsetMeasured, heightMeasured]]), pointsAll])
                    pointsAll_ahr = numpy.concatenate([numpy.array([bigSmallRadius]), pointsAll_ah])

                    if isArmed:
                        
                        pointsAll_ahrn = numpy.concatenate([pointsAll_ahr,numpy.array([[nextX, nextY]])])
                        sendDataToGround(serial_IO,pointsAll_ahrn)
                        
                    else:
                        
                        sendDataToGround(serial_IO,pointsAll_ahr)


                    
                if isArmed and scanPattern is not -1:
                    angleOffsetMeasured_int = int(angleOffsetMeasured)
                    if scanPattern is 1:
                        if sideToStart is 0:
                            xP,yP,zP = scanningPattern_leftToRight(angleOffsetMeasured_int, [0,0], 2000, 180, -10,heightMeasured, 400, -10)
                        elif sideToStart is 1:
                            xP,yP,zP = scanningPattern_leftToRight(angleOffsetMeasured_int, [0,0], 2000, 0, 190,heightMeasured, 400, 10)
                    if scanPattern is 2:
                        if sideToStart is 0:
                            xP,yP,zP = scanningPattern_downToUp(angleOffsetMeasured_int, [0,0], 2000, 180, -10,heightMeasured, 400, -10)
                        elif sideToStart is 1:
                            xP,yP,zP = scanningPattern_downToUp(angleOffsetMeasured_int, [0,0], 2000, 0, 190,heightMeasured, 400, 10)
                    

    #### Send command to drone
                if isArmed and isScanning:
                    
                    if goToNewLocation and distDroneToBlade is not 0:
    ##                        meanAngleC, meanDistC = calculateMeans(angleDistanceR)
                        

                        nextX, nextY = findNewPoint(bladePointsP, meanAngle_compensated, distDroneToBlade, sideToStart)  
                        goToNewLocation = False


                    elif not goToNewLocation and distDroneToBlade is not 0:
                        nextPoint = numpy.array((nextX,nextY))
                        distBetweenCurrAndNext = numpy.linalg.norm(nextPoint - droneP)
                        print(distBetweenCurrAndNext)
                        if distBetweenCurrAndNext < 50:
                            goToNewLocation = True

                        
                    
    ##                        fixDistanceToBlade(safeDistance, distDroneToBlade, vehicle)
    ##                        movementToPosition(nextX, nextY, droneP, vehicle)
    ##                        rotateDrone(0, False, vehicle)
    ####                    condition_yaw(meanAngle_compensated)
    ##                    condition_yaw(angleOffsetMeasured)

                if isArmed is True:
                    prevHeightMeasured = heightMeasured

                elapsed = timeit.default_timer() - start_time
                print(elapsed)
                

        except KeyboardInterrupt:
            print("Force stopped")
            break
        except RPLidarException:
            print(RPLidarException.message)
    ##            attempt = attempt + 1
            continue


    if countSaves is not 1:
        strName = 'angleDist_RPLIDAR.txt'
        numpy.savetxt(strName, saveAngleDist, "%5.3f")
        print("Saved")

    serial_lidar.stop()
    serial_lidar.stop_motor()
    serial_lidar.disconnect()
    ##vehicle.close()

    serial_IMU.close()
    
