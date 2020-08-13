import csv
import numpy

def radiiImport():
    with open('allRadii.csv',"r") as f:
        reader = csv.reader(f)
        csvList = list(reader)


    largeRadii = [i[0:2] for i in csvList]
    smallRadii = [i[2:4] for i in csvList]

    largeRadii = numpy.array(largeRadii)
    smallRadii = numpy.array(smallRadii)

    largeRadii = largeRadii.astype(numpy.float)
    smallRadii = smallRadii.astype(numpy.float)
    
    return largeRadii, smallRadii


def errorCorrectionImport():
    with open('lidarErrorCorrection.csv',"r") as f:
        reader = csv.reader(f)
        csvList = list(reader)


    distanceLidar = [i for i in csvList[0]]
    correctionsLidar = [i for i in csvList[1]]


    distCorrLidar = numpy.array([distanceLidar,correctionsLidar],numpy.float)



    return distCorrLidar
