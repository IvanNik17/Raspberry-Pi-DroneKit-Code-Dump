import h5py

def saveToFile(fileName, datasetName, dataArray):
    fileName = fileName + '.hdf5'
    f = h5py.File(fileName,'w')
    f.create_dataset(datasetName, data = dataArray)
    f.close()

def loadFromFile(fileName, datasetName):
    fileName = fileName + '.hdf5'
    f = h5py.File(fileName,'r')
    dset = f[datasetName]

    loadedData = dset[...]

    return loadedData
