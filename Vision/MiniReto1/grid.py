import numpy as np


nRows = 9
nCols = 6
worldPtsCur = np.zeros((nRows*nCols,3), np.float32)
worldPtsCur[:,:2] = np.mgrid[0:nRows,0:nCols].T.reshape(-1,2)
print(worldPtsCur)