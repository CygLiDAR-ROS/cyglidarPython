import ctypes
import platform
import serial
import time
import cv2
import numpy as np


RUN_2DMode = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x01, 0x00, 0x03]
RUN_3DMode = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x08, 0x00, 0x0A]
RUN_DualMode = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x07, 0x00, 0x05]
COMMAND_STOP = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x02, 0x00, 0x00]
dataLength3D = 14400
dataLength2D = 322
normalizeDistanceLimit = 4080


def makeCtypesPacket(pythonPacket):
    return ((ctypes.c_uint8 * len(pythonPacket))(*pythonPacket),len(pythonPacket))

def Visualize(resultlist, dataLength):
    if dataLength == dataLength3D:
        distanceData = Get3DDistanceDataFromReceivedData(resultlist)
        image = DistanceDataToNormalizedNumpyArray(distanceData)
        image = np.array(image, dtype=np.uint8)
        image = image.reshape(60, 160)
        image = cv2.resize(image, dsize=(480, 180), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('test', image)
        cv2.waitKey(1)
    #elif dataLength == dataLength2D:
        # type your code
        
def Get3DDistanceDataFromReceivedData(receivedData):
    global dataLength3D,normalizeDistanceLimit
    index = 0
    distanceData = [0 for i in range(int(dataLength3D / 3 * 2))]
    for i in range(0, dataLength3D-2, 3):
        pixelFirst = receivedData[i] << 4 | receivedData[i+1] >> 4
        pixelSecond = (receivedData[i+1] & 0xf) << 8 | receivedData[i+2]
        if pixelFirst > normalizeDistanceLimit:
            pixelFirst = normalizeDistanceLimit
        if pixelSecond > normalizeDistanceLimit:
            pixelSecond = normalizeDistanceLimit
        distanceData[index] = pixelFirst
        index += 1
        distanceData[index] = pixelSecond
        index += 1
    return distanceData

def DistanceDataToNormalizedNumpyArray(distanceData):
    global normalizeDistanceLimit
    result = np.array(distanceData)
    result = result / normalizeDistanceLimit * 255
    return result

current_system = platform.system()
if current_system == 'Windows':
    #path = 'C:\\Users\\cygbot\\cygparserDll.dll'
    path=''
    c_module = ctypes.windll.LoadLibrary(path)
elif current_system == 'Linux':
    #path = '/home/cygbot/cygparser.so'
    path = ''
    c_module = ctypes.cdll.LoadLibrary(path)
else:
    raise OSError()




getPayloadSize = c_module.getPayloadSize
getParserPassed = c_module.getParserPassed
Parser = c_module.Parser


Parser.argtypes = (ctypes.POINTER(ctypes.c_uint8),ctypes.c_uint16)
Parser.restype = ctypes.POINTER(ctypes.c_uint8 * 15000)

getPayloadSize.argtypes = None
getPayloadSize.restype = ctypes.c_uint16

getParserPassed.argtypes = None
getParserPassed.restype = ctypes.c_uint8

ser = serial.Serial(  # port open
    #port="/dev/ttyUSB0", # <- USB connection
    #'/dev/ttyAMA1',# <- GPIO connection
     "COM17", #<- Windows PC
    #baudrate=3000000,
    baudrate=250000,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
if __name__ == "__main__":
    #runMode = RUN_2DMode
    runMode = RUN_3DMode
    #runMode = RUN_DualMode
    ser.write(runMode)
    
    print("send : ", runMode)
    while True:
        try:
            read_lines = ser.readline()
            read_lines_ctypes = makeCtypesPacket([x for x in read_lines])
            parserResult = Parser(read_lines_ctypes[0],read_lines_ctypes[1])
            
            resultlist=[]
            
            if getParserPassed() == 1:
                print('parser passed')
                resultlist = [x for x in parserResult.contents]
                Visualize(resultlist,getPayloadSize())
        except KeyboardInterrupt:
            ser.write(COMMAND_STOP)
            ser.close()
        
