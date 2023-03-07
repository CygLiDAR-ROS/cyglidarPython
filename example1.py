import time
import serial

RUN_2D = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x01, 0x00, 0x03]
RUN_3D = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x08, 0x00, 0x0A]
RUN_DUAL = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x07, 0x00, 0x05]
COMMAND_STOP = [0x5A, 0x77, 0xFF, 0x02,0x00,0x02,0x00,0x00]

HEADER1, HEADER2, HEADER3, LENGTH_LSB, LENGTH_MSB, PAYLOAD_HEADER, PAYLOAD_DATA, CHECKSUM = 0, 1, 2, 3, 4, 5, 6, 7
POS_CYGBOT_HEADER, POS_DEVICE, POS_ID, POS_LENGTH_1, POS_LENGTH_2, POS_PAYLOAD_HEADER = 0, 1, 2, 3, 4, 5
PAYLOAD_POS_HEADER, PAYLOAD_POS_DATA = 0, 1
NORMAL_MODE = 0x5A
PRODUCT_CODE = 0x77
DEFAULT_ID = 0xFF
HEADER_LENGTH_SIZE = 5

buffercounter, CPC, lengthLSB, lengthMSB, data_length = 0, 0, 0, 0, 0
step = HEADER1
receivedData = []


def Parser(data):
    global step, CPC, lengthLSB, lengthMSB, data_length, buffercounter, receivedData
    if step != CHECKSUM:  # CPC is a variable for storing checksum. If it is not a checksum part, XOR operation is performed on each data and then stored.
        CPC = CPC ^ data

    if step == HEADER1 and data == NORMAL_MODE:
        step = HEADER2
        
    elif step == HEADER2 and data == PRODUCT_CODE:
        step = HEADER3
        
    elif step == HEADER3 and data == DEFAULT_ID:
        step = LENGTH_LSB
        CPC = 0
        
    elif step == LENGTH_LSB:
        step = LENGTH_MSB
        lengthLSB = data
        
    elif step == LENGTH_MSB:
        step = PAYLOAD_HEADER
        lengthMSB = data
        data_length = ((lengthMSB << 8) & 0xff00) | (lengthLSB & 0x00ff)
        
    elif step == PAYLOAD_HEADER:
        step = PAYLOAD_DATA
        if data_length == 1:
            step = CHECKSUM
        buffercounter = 0
        receivedData = []
        
    elif step == PAYLOAD_DATA:
        receivedData.append(data)
        buffercounter = buffercounter+1
        if buffercounter >= data_length - 1:
            step = CHECKSUM
            
    elif step == CHECKSUM:
        step = HEADER1
        
        if CPC == data:
            return True
    else:
        step = HEADER1
        return False
    
ser = serial.Serial(  # Port settings
    port= '/dev/ttyAMA1', 
    baudrate=3000000, # recommend 250,000
    #baudrate=250000,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
ser.write(RUN_2D) # Mode settings
print("send : ", RUN_2D)
time.sleep(1)

while True:
    try:
        readdata = ser.readline()
        for i in range(len(readdata)):
            if Parser(readdata[i]):
                print(len(receivedData))
                
                
    except KeyboardInterrupt:
        ser.write(COMMAND_STOP)
        ser.close()
