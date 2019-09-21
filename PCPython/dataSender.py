import serial
import os

filenameSource = "plik_pc.wav"
filenameResult = "result.wav"
portionSize = 60

fs = open(filenameSource,"rb")
fr = open(filenameResult,"wb")

fstemp = open(filenameSource, "rb")
startIndex = fstemp.read(1024).rfind(b'data')+8
fstemp.close()

print('approximate file size:')
filesize=os.stat(filenameSource).st_size
print(round(filesize/1024), 'KB')

print('Opening COM port')
ser = serial.Serial(
    port='COM3',
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    dsrdtr=False
)
if (ser.isOpen() == False):
        ser.open()
ser.flushInput()
ser.flushOutput()
print('COM Port opened')


portion = fs.read(startIndex)
fr.write(portion)

print('Start of mixing')
portion = fs.read(portionSize)
while portion:

    size = portion.__len__();
    ser.write(portion)
    result = ser.read(portionSize)
    #print(portion)
    #print(result)
    fr.write(result[0:size])
    portion = fs.read(portionSize)

print("File mixed on stm32f4 board!")
