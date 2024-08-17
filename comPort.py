import time
import serial

#Creating an instance of serial, passing the COM port number as an argument
ser = serial.Serial("COM3")

# set Baud rate to 115200
ser.baudrate = 115200  

# Number of data bits = 8
ser.bytesize = 8     

# No parity
ser.parity   ='N'   

# Number of Stop bits = 1
ser.stopbits = 1     

#Open COM port
ser.open

#Delay time for handshake
time.sleep(3)
