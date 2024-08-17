import threading
import time
import math
import numpy as np
from turtle import goto
import comPort

# Height of delta in between Pick and Drop
zSafe = -290#mm

# Height of delta when picking and droping 
zAction = -328#mm

# Home height of delta
zRetract= -250#mm

# Queue to store the target objects
object_queue = []


speed =0

# Conveyor speed for imaging
xSpeed = 71
ySpeed = 0

# Minimum range of motion for X axis
xMin = -100#mm

# Maximum range of motion for X axis
xMax = 100

# Minimum range of motion for Y axis
yMin = -100

# Maximum range of motion for Y axis
yMax = 100

# Location(coordinates) of the bins
bins = [[0,-150],[0,110],[-80,110]]

# Robot geometry
e = 40.0        #End Effector radius
f = 70.0        #Base radius
re = 322.0      #Forearm length
rf = 175.0      #Bicep length 
btf = 400.0     #Base to floor distance
s = 1000        #Steps per turn

# Trigonometric constants
sqrt3 = math.sqrt(3.0)
pi = math.pi #3.141592653     # PI
sin120 = sqrt3/2.0   
cos120 = -0.5        
tan60 = sqrt3
sin30 = 0.5
tan30 = 1.0/sqrt3

# To store the theta values of servo motors 
theta1_glb = 0
theta2_glb = 0
theta3_glb = 0

# To store the XYZ coordinates
x_glb = 0
y_glb = 0
z_glb = 0

# To store the step values in Binary 
binary1_glb = 0
binary2_glb = 0
binary3_glb = 0


# inverse kinematics
# helper functions, calculates angle theta1 (for YZ-pane)
def delta_calcAngleYZ(x0, y0, z0, t):
    y1 = -0.5 * 0.57735 * f  # f/2 * tan(30 deg)
    y0 -= 0.5 * 0.57735 * e  # shift center to edge
    # z = a + b*y
    a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1)/(2.0*z0)
    b = (y1-y0)/z0
    # discriminant
    d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf)
    if d < 0:
        return [1, 0]  # non-existing povar. return error, theta
    yj = (y1 - a*b - math.sqrt(d))/(b*b + 1)  # choosing outer povar
    zj = a + b*yj
    theta = math.atan(-zj/(y1 - yj)) * 180.0/math.pi + (180.0 if(yj>y1) else 0.0)
    return [0, theta]  # return error, theta


# inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
# returned {error code, theta1,theta2,theta3}
# Returns +ve angles for upward motion of the arm
# Returns -ve angles for downward motion of the arm
def delta_calcInverse(x0, y0, z0):
    theta1 = 0
    theta2 = 0
    theta3 = 0
    t=0
    status = delta_calcAngleYZ(x0, y0, z0, t)
    if status[0] == 0:
        theta1 = status[1]
        status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2)  # rotate coords to +120 deg
    if status[0] == 0:
        theta2 = status[1]
        status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3)  # rotate coords to -120 deg
    theta3 = status[1]
    return [status[0], theta1, theta2, theta3]

# To round off the value
def roundoff(x, y):
    z = math.pow(10, y)
    if y == 0:
          y = 3
    return round(x * z) / z

# To get the X,Y,Z cordinates given theta1,theta2,theta3
def test_fk():
    
    global theta1_glb
    global theta2_glb
    global theta3_glb

    theta1 = float(theta1_glb)
    theta2 = float(theta2_glb)
    theta3 = float(theta3_glb)

    # Call to get the  X,Y,Z cordinates given theta1,theta2,theta3
    results1 = delta_calcForward(theta1, theta2, theta3)

    global x_glb
    global y_glb
    global z_glb

    # Rounding off the result
    x_glb = roundoff(results1[1], 3)
    y_glb = roundoff(results1[2], 3)
    z_glb = roundoff(results1[3], 3)

    #XYZ coordinates
    print("x:" + str(x_glb))
    print("y:" + str(y_glb))
    print("z:" + str(z_glb))

# To get the theta1,theta2,theta3 given X,Y,Z coordinates
def test_ik(a,b,c):

    # Taking Y coordinate
    x = float(a)

    # Taking Y coordinate
    y = float(b)

    # Taking Z coordinate
    z = float(c)

    # Call to get the theta1,theta2,theta3 given X,Y,Z coordinates
    results2 = delta_calcInverse(x, y, z)
    
    global theta1_glb
    global theta2_glb
    global theta3_glb
    
    # Rounding off the result
    theta1_glb = roundoff(results2[1], 3)
    theta2_glb = roundoff(results2[2], 3)
    theta3_glb = roundoff(results2[3], 3)

    # Theta 1, Theta 2, Theta 3
    print("Motor 1 degrees:" + str(theta1_glb))
    print("Motor 2 degrees:" + str(theta2_glb))
    print("Motor 3 degrees:" + str(theta3_glb))

# To check forward/inverse results match (if needed)
def test_fk_ik_match():

    global theta1_glb
    global theta2_glb
    global theta3_glb

    theta1 = float(theta1_glb)
    theta2 = float(theta2_glb)
    theta3 = float(theta3_glb)
    results1 = delta_calcForward(theta1, theta2, theta3)
    #results2 = delta_calcInverse(results1[1], results1[2], results1[3])
    
    theta1_glb = roundoff(results1[1], 3)
    theta2_glb = roundoff(results1[2], 3)
    theta3_glb = roundoff(results1[3], 3)

    # Theta 1, Theta 2, Theta 3
    print("Motor 1 degrees:" + str(theta1_glb))
    print("Motor 2 degrees:" + str(theta2_glb))
    print("Motor 3 degrees:" + str(theta3_glb))

# To show the robot bounds (if needed)
def test_bounds():

    maxx = -e - f - re - rf
    maxy = maxx
    maxz = maxx
    minx = -maxx
    miny = -maxx
    minz = -maxx
    sd = 360.0 / s
    x, y, z = None, None, None
    # find extents
    for z in range(s):
        r = delta_calcForward(z * sd, z * sd, z * sd)
        if r[0] == 0:
            if minz > r[3]:
                minz = r[3]
            if maxz < r[3]:
                maxz = r[3]
    if minz < -btf:
        minz = -btf
    if maxz < -btf:
        maxz = -btf
    middlez = (maxz + minz) * 0.5
    # $('#output').append("<p>("+maxz+","+minz+","+middlez+")</p>")
    original_dist = (maxz - middlez)
    dist = original_dist * 0.5
    sum = 0
    r = [None] * 8
    mint1 = 360
    maxt1 = -360
    mint2 = 360
    maxt2 = -360
    mint3 = 360
    maxt3 = -360
    while True:
        sum += dist
        r[0] = delta_calcInverse(+sum, +sum, middlez + sum)
        r[1] = delta_calcInverse(+sum, -sum, middlez + sum)
        r[2] = delta_calcInverse(-sum, -sum, middlez + sum)
        r[3] = delta_calcInverse(-sum, +sum, middlez + sum)
        r[4] = delta_calcInverse(+sum, +sum, middlez - sum)
        r[5] = delta_calcInverse(+sum, -sum, middlez - sum)
        r[6] = delta_calcInverse(-sum, -sum, middlez - sum)
        r[7] = delta_calcInverse(-sum, +sum, middlez - sum)
        if (r[0][0] != 0 or r[1][0] != 0 or r[2][0] != 0 or r[3][0] != 0 or
            r[4][0] != 0 or r[5][0] != 0 or r[6][0] != 0 or r[7][0] != 0):
            sum -= dist
            dist *= 0.5
        else:
            for i in range(8):
                if mint1 > r[i][1]:
                    mint1 = r[i][1]
                if maxt1 < r[i][1]:
                    maxt1 = r[i][1]
                if mint2 > r[i][2]:
                    mint2 = r[i][2]
                if maxt2 < r[i][2]:
                    maxt2 = r[i][2]
                if mint3 > r[i][3]:
                    mint3 = r[i][3]
                if maxt3 < r[i][3]:
                    maxt3 = r[i][3]
        if original_dist <= sum or dist <= 0.1:
            break
    home = delta_calcForward(0,0,0)

    #center
    print("center :")
    print("(0,0,"+ str(roundoff(middlez,3)) +")")

    #home
    print("home: ")
    print("(0,0,"+ str(roundoff(home[3],3)) +")")

    #bounds
    print("bounds: ")
    print("X="+ str(roundoff(-sum,3)) + " to " + str(roundoff(sum,3)) +" mm")
    print("Y="+ str(roundoff(-sum,3)) + " to " + str(roundoff(sum,3)) + " mm")
    print("Z="+ str(roundoff(middlez-sum,3)) + " to " + str(roundoff(middlez+sum,3)) +" mm")

    #limits
    print("limits: ")
    print("theta 1="+ str(roundoff(mint1,2)) + " to " + str(roundoff(maxt1,2)))
    print("theta 2="+ str(roundoff(mint2,2)) + " to " + str(roundoff(maxt2,2)))
    print("theta 3="+ str(roundoff(mint3,2)) + " to " + str(roundoff(maxt3,2)))
    
    # resolution  
    r1=delta_calcForward(0,0,0)
    r2=delta_calcForward(sd,0,0)
    x=(r1[1]-r2[1])
    y=(r1[2]-r2[2])
    sum=math.sqrt(x*x+y*y)

    # resolution
    print("resolution: ") 
    print("+/-"+ str(roundoff(sum,3)) +"mm")


# forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
# returned {error code, theta1,theta2,theta3}
def delta_calcForward(theta1, theta2, theta3):
    x0 = 0.0
    y0 = 0.0
    z0 = 0.0
    t = (f - e) *tan30 / 2.0
    dtr = pi / 180.0
    theta1 *= dtr
    theta2 *= dtr
    theta3 *= dtr
    y1 = -(t + rf * math.cos(theta1))
    z1 = -rf * math.sin(theta1)
    y2 = (t + rf * math.cos(theta2)) * sin30
    x2 = y2 * tan60
    z2 = -rf * math.sin(theta2)
    y3 = (t + rf * math.cos(theta3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf * math.sin(theta3)
    dnm = (y2 - y1) * x3 - (y3 - y1) * x2
    w1 = y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3
    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0
    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0
    a = a1 * a1 + a2 * a2 + dnm * dnm
    b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
    c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re)
    d = b * b - 4.0 * a * c
    if d < 0.0:
        return [1, 0, 0, 0]
    z0 = -0.5 * (b + math.sqrt(d)) / a
    x0 = (a1 * z0 + b1) / dnm
    y0 = (a2 * z0 + b2) / dnm
    return [0, x0, y0, z0]    

# To manually take robot geomatry (if needed)
def read_inputs():
    e = input('Enter End Effector radius e: ')
    f = input('Enter Base radius f: ')
    re = input('Enter Forearm length re: ')
    rf = input('Enter Bicep length rf: ')
    s = input('Enter Steps per turn s: ')
    btf = input('Enter Base to floor distance b: ')

# To convert +ve,-ve angles to (0-180)         
# To convert degrees to steps (steps ranging from 500-2500)
# Total steps 2000
def degreesToSteps():

    global theta1_glb
    global theta2_glb
    global theta3_glb
    global binary1_glb
    global binary2_glb
    global binary3_glb

    # converting +ve,-ve angles to (0-180)
    theta1_glb=180-(theta1_glb+90)
    theta2_glb=180-(theta2_glb+90)
    theta3_glb=180-(theta3_glb+90)

    # Theta 1, Theta 2, Theta 3
    print("Motor 1 degrees:" + str(theta1_glb))
    print("Motor 2 degrees:" + str(theta2_glb))
    print("Motor 3 degrees:" + str(theta3_glb))   

    # Doing some angle corrections of 1st and 2nd arms to put them at 90 degree 
    theta1_glb = theta1_glb-4
    theta2_glb = theta2_glb+2

    # converting degrees to steps
    step1 = (2000/180)*theta1_glb+500
    step2 = (2000/180)*theta2_glb+500
    step3 = (2000/180)*theta3_glb+500

    # Step value 1, Step value 2, Step value 3
    print("Motor 1 Steps:" + str(step1))
    print("Motor 2 Steps:" + str(step2))
    print("Motor 3 Steps:" + str(step3))

    # Call to convert step values to binary
    binary1_glb = stepsToBinary(step1)
    binary2_glb = stepsToBinary(step2)
    binary3_glb = stepsToBinary(step3)
    
# To convert step values to binary
# Reterns 16bit values
def stepsToBinary(s):

    # Converting a step value to 16bit binary  
    positive_binary = '{0:016b}'.format(int(s))

    # Taking higher and lower 8 bits into variables
    hb= positive_binary[0:8]
    lb= positive_binary[8:16]

    # To convert 8bit binary to decimal 
    def binaryToDecimal(binary):
 
        decimal, i = 0, 0
        while(binary != 0):
            dec = binary % 10
            decimal = decimal + dec * pow(2, i)
            binary = binary//10
            i += 1
        return(decimal)
 
    # Assigning decimal values of higher and lower bytes into variables
    hib = binaryToDecimal(int(hb))
    lib = binaryToDecimal(int(lb))

    # Left shift decimal value of higher byte by 1 
    hib = hib <<1

    # If lower byte is greater then or equal to 128 we subtract it by 128 and left shift higher byte by 1
    if (lib>=128):
        lib = lib-128
        hib = (1<<1)-1 | hib

    # To convert decimal value of lower byte to the 2's compliment binary value
    def to_twoscomplement(bits, value):
        if value < 0:
            value = ( 1<<bits ) + value
        formatstring = '{:0%ib}' % bits
        return formatstring.format(value)

    # Assigning 2's compliment binary value of lower byte to the variable 
    lib= to_twoscomplement(8,lib)

    # Swap higher byte to lower byte and vice versa
    temp = lib
    lib=hib
    hib=temp

    # Converting lower byte decimal value to binary(earlier it was higher byte decimal value befor swap)
    #hib = '{0:08b}'.format(hib)
    lib = '{0:08b}'.format(lib)

    print(positive_binary)

    # Combining higher and lower byte to create a 16bit binary value and return it
    binary = hib+lib
    return(binary)
            
# To generate the binary command of 38 Bytes and send it over COM port to the Arduino
# First 2 Bytes as start command
# Rest of 36 Bytes for 18 servo motors, each motor receiving a command of 2 Bytes  
def positionSet():

    # Command to put the Arduino into the state in which we can select as to which operation we want to perform (170)
    # Command to put it in State Servo Position mode (10)
    #             170         10
    startcmd = "10101010"+"00001010"

    # Empty command as pading
    #             92         11
    emptycmd = "01011100"+"00001011"

    # List of start command and commands for motors
    list = [startcmd[0:8],startcmd[8:16],binary1_glb[0:8],binary1_glb[8:16],binary2_glb[0:8],binary2_glb[8:16],binary3_glb[0:8],binary3_glb[8:16],binary1_glb[0:8],binary1_glb[8:16],binary2_glb[0:8],binary2_glb[8:16],binary3_glb[0:8],binary3_glb[8:16],binary1_glb[0:8],binary1_glb[8:16],binary2_glb[0:8],binary2_glb[8:16],binary3_glb[0:8],binary3_glb[8:16]]
    
    # Padding the command list with empty commands to make 38 Bytes in total 
    i=20
    while(i<38):
        list.insert(i,emptycmd[0:8])
        i = i+1
        list.insert(i,emptycmd[8:16])
        i = i+1

    # Creating a numpy array of the binary command list
    arr = np.array(list)

    # converting the binary values in numpy array to the decimal list
    decimal = [0]*38
    for i in range(38):
        decimal[i]= int(arr[i],2)

    # Converting the decimal list to a string  
    st= ""
    for i in range(38):
        st += chr(decimal[i])
    print(st)

    # Converting deicmal list to Hex and printing it for reference 
    my_hex = ''.join(f'{i:02x}' for i in decimal)
    print(my_hex)

    # Converting the string command to byte array while encoding it in UTF-8
    cmd= bytearray(st,'utf-8')

    # Sending the command to the Arduino via COM port
    comPort.ser.write(cmd[1:39])

# To set the speed of the servo motors
# Taking speed value as input and generating the speed command and send it via COM port to Arduino
def speedSet(s):

    # Command to put the Arduino into the state in which we can select as to which operation we want to perform (170)
    # Command to put it in State Speed mode (8)
    #             170         8
    startCmd = "10101010"+"00001000"

    # Converting the speed value to binary
    binary = '{0:08b}'.format(int(s))

    # Genrating a list of binary commands
    cm = [startCmd[0:8],startCmd[8:16],binary]

    # Converting the list to a numpy array
    arr1 = np.array(cm)
    
    # Initializing the list
    dec = [0]*3
    st1=""

    # Converting the Binary values in numpy array to the decimal list  
    for i in range(3):
        dec[i]= int(arr1[i],2)

    # Converting the decimal list to a string  
    st1= ""
    for i in range(3):
        st1 += chr(dec[i])
    print(st1)

    # Converting deicmal list to Hex and printing it for reference 
    hex = ''.join(f'{i:02x}' for i in dec)
    print(hex)

    # Converting the string command to byte array while encoding it in UTF-8
    speedCmd= bytearray(st1,'utf-8')

    # Sending the command to the Arduino via COM port
    comPort.ser.write(speedCmd[1:4])

# To Activate/Deactivate the Electromagnet
# Taking 0/1 as input and generating the Activation/Deactivation command and send it via COM port to Arduino
def setElectroMagnet(state):

    # Command to put the Arduino into the state in which we can select as to which operation we want to perform (170)
    # Command to put it in Electromagnet set mode (16)
    #             170         16
    startCmd = "10101010" + "00010000"

    # Converting state value to binary
    binary = '{0:08b}'.format(int(state))

    # Genrating a list of binary commands
    cm = [startCmd[0:8], startCmd[8:16], binary]

    # Converting the list to a numpy array
    arr1 = np.array(cm)

    # Initializing the list
    dec = [0] * 3
    st1 = ""

    # Converting the Binary values in numpy array to the decimal list
    for i in range(3):
        dec[i] = int(arr1[i], 2)

    # Converting the decimal list to a string
    st1 = ""
    for i in range(3):
        st1 += chr(dec[i])
    print(st1)

    # Converting deicmal list to Hex and printing it for reference
    hex = ''.join(f'{i:02x}' for i in dec)
    print(hex)

    # Converting the string command to byte array while encoding it in UTF-8
    electroMagnetCmd = bytearray(st1, 'utf-8')

    # Sending the command to the Arduino via COM port
    comPort.ser.write(electroMagnetCmd[1:4])



# To set the speed of the Stepper motor(Conveyor belt)
# Taking speed value as input and generating the speed command and send it via COM port to Arduino
def setStepperSpeed(s):

    # Command to put the Arduino into the state in which we can select as to which operation we want to perform (170)
    # Command to put it in State Speed mode (17)
    #             170         17
    startCmd = "10101010" + "00010001"
    
    # Converting the speed value to binary
    binary_1 = '{0:08b}'.format(int(s))

    # Genrating a list of binary commands
    cm = [startCmd[0:8], startCmd[8:16], binary_1]

    # Converting the list to a numpy array
    arr1 = np.array(cm)

    # Initializing the list
    dec = [0] * 3
    st1 = ""

    # Converting the Binary values in numpy array to the decimal list
    for i in range(3):
        dec[i] = int(arr1[i], 2)

    # Converting the decimal list to a string 
    st1 = ""
    for i in range(3):
        st1 += chr(dec[i])
    print(st1)

    # Converting deicmal list to Hex and printing it for reference
    hex = ''.join(f'{i:02x}' for i in dec)
    print("stepper")
    print(hex)

    # Converting the string command to byte array while encoding it in UTF-8
    setStepperSpeedCmd = bytearray(st1, 'utf-8')

    # Sending the command to the Arduino via COM port
    comPort.ser.write(setStepperSpeedCmd[1:4])

# To normalize step size of 0-180mm(distance to be travelled) with the speed of 90-100 and return it
# Bigger the distance greater the speed
def minMaxNormalize(s):
    speed = s/180*(10)+90
    return(speed)

# To normalize speed of 90-100 with the delay of 0.01-0.31 and return it 
# Greater the speed longer the delay time
def minMaxDelay(s):
    val = 0.01+(0.03)*(s-90)
    return val

# Class to initialize x and y for the persuite and distance functions
class Point:
  def __init__(self,x, y):
    self.x = x
    self.y = y


# To check if the cross product of (b-a) and (c-a) is 0, tells you if the points a, b and c are aligned.
# Then if you want to know if c is between a and b, 
# You also have to check that the dot product of (b-a) and (c-a) is positive,
# And is less than the square of the distance between a and b.
# It returns true if c is between a and b
def isBetween(a, b, c):

    # To calculate cross product of (b-a) and (c-a)
    crossproduct = (c.y - a.y) * (b.x - a.x) - (c.x - a.x) * (b.y - a.y)

    # compare versus epsilon for floating point values, or != 0 if using integers
    if abs(crossproduct) >0.00000001:
        return False

    # To calculate dot product of (b-a) and (c-a)
    dotproduct = (c.x - a.x) * (b.x - a.x) + (c.y - a.y)*(b.y - a.y)
    if dotproduct < 0:
        return False
    
    # To check if the dot product is less than the square of the distance between a and b.
    squaredlengthba = (b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y)
    if dotproduct > squaredlengthba:
        return False

    # Return true
    return True

# To calculate the Euclidean distance between the two points
def distance(a,b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)


# To return -1 if num is negative, 1 otherwise
def sgn (num):
    if num >= 0:
        return 1
    else:
        return -1
    
# currentPos: [currentX, currentY]
# pt1: [currentX, currentY]
# pt2: [Targetx, Targety]
# lookAheadDis: step value
def pursuit(currentPos, pt1, pt2, lookAheadDis):


    # extract currentX, currentY, x1, x2, y1, and y2 from input arrays
    currentX = currentPos[0]
    currentY = currentPos[1]
    x1 = pt1[0]  
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]  

    # Assigning robot [currentX, currentY] to p1 and [Targetx, Targety] to p2
    p1 = Point(pt1[0],pt1[1])
    p2 = Point(pt2[0],pt2[1])

    
    # boolean variable to keep track of if intersections are found
    intersectFound = False  
    
    # output (intersections found) should be stored in arrays sol1 and sol2  
    # if two solutions are the same, store the same values in both sol1 and sol2  
    
    # subtract currentX and currentY from [x1, y1] and [x2, y2] to offset the system to origin  
    x1_offset = x1 - currentX  
    y1_offset = y1 - currentY  
    x2_offset = x2 - currentX  
    y2_offset = y2 - currentY  
    
    # calculate the discriminant 
    dx = x2_offset - x1_offset
    dy = y2_offset - y1_offset
    dr = math.sqrt (dx**2 + dy**2)
    D = x1_offset*y2_offset - x2_offset*y1_offset
    discriminant = (lookAheadDis**2) * (dr**2) - D**2  
    
    # if discriminant is >= 0, there exist solutions
    if discriminant >= 0:
        intersectFound = True
    
        # calculate the solutions
        sol_x1 = (D * dy + sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
        sol_x2 = (D * dy - sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
        sol_y1 = (- D * dx + abs(dy) * np.sqrt(discriminant)) / dr**2
        sol_y2 = (- D * dx - abs(dy) * np.sqrt(discriminant)) / dr**2    
    
        # add currentX and currentY back to the solutions, offset the system back to its original position
        sol1 = [sol_x1 + currentX, sol_y1 + currentY]
        sol1_pt = Point(sol1[0],sol1[1])
        
        sol2 = [sol_x2 + currentX, sol_y2 + currentY] 
        sol2_pt = Point(sol2[0],sol2[1]) 
    
    # If the intersection is equal to false then No intersection Found
    if intersectFound == False :
        print ('No intersection Found!')

    # Else Solution found
    else:
        print ('Solution 1 found at [{}, {}]'.format(sol1[0], sol1[1]))
        print ('Solution 2 found at [{}, {}]'.format(sol2[0], sol2[1]))

        # To check if the solution 1 is between the robot and the object then return solution 1 
        if(isBetween(p1,p2,sol1_pt)):
            return(sol1)
        
        # To check Else if the solution 2 is between the robot and the object then return solution 2
        elif(isBetween(p1,p2,sol2_pt)):
            return(sol2)
    
# To check the length of the queue and return it using lock
def length(lock):
    lock.acquire()
    val_length = len(object_queue)
    lock.release()
    return val_length

# To enqueue the target object into the queue at the end using lock
def enqueue(obj,lock):
    lock.acquire()
    object_queue.append(obj)
    lock.release()

# To get the target object from the front of the queue using lock
def get(lock):
    lock.acquire()
    obj=object_queue[0]
    lock.release()
    return(obj)

# To dequeue the target object from the front of the queue using lock
def dequeue(lock):
    lock.acquire()
    obj = object_queue.pop(0)
    lock.release()
    return(obj)
	

##################################################################################

import cv2
#import time as time
#import threading
#import numpy as np
from image_registration import chi2_shift

global obj_count
obj_count = 0

global size_thresh
size_thresh = 30

global calib_factor
global new_origin_x
global new_origin_y
global output_img

output_img = np.zeros(300)


calib_factor = 0.333
new_origin_x = 400
new_origin_y = 85

def preprocess(frame):
    # resizing
    frame = cv2.resize(frame, (640, 480))

    # cropping
    frame = frame[170:460, :]
    output = frame.copy()

    # color conversion to Gray format
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # current_frame = 255-current_frame

    # Applying Gaussian Blur
    frame = cv2.GaussianBlur(frame, (11, 11), 0)

    _, frame_one = cv2.threshold(frame[:, :, 0], 160, 255, cv2.THRESH_BINARY)
    _, frame_two = cv2.threshold(frame[:, :, 1], 160, 255, cv2.THRESH_BINARY)
    _, frame_three = cv2.threshold(frame[:, :, 2], 160, 255, cv2.THRESH_BINARY)

    frame = cv2.bitwise_or(frame_one,frame_two)
    frame = cv2.bitwise_or(frame, frame_three)

    # Applying Image Thresholding
    # _, frame = cv2.threshold(frame, 130, 255, cv2.THRESH_BINARY)

    return frame, output

def remove_boundary(original_mask):
    _, labels = cv2.connectedComponents(original_mask)
    # Get frame dimensions
    height, width = labels.shape

    # Create a set to store labels of components that touch the boundary
    boundary_touching_labels = set()

    # Iterate through the boundary pixels and collect component labels
    for i in range(height):
        for j in range(width):
            if i == 0 or j == 0 or i == height - 1 :
                boundary_touching_labels.add(labels[i, j])

    # Create a mask for the components to ignore
    filtered_mask = np.zeros_like(labels, dtype=np.uint8)

    for label in boundary_touching_labels:
        filtered_mask[labels == label] = 255

    # Set the boundary-touching components to black in the original frame
    original_mask[filtered_mask == 255] = 0
    return original_mask

def img_reg(prev_img, curr_img):
    image = prev_img
    offset_image = curr_img

    # chi squared shift
    # Find the offsets between image 1 and image 2 using the DFT upsampling method
    noise = 0.1
    xoff, yoff, exoff, eyoff = chi2_shift(image, offset_image, noise, return_error=True, upsample_factor='auto')

    #print("Pixels shifted by: ", xoff, yoff)

    from scipy.ndimage import shift
    # In order to get the shifted aligned image we use shift
    # corrected_image = shift(offset_image, shift=(-yoff, -xoff), mode='constant')
    corrected_image = shift(image, shift=(yoff, xoff), mode='constant')

    # xoring the corrected and offset images gives us the new object in the frame
    xor_image = cv2.bitwise_xor(offset_image, corrected_image)
    # cv2.imshow("xor1", xor_image)

    # Using morphological opening to reduce noise in the frame
    kernel = np.ones((13, 13), np.uint8)
    xor_image = cv2.morphologyEx(xor_image, cv2.MORPH_OPEN, kernel, iterations=1)
    cv2.imshow("xor", xor_image)

    height, width = xor_image.shape
    mask = np.ones_like(xor_image, dtype=np.uint8) * 255
    roi = mask[:, width // 3:]
    roi[:, :] = 0

    # Apply the mask to the binarized image
    masked_image = cv2.bitwise_and(xor_image, xor_image, mask=mask)
    cv2.imshow("masked", masked_image)
    return masked_image, xoff, yoff


def get_details(xor_mask, output):

    # calib_factor = 0.5416
    # new_origin_x = 900
    # new_origin_y = 900

    # Apply the Component analysis function
    analysis = cv2.connectedComponentsWithStats(xor_mask)
    (num_labels, labels, stats, centroids) = analysis

    for i in range(1, num_labels):  # index 0 is for background
        # count += 1
        col_label = None
        obj_img = np.zeros(labels.shape, dtype="uint8")
        obj_img[labels == i] = 255
        res = cv2.bitwise_and(output, output, mask=obj_img)  # AND with original image
        hsv_res = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)

        red_mask1 = cv2.inRange(hsv_res, red_lower1, red_upper1)
        # red_area = cv2.countNonZero(red_mask1)
        red_mask2 = cv2.inRange(hsv_res, red_lower2, red_upper2)
        # red_area = cv2.countNonZero(red_mask2)
        red_mask = red_mask1 + red_mask2
        red_area = cv2.countNonZero(red_mask)
        #cv2.imshow("red", red_mask)
        green_mask = cv2.inRange(hsv_res, green_lower, green_upper)
        green_area = cv2.countNonZero(green_mask)

        blue_mask = cv2.inRange(hsv_res, blue_lower, blue_upper)
        blue_area = cv2.countNonZero(blue_mask)

        if red_area > 70:
            col_label = 0   # "Red"
        elif green_area > 70:
            col_label = 1   # "Green"
        elif blue_area > 70:
            col_label = 2   # "Blue"

        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]

        (cX, cY) = centroids[i]
        cX, cY = np.float16(cX), np.float16(cY)

        global obj_count
        obj_count = obj_count + 1

        mm_cX = cX * calib_factor
        mm_cY = cY * calib_factor

        relative_x = mm_cX - new_origin_x
        relative_y = mm_cY - new_origin_y

        # relative_x = new_origin_x - mm_cX
        # relative_y = new_origin_y - mm_cY

        thread_obj = [relative_x, relative_y, col_label, w, h, obj_count]
        # thread_obj = [mm_cX, mm_cY, col_label, w, h, obj_count]

        return thread_obj

def show_details(obj, convey_img):
    img = np.copy(convey_img)
    if(obj==None):
        return(img)
    mm_cX = obj[0] + new_origin_x
    mm_cY = obj[1] + new_origin_y

    # mm_cX = new_origin_x - obj[0]
    # mm_cY = new_origin_y - obj[1]

    cenX = mm_cX/calib_factor
    cenY = mm_cY/calib_factor

    w = obj[3]
    h = obj[4]
    x = cenX - (w/2)
    y = cenY - (h/2)

    if x >= 0 and y >= 0 and x + w <= output_img.shape[1] and y + h <= output_img.shape[0]:
        cv2.putText(img, str(obj[5]), (int(cenX), int(cenY)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 215, 255), 2)
        # cv2.rectangle(img, (int(obj[3]), int(obj[4])), (int(obj[3]) + int(obj[5]), int(obj[4]) + int(obj[6])), (0, 255, 0), 2)
        # cv2.rectangle(img, (int(x), int(y)), (int(x+w), int(y+h)), (0, 255, 0), 2)
        # cv2.imshow("show details", output_img)
    return(img)

red_lower1 = np.array([1, 40, 70])
red_upper1 = np.array([20, 255, 255])
red_lower2 = np.array([140, 40, 70])
red_upper2 = np.array([179, 255, 255])

green_lower = np.array([20, 50, 100])
green_upper = np.array([80, 255, 255])

blue_lower = np.array([87, 30, 100])
blue_upper = np.array([130, 255, 255])

def isCopy(lock, obj):

    if length(lock)<1:
        return False
    lock.acquire()
    for q_obj in object_queue:
        Dist = distance(Point(q_obj[0], q_obj[1]), Point(obj[0], obj[1]))
        print("Distance " + str(Dist))
        if(Dist < size_thresh):
            lock.release()
            return True
    lock.release()
    return False

def imaging_thread(lock):
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    #cap = cv2.VideoCapture('VID20230826091942.mp4')

    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
    # cap.set(cv2.CAP_PROP_EXPOSURE, -6.0)
    # cap.set(cv2.CAP_PROP_FPS, 30)
    # cap.set(cv2.CAP_PROP_BRIGHTNESS, 50.0)
    # cap.set(cv2.CAP_PROP_CONTRAST, 150.0)
    # cap.set(cv2.CAP_PROP_SATURATION, 120.0)

    previous_frame = None
    global output_img
    # count = 0

    # Loop to read frames in pairs
    while True:
        ret, current_frame = cap.read()

        # call to method 'preprocess'
        current_frame, output = preprocess(current_frame)

        # call to method 'remove_boundary'
        current_frame = remove_boundary(current_frame)
        #lock.acquire()
        output_img = output.copy()
        contours, _ = cv2.findContours(current_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for i in range(0, len(contours)):
            if((cv2.contourArea(contours[i]))>100):
                    cv2.drawContours(output_img, contours, i, (0, 255, 200), 2)
        #lock.release()
        if previous_frame is not None:

            cv2.imshow('Current Frame', current_frame)
            cv2.imshow('Previous Frame', previous_frame)

            # call to method 'img_reg'
            # xor_mask, xoff, yoff = img_reg(previous_frame, current_frame)
            xor_mask, xoff1, yoff1 = img_reg(previous_frame, current_frame)

            thread_obj = get_details(xor_mask, output)

            if thread_obj is not None and thread_obj[2] is not None:
                if not isCopy(lock, thread_obj):
                    enqueue(thread_obj, lock)

        previous_frame = current_frame

        #cv2.imshow('output', output)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # Press 'q' to exit the loop
        # time.sleep(0.1)

# def imaging_thread(lock):
#     # To insert elements into the queue
#     while (1):
#         lable_bin = random.randint(0, 2)
#         y = random.randint(-40, 40)
#         x = -490
#         enqueue([x, y, lable_bin], lock)
#         print("Object Created", x, y, lable_bin)
#         time.sleep(1.8)

###################################################################################

def update(lock, offset):
    global object_queue
    img = np.copy(output_img)
    lock.acquire()
    if (len(object_queue) < 1):
        img = show_details(None, img)
        cv2.imshow("show details", img)
        cv2.waitKey(1)
        lock.release()
        return
    print("length", len(object_queue))
     #print("updating")
    queue_temp = []
    for i in range(0, len(object_queue)):
        obj = object_queue[i]

        print("obj", obj)
        # print("offset",offset)
        obj[0] = obj[0] + offset[0]
        obj[1] = obj[1] + offset[1]

        if (obj[0] < xMax):
            queue_temp.append(obj)
        else:
            print("object out of bound")
        img = show_details(obj, img)

    object_queue = queue_temp
    # print(object_queue)
    lock.release()
    cv2.imshow("show details", img)
    cv2.waitKey(1)
    # print(object_queue)


def updatation_thread(lock):
    #To update the coordinates
    previous_time = 0
    offset = []
    while(1):
        current_time = time.time()
        if(previous_time==0):
            previous_time = current_time
            continue
        elif(current_time - previous_time > 0):
            quanta = current_time-previous_time
            offset.append(quanta*xSpeed)
            offset.append(quanta*ySpeed)
            update(lock, offset)

            #print("offset",offset)
            offset.clear()
        previous_time = current_time
        time.sleep(0.1)

# To move the delta robot
def delta_thread(lock):

    #Testing Bounds for reference
    test_bounds()

    # Homing the Delta to homing Height
    test_ik(0,0,zRetract)
        
    # To convert degrees to steps (steps ranging from 500-2500)
    degreesToSteps()

    #To set the speed of the servo motor
    speedSet(80)

    # To set the Positions of the servo motor
    positionSet()
    currentX=0
    currentY=0
    currentZ=zSafe

    # Infinite loop 
    while(1):

        # Delay
        time.sleep(0.1)

        # Loop while the length of the object queue is greater then 0
        while(length(lock)>0):

            # Infinite loop
            while(1):

                # To place the target object ahead
                xAhead=xSpeed*0.5
                yAhead=ySpeed*0.5
                obj = get(lock)
                targetX = obj[0]+xAhead
                targetY = obj[1]+yAhead

                # To check if target object is outside minimum x bounds
                if(targetX<xMin):
                    time.sleep(0.05) #Sleep 
                    break # Break loop

                # To check Else if target object is outside maximum x bounds
                elif(targetX>xMax):
                    dequeue(lock) # Dequeue the object from the object queue
                    print("object out of bound")  
                    break # Break the loop

                # Calculating the Euclidean distance from the robot to the target object
                step = abs(distance(Point(currentX,currentY),Point(targetX,targetY)))

                # Normalizing distance to speed
                speed = minMaxNormalize(step)

                # To find the solution using pure pursuit
                # Taking current coordinates of robot as current position and point 1 as current position also 
                # Taking full step to the target object
                midPoint = pursuit([currentX,currentY],[currentX,currentY],[targetX,targetY],step)

                # Assigning the solution Coordinates
                motionX = midPoint[0]
                motionY = midPoint[1]

                # Updating the current position beforehand
                currentX=motionX 
                currentY=motionY

                print(motionX,motionY)

                # Checking if the target object coordinates are withen the bounds of the robot
                if(motionX>=xMin and motionX<=xMax and motionY>=yMin and motionY<=yMax):

                    # To calculate the motor angles/degrees given target object coordinates
                    test_ik(motionX,motionY,zSafe)

                    # Converting degrees to steps
                    degreesToSteps()

                    # Setting the servo motor's speed  
                    speedSet(speed)

                    # Command to take the robot to the target object location
                    positionSet()

                    # Sleep, giving time for the physical motion of the robot to happen 
                    time.sleep(minMaxDelay(speed))

                # Re calculating the distance between the robot and the object
                current_diff = abs(distance(Point(motionX,motionY),Point(targetX,targetY)))

                # To check if there is any distance between the robot and the object
                # If its less the or equal to 5mm
                if(current_diff<=5):

                    # To turn on the electtromagent
                    setElectroMagnet(1)

                    # Sleep
                    time.sleep(0.15)

                    # To calculate the motor angles/degrees given target object coordinates
                    # Going down to pick the target object at Action height 
                    test_ik(motionX,motionY,zAction)

                    # Converting degrees to steps
                    degreesToSteps()

                    # Setting the servo motor's speed
                    speedSet(100)

                    # Command to go down to pick the target object  
                    positionSet()

                    # Sleep, giving time for the physical motion of the robot to happen
                    time.sleep(0.2)

                    print("object picked at", currentX, currentY)
                    print("object position", targetX, targetY)

                    # To calculate the motor angles/degrees given target object coordinates
                    # Going Up to the Safe height 
                    test_ik(motionX,motionY,zSafe)

                    # Converting degrees to steps
                    degreesToSteps()

                    # Setting the servo motor's speed
                    speedSet(100)

                    # Command to go Up to the Safe height 
                    positionSet()

                    # Sleep, giving time for the physical motion of the robot to happen
                    time.sleep(0.2)

                    # To assign the bin lable of the object
                    lable = obj[2]

                    # To asign the bin coocrdinates 
                    bin_cord = bins[lable]

                    #Calculating the Euclidean distance from the robot to the target object BIN
                    step = distance(Point(currentX, currentY), Point(bin_cord[0],bin_cord[1]))

                    # Normalizing distance to speed
                    sped = minMaxNormalize(step)


                    print("speed at the time of dropping",sped)
                    
                    # To calculate the motor angles/degrees given target object BIN coordinates
                    # Going to the target object BIN coordinates
                    test_ik(bin_cord[0],bin_cord[1],zSafe)

                    # Converting degrees to steps
                    degreesToSteps()

                    # Setting the servo motor's speed
                    speedSet(sped)

                    # Command to go to the target object BIN coordinates
                    positionSet()

                    # Sleep, giving time for the physical motion of the robot to happen
                    time.sleep(minMaxDelay(sped))

                    # To calculate the motor angles/degrees given target object BIN coordinates
                    # Going down to drop the target object into the BIN
                    test_ik(bin_cord[0],bin_cord[1],zAction)

                    # Converting degrees to steps
                    degreesToSteps()

                    # Setting the servo motor's speed
                    speedSet(100)

                    # Command to go down to drop the target object into the BIN
                    positionSet()

                    # Sleep, giving time for the physical motion of the robot to happen
                    time.sleep(0.2)

                    # Command to turn off the electromagnet
                    setElectroMagnet(0)

                    # To calculate the motor angles/degrees given target object BIN coordinates
                    # Going up to the Safe height
                    test_ik(bin_cord[0],bin_cord[1],zSafe)

                    # Converting degrees to steps
                    degreesToSteps()

                    # Setting the servo motor's speed
                    speedSet(100)

                    # Command to go up to the safe height
                    positionSet()

                     # Sleep, giving time for the physical motion of the robot to happen
                    time.sleep(0.2)

                    print("object dropped")

                    # Dequeue to target object from the target object queue
                    dequeue(lock)

                    # Breaking out of the loop
                    break


def main_task():
	
	# creating a lock
    lock = threading.Lock()

	# creating threads
    t1 = threading.Thread(target=imaging_thread, args=(lock,))
    t2 = threading.Thread(target=delta_thread, args=(lock,))
    t3 = threading.Thread(target=updatation_thread, args=(lock,))

	# start threads
    t1.start()
    t2.start()
    t3.start()

	# wait until threads finish their job
    t1.join()
    t2.join()
    t3.join()
	
# To check if the file is executed directly then run the main_task 
if __name__ == "__main__":
    
    # TO RUN THE THREADS 
	main_task()

		 
         	
