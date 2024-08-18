/*		ROBOKITS ARDUINO USB/BLUETOOTH 18 SERVO CONTROLLER FIRMWARE

This code is generated from Robokits Arduino Bluetooth/USB Servo Controller software.
The hardware consist of 2 AVRs out of which one is Master and has Arduino bootloader. The second one is slave and runs all servo routines.
The master controls slave through I2C and slave operates all servos.
Many pins of the main controller are free and can be used for connecting other peripherals and interfaces. 

Firmware contains 2 parts (functions in void loop()) - 1. Run in PC controlled Mode and 2. Run User Code
The code is selected through a Jumper on Servo controller board.
You can change the user code as per your need.


This code is generated for Arduino UNO board profile and the same should be chosed while programming with Arduino IDE.
Arduino IDE veersion 1.6.5 + is recommended.

If the code is moved to other servo base platform like a robot, offsets must be set properly so that servos will move correctly as per program.

*/
//I2C library
#include <Wire.h>

//TimerOne Library to generate intrupt frequency for stepper motor 
#include <TimerOne.h>

//I2C board 1
#define servo1	(16>>1)

//I2C board 2 if attached
#define servo2	(18>>1)

//Setting the UART BAUD rate (Bits/Second)
#define UART_BAUD_RATE	115200

//LED pin
#define LED 13

//Serial buffer size
#define SERIAL_BUFFER_SIZE  256

//electromagnet pin
const int electroPin = 7;

//stepperMotor pins
const int enblPin = 4;
const int dirPin = 5; 
const int stepPin = 6;

//Define delays between commands, you can play with these
int delayInMicroSeconds; // Time between (HIGH/LOW) , (HIGH/LOW)

//Function declearations 
void I2C_SERVOSET(unsigned char servo_num,unsigned int servo_pos);
void I2C_SERVOREVERSE(unsigned char servo_num,unsigned char servo_dir);
void I2C_SERVOOFFSET(unsigned char servo_num,int value);
void I2C_SERVOSPEED(unsigned char value);
void I2C_SERVONUTRALSET(unsigned char servo_num,unsigned int servo_pos);
void I2C_SERVOMIN(unsigned char servo_num,unsigned int servo_pos);
void I2C_SERVOMAX(unsigned char servo_num,unsigned int servo_pos);
char I2C_SERVOEND(void);
int I2C_SERVOGET(int servo_num);
int I2C_SERVOGETOFFSET(int servo_num);
void CheckEndMovement(void);
void PCControlledCode(void);
void UserCode(void);
void LEDToggle(void);

//New functions Created
void stepper(void);
void stepperSpeed(int speed);
void electroMagnet(int var);
void timer(int t);

//Temporary veriables to store the buffer values 
volatile int cnt,c,servoval;

//State,buffer and count variable
volatile char state,servobuf[36],bytecnt;

//Interval of 100 Miliseconds to to check end movement and toggle LED
int interval=100;

//To store the privious time to calculate if 100 Miliseconds have passed or not
unsigned long previousMillis=0;

//Returns the number of milliseconds passed since the Arduino board began running the current program
unsigned long currentMillis = millis();

//1 for PC controlled, 0 for user controlled
char runCode =0; 	

//LED state variable
char LEDState=0;


//Defining Macros for different functions
#define State_Start  0
#define State_Command  1
#define State_Servoposition  2
#define State_Speed  3
#define State_Servomin  4
#define State_Servomax  5
#define State_Servooffset  6
#define State_Servoreverse  7
#define State_Servonutral  8
#define State_ReadOffsets  9
#define electroMagnet_set 10
#define stepper_speed_set 11


//To genrate a frequency which will drive the stepper motor
//Using TimerOne library to use the internal clock of the arduino
//An intrupt will be generated after the specified amount of miliseconds
//The intrupt will call the function stepper which genrates a HIGH/LOW signal
void timer(int t)
{ 
  //To check if the time is equal to 0
  if(t==0)
  {  
    Timer1.stop();//Stop the timer
  }

  //To check if the timer is between 200-32750
  if(t>=200 && t<=32750)
  {
    Timer1.initialize(t);//Initialize timer
    Timer1.attachInterrupt(stepper);//Attach it to a function
  }
}




//The setup() function is called when a sketch starts. 
//Use it to initialize variables, pin modes, start using libraries, etc. 
//The setup() function will only run once, after each powerup or reset of the Arduino board.
void setup()
{ 
	cnt=0;
	int i;
	unsigned int x;
	char buffer[10],tmp,tmp1;
	float range;
	Serial.begin(UART_BAUD_RATE);
	Serial.write('S');
	Serial.write('r');

  //LED pin
	pinMode(13, OUTPUT);
  
	pinMode(2, OUTPUT);

  //Jumper pin
	pinMode(8, INPUT);

  //electromagnet pin
  pinMode(7, OUTPUT);
	

  //stepperMotor
  pinMode( stepPin, OUTPUT); 
  pinMode( dirPin,  OUTPUT);
  pinMode( enblPin,  OUTPUT);
  digitalWrite(enblPin, HIGH);
  digitalWrite(dirPin, HIGH);
  

  //
  digitalWrite(8,1);
	delay(500);


  //To re-enables global interrupts after begin disabled. 
	sei();

  //I2C begin
	Wire.begin();
 
  //This register is used in master mode to set the division factor for bit rate generator (SCL clock frequency). 
  //The bit rate generator unit controls the time period of SCL. 
  //The SCL clock frequency is decided by the Bit Rate Register (TWBR) and the prescaler bits of TWSR register
  //Bits7-3 – TWS: TWI Status
  //These bits reflect the status of TWI bus 
  //Bit 2- Reserved bit
  //Bits1-0 – TWPS: TWI Prescaler Bits
  //These bits are used to set the prescaler of ‘Bit rate generator unit’. 
	TWSR = 3;	// no prescaler
	TWBR = 18;	//Set I2C speed lower to suite I2C Servo controller
 
	pinMode(2,OUTPUT);
	digitalWrite(2,HIGH);
	delay(500);

  //To set the state to State_start macro (0)
	state=State_Start;

  //To check if the pin 8 is jumpered with GND pin or not 
  //If its not jumpered with GND then run the PC code 
	if (digitalRead(8))
	{
		runCode=1;
	}
  //Else run the User code
	else
	{
		runCode=0;
	}
}


//After creating a setup() function, which initializes and sets the initial values, 
//the loop() function loops consecutively, allowing your program to change and respond. 
//Use it to actively control the Arduino board.
void loop()
{ 
  //If runcode is 1 it will execute the PC controlled code
	if (runCode ==1)
	{
		PCControlledCode();
	}
 
  //Else it will execute the User code
	else
	{
		UserCode();
	LEDToggle();
	}
}

//TO receive the commands from the COM port and execute them accordingly
void PCControlledCode(void)
{ 
  //Returns the number of milliseconds passed since the Arduino board began running the current program
	currentMillis = millis();

  //To check if any data is available on the COM port(Serial port)
  //Using IF-ELSE ladder to determin State
	if(Serial.available()>0)
	{ 
    //Reading the data from the COM port and assigning it to c
		c=Serial.read();

    //Assigning the current time elapsed to perivious time elapsed
		previousMillis = currentMillis;

    //To check if the state is equal to State_start(0)
		if(state==State_Start)
		{ 
      //To check if the data read from the COM port is equal to 170
			if(c==170)    
				state=State_Command;  //Changing the State to State_command                    
		}
    //To check if the state is equal to State_servoPosition (set servo position)
		else if(state==State_Servoposition)
		{ 
      //Saving the data command from the COM port into a buffer array
			servobuf[bytecnt]=c;
      
      //incrimenting the index
			bytecnt++;

      //To check if the index has reached 36
			if(bytecnt==36)
			{ 
        //Ittrating through the commands (int values) for 18 Servo motors
				for(int i=1;i<19;i++)
				{ 
				  //Accessing index starting from 1 and then 3 and so on skipping 1 position
          //Accessing Lower Byte 
          //Left shifting by 7 that value and assigning it to servoval
					servoval=servobuf[(i*2)-1]<<7;

          //Accessing index starting from 0 and then 2 and so on skipping 1 position
          //Accessing Higher Byte 
          //Adding that value to the lower byte and assigning it to servoval
          //It is the step value command for the current ith servo(i=servo No)
					servoval=servoval+servobuf[(i*2)-2];

          //To Set the specified servo to the specified step value
					I2C_SERVOSET(i,servoval);
				}
        //Reseting the index back to 0 
				bytecnt=0;

        //Changing the state baack to state_start
				state=State_Start;

        //write "r" on the COM port
				Serial.write('r');

        //To check if the last command has executed and the salve board i responsive
				CheckEndMovement();

        //Assigning the current time elapsed to perivious time elapsed
				previousMillis = currentMillis; 
			}
		}
    //To check if the state is equal to State_Speed (set servo speed)               
		else if(state==State_Speed)
		{ 
		  //To Set the specified speed value for the servos received from the COM port 
			I2C_SERVOSPEED(c);
      
      //Changing the state baack to state_start
			state=State_Start;

      //write "rs" on the COM port
			Serial.write("rs");

      //To check if the last command has executed and the salve board i responsive
			CheckEndMovement();
		}
    //To check if the state is equal to State_Servomin (set servo min position)
		else if(state==State_Servomin)
		{ 
      //Saving the data command from the COM port into a buffer array
			servobuf[bytecnt]=c;

      //incrimenting the index
			bytecnt++;    

      //To check if the index has reached 36                
			if(bytecnt==36)
			{ 
        //Ittrating through the commands (int values) for 18 Servo motors
				for(int i=1;i<19;i++)
				{ 
          //Accessing index starting from 1 and then 3 and so on skipping 1 position
          //Accessing Lower Byte 
          //Left shifting by 7 that value and assigning it to servoval
					servoval=servobuf[(i*2)-1]<<7;

          //Accessing index starting from 0 and then 2 and so on skipping 1 position
          //Accessing Higher Byte 
          //Adding that value to the lower byte and assigning it to servoval
          //It is the minimum step value command for the current servo(i=servo No)
					servoval=servoval+servobuf[(i*2)-2];

          //To Set the specified servo to the specified minimum step value
					I2C_SERVOMIN(i,servoval);
					delay(20);
				}
        //Reseting the index back to 0 
				bytecnt=0;

        //Changing the state baack to state_start
				state=State_Start;

        //write "rn" on the COM port
				Serial.write("rn");

        //To check if the last command has executed and the salve board i responsive
				CheckEndMovement();

        //Assigning the current time elapsed to perivious time elapsed
				previousMillis = currentMillis; 
			}
		}
    //To check if the state is equal to State_Servomax (set servo max position)     
		else if(state==State_Servomax)
		{ 
      //Saving the data command from the COM port into a buffer array
			servobuf[bytecnt]=c;
      
      //incrimenting the index
			bytecnt++;

      //To check if the index has reached 36                      
			if(bytecnt==36)
			{ 
        //Ittrating through the commands (int values) for 18 Servo motors
				for(int i=1;i<19;i++)
				{ 
          //Accessing index starting from 1 and then 3 and so on skipping 1 position
          //Accessing Lower Byte 
          //Left shifting by 7 that value and assigning it to servoval
					servoval=servobuf[(i*2)-1]<<7;

          //Accessing index starting from 0 and then 2 and so on skipping 1 position
          //Accessing Higher Byte 
          //Adding that value to the lower byte and assigning it to servoval
          //It is the Maximum step value command for the current ith servo(i=servo No)
					servoval=servoval+servobuf[(i*2)-2];

          //To Set the specified servo to the specified maximum step value
					I2C_SERVOMAX(i,servoval);
					delay(20);
				}
        //Reseting the index back to 0 
				bytecnt=0;

        //Changing the state baack to state_start
				state=State_Start;

        //write "rx" on the COM port
				Serial.write("rx");

        //To check if the last command has executed and the salve board i responsive
				CheckEndMovement();

        //Assigning the current time elapsed to perivious time elapsed
				previousMillis = currentMillis;
			}  
		}
    //To check if the state is equal to State_Servooffset (set servo offSet)             
		else if(state==State_Servooffset)
		{ 
      //Saving the data command from the COM port into a buffer array
			servobuf[bytecnt]=c;

      //incrimenting the index
			bytecnt++;   

      //To check if the index has reached 36                 
			if(bytecnt==36)
			{ 
        //Ittrating through the commands (int values) for 18 Servo motors
				for(int i=1;i<19;i++)
				{ 
          //Accessing index starting from 1 and then 3 and so on skipping 1 position
          //Accessing Lower Byte 
          //Left shifting by 7 that value and assigning it to servoval
					servoval=servobuf[(i*2)-1]<<7;

          //Accessing index starting from 0 and then 2 and so on skipping 1 position
          //Accessing Higher Byte 
          //Adding that value to the lower byte and assigning it to servoval
          //It is the offSet step value command for the current ith servo(i=servo No)
					servoval=servoval+servobuf[(i*2)-2];

          //To Set the specified servo to the specified offSet step value
          I2C_SERVOOFFSET(i,servoval);
					delay(20);
				}
        //Reseting the index back to 0  
				bytecnt=0;

        //Changing the state baack to state_start
				state=State_Start;

        //write "ro" on the COM port
				Serial.write("ro");

        //To check if the last command has executed and the salve board i responsive
				CheckEndMovement();

        //Assigning the current time elapsed to perivious time elapsed
				previousMillis = currentMillis; 
			} 
		}
    //To check if the state is equal to State_Servoreverse(set servo direction to reverse)     
		else if(state==State_Servoreverse)
		{ 
      //Saving the data command from the COM port into a buffer array
			servobuf[bytecnt]=c;

      //incrimenting the index
			bytecnt++; 

      //To check if the index has reached 18                       
			if(bytecnt==18)
			{ 
        //Ittrating through the commands (int values) for 18 Servo motors
				for(int i=1;i<19;i++)
				{ 
          //To Set the specified servo to the reverse direction
					I2C_SERVOREVERSE(i,servobuf[(i-1)]);
					delay(20);
				}
        //Reseting the index back to 0  
				bytecnt=0;

        //Changing the state baack to state_start
				state=State_Start;

        //write "rv" on the COM port
				Serial.write("rv");

        //To check if the last command has executed and the salve board i responsive
				CheckEndMovement();

        //Assigning the current time elapsed to perivious time elapsed
				previousMillis = currentMillis;
			} 
		}
    //To check if the state is equal to State_Servonutral(set servo nutral position)           
		else if(state==State_Servonutral)
		{ 
      //Saving the data command from the COM port into a buffer array
			servobuf[bytecnt]=c;

      //incrimenting the index
			bytecnt++;                    

      //To check if the index has reached 36 
			if(bytecnt==36)
			{ 
        //Ittrating through the commands (int values) for 18 Servo motors
				for(int i=1;i<19;i++)
				{ 
          //Accessing index starting from 1 and then 3 and so on skipping 1 position
          //Accessing Lower Byte 
          //Left shifting by 7 that value and assigning it to servoval
					servoval=servobuf[(i*2)-1]<<7;

          //Accessing index starting from 0 and then 2 and so on skipping 1 position
          //Accessing Higher Byte 
          //Adding that value to the lower byte and assigning it to servoval
          //It is the nutural step value command for the current ith servo(i=servo No)
					servoval=servoval+servobuf[(i*2)-2];

          //To Set the specified servo to the specified nutural position
					I2C_SERVONUTRALSET(i,servoval);
				}

        //Reseting the index back to 0 
				bytecnt=0;

        //Changing the state baack to state_start
				state=State_Start;

        //write "rn" on the COM port
				Serial.write("rn");

        //To check if the last command has executed and the salve board i responsive
				CheckEndMovement();

        //Assigning the current time elapsed to perivious time elapsed
				previousMillis = currentMillis; 
			}  
		}
    //To check if the state is equal to State_ReadOffsets (Read servo offSet/Position)      
		else if(state==State_ReadOffsets)
		{ 
      //write "O" on the COM port
			Serial.write("O");

      //Ittrating through the 18 Servo motors 
			for(int i=1;i<19;i++)
			{ 
        //Printing there position on the COM port
				Serial.print(I2C_SERVOGETOFFSET(i));
				Serial.write(",");
				delay(1);
			}

      //write "F" on the COM port
			Serial.write("F");

      //Changing the state baack to state_start
			state=State_Start;

      //write "O" on the COM port
			Serial.write('r');

      //To check if the last command has executed and the salve board i responsive
			CheckEndMovement();

      //Assigning the current time elapsed to perivious time elapsed
			previousMillis = currentMillis;
		}
    //To check if the state is equal to electroMagnet_set(set the electromagnet ON/OFF)    
    else if(state==electroMagnet_set)
		{ 
      //TO set the electromagnet to the specified ON/OFF command
			electroMagnet(c);

      //Changing the state baack to state_start
      state=State_Start;

      //write "k" on the COM port
      Serial.write('k');

      //To check if the last command has executed and the salve board i responsive
      CheckEndMovement();
		}
    //To check if the state is equal to stepper_speed_set(set the Stepper motor/Conveyor speed)   
    else if(state==stepper_speed_set)
    { 
      //TO set the Stepper motor/Conveyor to the specified speed command
      stepperSpeed(c);

      //Changing the state baack to state_start
      state=State_Start;

      //write "k" on the COM port
      Serial.write('k');

      //To check if the last command has executed and the salve board i responsive
      CheckEndMovement();
    }
    //To check if the state is equal to State_Command(to set the state to the specified state command from the COM port) 
		else if(state==State_Command)
		{   
        //To check if the state command from the COM port is equal to 8
			  if(c==8)
				state=State_Speed;
        
        //To check if the state command from the COM port is equal to 10
			  if(c==10)
				state=State_Servoposition;
        
        //To check if the state command from the COM port is equal to 11
			  if(c==11)
				state=State_Servomin;
       
        //To check if the state command from the COM port is equal to 12
			  if(c==12)
				state=State_Servomax;
       
        //To check if the state command from the COM port is equal to 13
			  if(c==13)
				state=State_Servooffset;
        
        //To check if the state command from the COM port is equal to 14
			  if(c==14)
				state=State_Servoreverse;   

        //To check if the state command from the COM port is equal to 15
			  if(c==15)
				state=State_Servonutral;

        //To check if the state command from the COM port is equal to 16
        if(c==16)
        state=electroMagnet_set;

        //To check if the state command from the COM port is equal to 17
        if(c==17)
        state=stepper_speed_set;

        //To check if the state command from the COM port is equal to 21
			  if(c==21)
				state=State_ReadOffsets;

        //Reseting the index back to 0 
			  bytecnt=0;                     
		}
    //Else 
		else
		{ 
      //write "e" on the COM port
			Serial.write('e');

      //To check if the last command has executed and the salve board i responsive
			CheckEndMovement();
		}
	}
  //Else check elapsed time
	else
	{ 
	  //To check if the elapsed time is equal to 100miliseconds
		if ((unsigned long)(currentMillis - previousMillis) >= interval) 
		{ 
      //write "i" on the COM port
			Serial.write('i');
      
      //write "i" on the COM port
			Serial.write('r');

      //To check if the last command has executed and the salve board i responsive
			CheckEndMovement();

      //Assigning the current time elapsed to perivious time elapsed
			previousMillis = currentMillis;
		}
	}  
}

//To check if the last command has executed and the salve board i responsive
void CheckEndMovement(void)
{ 
  //To check if the slave board has executed the last command and is responsive
	if(I2C_SERVOEND())
		Serial.write('X');//Writing 'X' to the COM port
  //Else 
	else
		Serial.write('G');//Writing 'G' to the COM port
  
  //To toggle the LED
	LEDToggle();
}

//To set the specified servo to the specified step value
void I2C_SERVOSET(unsigned char servo_num,unsigned int servo_pos)
{ 
  //To check if the step value is less then 500
	if(servo_pos<500)
		servo_pos = 500; //Making it 500

  //To check if the step value is greater then 2500
	else if(servo_pos>2500)
		servo_pos=2500; //Making it 2500

  //To check if the step value is less then 501
	if(servo_pos>501)
		servo_pos=(((servo_pos-2)*2)-1000);//Normalising 500-2500 to 0-3996
	else
		servo_pos=0;//Making it zero

  //To check if the servo number is samller then 19
  //To select slave board 1 
	if(servo_num<19)
		Wire.beginTransmission(servo1);//Starting the transmission to using I2C wire library, sending (8) 
  
  //Else select slave board 2 
	else
		Wire.beginTransmission(servo2);//Starting the transmission to using I2C wire library, sending (9)

  //Writing servo number to I2C
	Wire.write(servo_num-1);
	
	////Transmitting to board 1, sending servo step position value with left shift by 8
	Wire.write(servo_pos>>8);
	
	//Transmitting to board 1, sending servo step position value that is bitwise and with 0xFF to send one byte
	Wire.write(servo_pos & 0XFF);
	
	//Ending transmission to board 1
	Wire.endTransmission();
}

//To set the min step value for the specified servo 
void I2C_SERVOMIN(unsigned char servo_num,unsigned int servo_pos)
{ 
  //To check if the step value is less then 500
	if(servo_pos<500)
		servo_pos = 500;

  //To check if the step value is greater then 2500
	else if(servo_pos>2500)
		servo_pos=2500;

  //Normalising 500-2500 to 0-4000
	servo_pos=((servo_pos*2)-1000);

  //To check if the servo number is samller then 19
  //To select slave board 1
	if(servo_num<19)
		Wire.beginTransmission(servo1);//Starting the transmission to using I2C wire library, sending (8)
   
  //Else select slave board 2   
	else
		Wire.beginTransmission(servo2);//Starting the transmission to using I2C wire library, sending (9)

  //Transmitting to board 1, sending (72)+servo number
	Wire.write((servo_num-1)+(18*4));

  //Transmitting to board 1, sending servo min position value with left shift by 8
	Wire.write(servo_pos>>8);
	
	//Transmitting to board 1, sending servo min position value that is bitwise and with 0xFF to send one byte
	Wire.write(servo_pos & 0XFF);
	
	//Ending transmission to board 1
	Wire.endTransmission();
	
	//Delay
	delay(20);
}

//To set the max step value for the specified servo
void I2C_SERVOMAX(unsigned char servo_num,unsigned int servo_pos)
{ 
  //To check if the step value is less then 500
	if(servo_pos<500)
		servo_pos = 500;//Making it 500

  //To check if the step value is greater then 2500
	else if(servo_pos>2500)
		servo_pos=2500;//Making it 2500

  //Normalising 500-2500 to 0-4000
	servo_pos=((servo_pos*2)-1000);

  //To check if the servo number is samller then 19
  //To select slave board 1
	if(servo_num<19)
		Wire.beginTransmission(servo1);//Starting the transmission to using I2C wire library, sending (8)

  //Else select slave board 2   
	else
		Wire.beginTransmission(servo2);//Starting the transmission to using I2C wire library, sending (9)

  //Transmitting to board 1, sending (54)+servo number
	Wire.write((servo_num-1)+(18*3));

  //Transmitting to board 1, sending servo max position value with left shift by 8
	Wire.write(servo_pos>>8);
	
	//Transmitting to board 1, sending servo max position value that is bitwise and with 0xFF to send one byte
	Wire.write(servo_pos & 0XFF);
	
	//Ending transmission to board 1
	Wire.endTransmission();
	
	//Delay
	delay(20);
}

//To set the min step value for the specified servo 
void I2C_SERVONUTRALSET(unsigned char servo_num,unsigned int servo_pos)
{ 
  //To check if the step value is less then 500
	if(servo_pos<500)
		servo_pos = 500;

  //To check if the step value is greater then 2500
	else if(servo_pos>2500)
		servo_pos=2500;

  //Normalising 500-2500 to 0-4000
	servo_pos=((servo_pos*2)-1000);
 
  //To check if the servo number is samller then 19
  //To select slave board 1
	if(servo_num<19)
		Wire.beginTransmission(servo1);//Starting the transmission to using I2C wire library, sending (8)

  //Else select slave board 2
	else
		Wire.beginTransmission(servo2);//Starting the transmission to using I2C wire library, sending (9)

  //Transmitting to board 1, sending (90)+servo number
	Wire.write((servo_num-1)+(18*5));

  //Transmitting to board 1, sending servo nutural position value with left shift by 8
	Wire.write(servo_pos>>8);
	
	//Transmitting to board 1, sending servo nutural position value that is bitwise and with 0xFF to send one byte 
	Wire.write(servo_pos & 0XFF);
	
	//Ending transmission to board 1
	Wire.endTransmission();
}

//To Set the specified speed value for the servos received from the COM port 
void I2C_SERVOSPEED(unsigned char value)
{ 
  //To select slave board 1
  //Starting the transmission to using I2C wire library, sending (8)
	Wire.beginTransmission(servo1);
 
  //Transmitting to board 1, sending (36)
	Wire.write(18*2);

  //Transmitting to board 1, sending speed value (0-100)
	Wire.write(value);

  //Transmitting to board 1, sending (0)
	Wire.write(0);

  //Ending transmission to board 1
	Wire.endTransmission();

  //To select slave board 2
  //Starting the transmission to using I2C wire library, sending (9)
	Wire.beginTransmission(servo2);

  //Transmitting to board 2, sending (36)
	Wire.write(18*2);

  //Transmitting to board 2, sending speed value (0-100)
	Wire.write(value);

  //Transmitting to board 1, sending (0)
	Wire.write(0);

  //Ending transmission to board 2
	Wire.endTransmission();

  //Delay
	delay(20);
}

//To set the specified servo to the specified offset step value
void I2C_SERVOOFFSET(unsigned char servo_num,int value)
{ 
  //Calculating Offset from the center at 1500, +ve offset values move towareds 500, -ve Offset values move towards 2500(total range 500-2500)
	value=3000-value;
	value=value-1500;

  //To check if the offset value is less then -500
	if (value<-500)
		value=-500;//Making it -500

  //To check if the offset value is greater then 500
	else if (value>500)
		value=500;//Making it 500

  //To check if the offset value is greater then 0
	if(value>0)
		value=2000+(value*2);

  //To check if the offset value is less then or equal to 0
	else if(value<=0)
		value=-value*2;

	//To select slave board 1
	if(servo_num<19)
		Wire.beginTransmission(servo1);//Starting the transmission to using I2C wire library, sending (8)

  //Else select slave board 2
	else
		Wire.beginTransmission(servo2);//Starting the transmission to using I2C wire library, sending (9)

  //Transmitting to board 1, sending (108)+servo number
	Wire.write((servo_num-1)+(18*6));

  //Transmitting to board 1, sending servo offset value with left shift by 8
	Wire.write(value>>8);
	
	//Transmitting to board 1, sending servo offset value that is bitwise and with 0xFF to send one byte 
	Wire.write(value & 0XFF);
	
	//Ending transmission to board 1
	Wire.endTransmission();
  
  //Delay
  delay(20);
}
//To reverse the direction of the specified servo
void I2C_SERVOREVERSE(unsigned char servo_num,unsigned char servo_dir)
{ 
  //To check if the direction command is >0
	if(servo_dir>0)
		servo_dir=1;//Making it 1

  //To select slave board 1
	if(servo_num<19)
		Wire.beginTransmission(servo1); //Starting the transmission to using I2C wire library, sending (8)

  //Else select slave board 2
	else
		Wire.beginTransmission(servo2); //Starting the transmission to using I2C wire library, sending (9)

  //Transmitting to board 1, sending (126)+servo number
	Wire.write((servo_num-1)+(18*7));

  //Transmitting to board 1, sending (1)
	Wire.write(servo_dir);
	
	//Transmitting to board 1, sending (0)
	Wire.write(0);
	
	//Ending transmission to board 1
	Wire.endTransmission();
  
  //delay
  delay(20);
}

//To check if the slave board has finished the last command or not
char I2C_SERVOEND(void)
{
	int i, n;
	char buffer;

  //Starting the transmission to using I2C wire library, sending (8)
	Wire.beginTransmission(servo1);
	n = Wire.write(181);
	if (n != 1)
		return (-10);

	n = Wire.endTransmission(false);
	if (n != 0)
		return (n);

  //Delay
	delayMicroseconds(350);
 
  //To request bytes from a peripheral device
	Wire.requestFrom(servo1, 1, true);

  //Waiting till the bytes are available
	while(Wire.available())
		buffer=Wire.read();//Saving the bytes in a buffer

  //Returning the buffer
	return(buffer);
}

//To get the current step position of the specified servo
int I2C_SERVOGET(int servo_num)
{
	int i, n, error;
	uint8_t buffer[2];

  //Starting the transmission to using I2C wire library, sending (8)
	Wire.beginTransmission(servo1);

  //Transmitting to board 1, sending (144)+servo number
	n = Wire.write((servo_num-1)+(18*8));
	if (n != 1)
		return (-10);

	n = Wire.endTransmission(false);
	if (n != 0)
		return (n);

  //Delay
	delayMicroseconds(240);

  //To request bytes from a peripheral device
	Wire.requestFrom(servo1, 2, true);
	i = 0;

  //Waiting till the bytes are available and iid less then 2
	while(Wire.available() && i<2)
	{
		buffer[i++]=Wire.read();//Saving the bytes in a buffer
	}

  //To check if the i is not equal to 2
	if ( i != 2)
		return (-11);//Return -11
	return (((buffer[0]*256 + buffer[1])+4)/2 +500);//Return buffer
}


//To get the current offset of the specified servo
int I2C_SERVOGETOFFSET(int servo_num)
{
	int i, n, error;
	uint8_t buffer[2];

  //Starting the transmission to using I2C wire library, sending (8)
	Wire.beginTransmission(servo1);

  //Transmitting to board 1, sending (182)+servo number
	n = Wire.write((servo_num-1)+(182));
	if (n != 1)
		return (-10);

	n = Wire.endTransmission(false);
	if (n != 0)
		return (n);

  //Delay
	delayMicroseconds(240);
 
  //To request bytes from a peripheral device
	Wire.requestFrom(servo1, 2, true);
	i = 0;

  //Wait till the bytes are available and i is less then 2
	while(Wire.available() && i<2)
	{
		buffer[i++]=Wire.read();//Saving the bytes in a buffer
	}

  
	if ( i != 2)
		return (-11);
	i=((buffer[0]*256 + buffer[1]));
	if(i>2000)
		return(3000-(((i-2000)/2)+1500));
	else
		return(3000-((-i/2)+1500));
}

//To Set electroMagnet ON/OFF
void electroMagnet(int var)
{
  //To check if the var is equal to  1
  if(var==1)
  { 
    //To set the pin 7 HIGH(to turn on the relay)
    digitalWrite(7,HIGH);
  }

  //To check if the var is equal to  0
  else if(var==0)
  {
    //To set the pin 7 LOW(to turn off the relay)
    digitalWrite(7,LOW);
  }

}

//To genrate ON/OFF for the stepper motor
void stepper(void)
{ 
  //To set stepper pin HIGH
  digitalWrite( stepPin, HIGH );

  //delay
  delay(0.001);

  //To set stepper pin LOW
  digitalWrite( stepPin, LOW ); 
}

//To set the speed of the stepper motor (conveyor)
void stepperSpeed(int speed)
{ 
  //To check if the speed value is 101
  if(speed==101)
  { 
    //To pass (0) to the timer function, to stop the intrupt timer
    timer(0);
  }
  //To check if the value less then or equal to 100
  if(speed<=100)
  { 
    //To map (0-100) to (32750-200)
    //32750 being the lowest speed(intrupt timer delay)
    //200  being the highest speed(intrupt timer delay)
    delayInMicroSeconds=map(speed,0,100,32750,200);

    //To call the timer function and pass the intrupt timer delay
    timer(delayInMicroSeconds);
  }
  
}

//To set all the servos to the specified step values respectiely
void ServoSetAll(unsigned int Servo1, unsigned int Servo2, unsigned int Servo3, unsigned int Servo4, unsigned int Servo5, unsigned int Servo6, unsigned int Servo7, unsigned int Servo8, unsigned int Servo9, unsigned int Servo10, unsigned int Servo11, unsigned int Servo12, unsigned int Servo13, unsigned int Servo14, unsigned int Servo15, unsigned int Servo16, unsigned int Servo17, unsigned int Servo18)
{ 
  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo1 >= 500) {I2C_SERVOSET(1,Servo1);}
  
  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo2 >= 500) {I2C_SERVOSET(2,Servo2);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo3 >= 500) {I2C_SERVOSET(3,Servo3);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo4 >= 500) {I2C_SERVOSET(4,Servo4);}
  
  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo5 >= 500) {I2C_SERVOSET(5,Servo5);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo6 > 500) {I2C_SERVOSET(6,Servo6);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo7 >= 500) {I2C_SERVOSET(7,Servo7);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo8 >= 500) {I2C_SERVOSET(8,Servo8);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo9 >= 500) {I2C_SERVOSET(9,Servo9);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo10 >= 500) {I2C_SERVOSET(10,Servo10);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo11 >= 500) {I2C_SERVOSET(11,Servo11);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo12 >= 500) {I2C_SERVOSET(12,Servo12);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo13 >= 500) {I2C_SERVOSET(13,Servo13);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo14 >= 500) {I2C_SERVOSET(14,Servo14);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo15 >= 500) {I2C_SERVOSET(15,Servo15);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo16 >= 500) {I2C_SERVOSET(16,Servo16);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo17 >= 500) {I2C_SERVOSET(17,Servo17);}

  //To check if the specified step value is greater then or equal to 500 and setting the specified servo to the specified step value
	if (Servo18 >= 500) {I2C_SERVOSET(18,Servo18);}

  //Waiting for the slave board to respond
	while (!I2C_SERVOEND())
	{ 
    //Delay
		delay(1);
	}

  //To toggle LED on board
	LEDToggle();
}

//To toggle the LED on the board
void LEDToggle(void)
{ 
  //To check if the LED state is 0
	if (LEDState == 0)
		LEDState = 1;//Making it 1

  //Else making it 0
	else
		LEDState = 0;

  //TO make the LED pin HIGH/LOW(Pin 13)
	digitalWrite(LED, LEDState);
}

void UserCode(void)
{
  //To set the Configuration of the servo motors by calling the methods and passing desired values 
	//------------------------------Configuration------------------------------

	I2C_SERVOMAX(1,2500); I2C_SERVOMAX(2,2500); I2C_SERVOMAX(3,2500); I2C_SERVOMAX(4,2500); I2C_SERVOMAX(5,2500); I2C_SERVOMAX(6,2500); I2C_SERVOMAX(7,2500); I2C_SERVOMAX(8,2500); I2C_SERVOMAX(9,2500); I2C_SERVOMAX(10,2500); I2C_SERVOMAX(11,2500); I2C_SERVOMAX(12,2500); I2C_SERVOMAX(13,2500); I2C_SERVOMAX(14,2500); I2C_SERVOMAX(15,2500); I2C_SERVOMAX(16,2500); I2C_SERVOMAX(17,2500); I2C_SERVOMAX(18,2500);		//Maximum Values

	I2C_SERVOMIN(1,500); I2C_SERVOMIN(2,500); I2C_SERVOMIN(3,500); I2C_SERVOMIN(4,500); I2C_SERVOMIN(5,500); I2C_SERVOMIN(6,500); I2C_SERVOMIN(7,500); I2C_SERVOMIN(8,500); I2C_SERVOMIN(9,500); I2C_SERVOMIN(10,500); I2C_SERVOMIN(11,500); I2C_SERVOMIN(12,500); I2C_SERVOMIN(13,500); I2C_SERVOMIN(14,500); I2C_SERVOMIN(15,500); I2C_SERVOMIN(16,500); I2C_SERVOMIN(17,500); I2C_SERVOMIN(18,500);		//Minimum Values

	I2C_SERVOOFFSET(1,1500); I2C_SERVOOFFSET(2,1500); I2C_SERVOOFFSET(3,1500); I2C_SERVOOFFSET(4,1500); I2C_SERVOOFFSET(5,1500); I2C_SERVOOFFSET(6,1500); I2C_SERVOOFFSET(7,1500); I2C_SERVOOFFSET(8,1500); I2C_SERVOOFFSET(9,1500); I2C_SERVOOFFSET(10,1500); I2C_SERVOOFFSET(11,1500); I2C_SERVOOFFSET(12,1500); I2C_SERVOOFFSET(13,1500); I2C_SERVOOFFSET(14,1500); I2C_SERVOOFFSET(15,1500); I2C_SERVOOFFSET(16,1500); I2C_SERVOOFFSET(17,1500); I2C_SERVOOFFSET(18,1500);		//Offset Values

	I2C_SERVOREVERSE(1,0); I2C_SERVOREVERSE(2,0); I2C_SERVOREVERSE(3,0); I2C_SERVOREVERSE(4,0); I2C_SERVOREVERSE(5,0); I2C_SERVOREVERSE(6,0); I2C_SERVOREVERSE(7,0); I2C_SERVOREVERSE(8,0); I2C_SERVOREVERSE(9,0); I2C_SERVOREVERSE(10,0); I2C_SERVOREVERSE(11,0); I2C_SERVOREVERSE(12,0); I2C_SERVOREVERSE(13,0); I2C_SERVOREVERSE(14,0); I2C_SERVOREVERSE(15,0); I2C_SERVOREVERSE(16,0); I2C_SERVOREVERSE(17,0); I2C_SERVOREVERSE(18,0);		//Directions (Servo Reverse)

	//------------------------------Code Flow------------------------------

  //To set all the servos to the specified step values respectiely
	ServoSetAll(0,2117,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500);		// Line # 0

  //Default Delay
	delay(300);		

  //To set all the servos to the specified step values respectiely
	ServoSetAll(0,907,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500);		// Line # 1

  //Default Delay
	delay(300);	

  //Delay
	delay(1000);  
  
	// Comment or remove the next 5 lines to run code in loop...
	while(1)
	{ 
    //TO toggle the LED on the board
		LEDToggle();

    //Delay
		delay(100);
	}

}
