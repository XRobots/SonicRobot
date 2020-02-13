#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary
#include <Servo.h>
#include <IFCT.h>

Servo servo1;
Servo servo2;

double Pk1 = 7500;          // balancing PID
double Ik1 = 75000;
double Dk1 = 180;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup - balancing IMU

double Pk2 = 3;          // Side to Side PID
double Ik2 = 0;
double Dk2 = 0;

double Setpoint2, Input2, Output2;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup - side to side IMU

double Pk3 = 22;          // load cell PID
double Ik3 = 0;
double Dk3 = 0;

double Setpoint3, Input3, Output3;    // PID variables
PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);    // PID Setup - load cell 1

double Pk4 = 22;          // load cell PID
double Ik4 = 0;
double Dk4 = 0;

double Setpoint4, Input4, Output4;    // PID variables
PID PID4(&Input4, &Output4, &Setpoint4, Pk4, Ik4 , Dk4, DIRECT);    // PID Setup - loadcell 2

RF24 radio(25, 24); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <ODriveArduino.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//ODrive Object
ODriveArduino odrive(Serial2);
ODriveArduino odrive2(Serial3);

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO 

    int16_t menuDown;      
    int16_t Select; 
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1; 
    int16_t toggle2; 
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
    int16_t checkit;
    int16_t checkit2;

};

int stickRFB;
int stickRLR;
int stickLLR;
int toggleTop;
int toggleBottom;

RECEIVE_DATA_STRUCTURE mydata_remote;

int requested_state;   

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define LED_PIN 13 // (Arduino is 11, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


float pitch;
float roll;
long velocity;

int menuFlag = 0;
float SetpointTrim1 = 0;

float accum = 0;
float accum2 = 0;
float outputDiv;

float trimPot;
float trimAngle;

float RFB;
int RLR;
int LT;
int LLR;

long drive1;
long drive2;

int switch1;
int switch2;

int toggle1;
int toggle2;

float leg1Dyn;
float leg2Dyn;
float leg1;
float leg2;
float leg1Filtered;
float leg2Filtered;

float leg3;
float leg4;
float leg3Filtered;
float leg4Filtered;

float leg1Output;
float leg2Output;

float legs;
float legs2;

float arm1;
float arm2;

float arm1Filtered;
float arm2Filtered;

unsigned long currentMillis;

long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timers
unsigned long count;

int loopTime;
int previousLooptime;

unsigned long previousSafetyMillis;

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

int bitVar1;    // CAN bus receive
int bitVar2;
int bitVar1Z = 0;    // CAN bus receive
int bitVar2Z = 0;

// ****************** SETUP ******************************

void setup() {

    // radio stuff

    radio.begin();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001
    radio.setPALevel(RF24_PA_MIN);

    radio.startListening();
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(100000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);

    //ODrive serial port
    Serial2.begin(115200);
    Serial3.begin(115200);

    // initialize IMU 
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(176);
    mpu.setYGyroOffset(-31);
    mpu.setZGyroOffset(57);
    mpu.setXAccelOffset(-2424);
    mpu.setYAccelOffset(272);
    mpu.setZAccelOffset(987);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    Can0.setBaudRate(500000);     // CAN Bus device
    Can0.enableFIFO();

    // Setup PID controllers

    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-360000, 360000);
    PID1.SetSampleTime(10);

    PID2.SetMode(AUTOMATIC);              
    PID2.SetOutputLimits(-120000, 120000);
    PID2.SetSampleTime(10);

    PID3.SetMode(AUTOMATIC);              
    PID3.SetOutputLimits(-80000, 80000);
    PID3.SetSampleTime(10);

    PID4.SetMode(AUTOMATIC);              
    PID4.SetOutputLimits(-80000, 80000);
    PID4.SetSampleTime(10);

    //switch inputs 
    
    pinMode(22, INPUT_PULLUP);
    pinMode(23, INPUT_PULLUP);

    servo1.attach(26);    // arm servos 
    servo2.attach(27);
    servo1.write(90);     // default positons
    servo2.write(90);

}

// ********************* MAIN LOOP *******************************

void loop() {  

        CAN_message_t msg;
        if ( Can0.read(msg) ) canSniff(msg);
      
        currentMillis = millis();
        if (currentMillis - previousMillis >= 10) {  // start timed event
          
        previousMillis = currentMillis; 

        // measure loop time / optional

        loopTime = currentMillis - previousLooptime;
        //Serial.print("time: ");
        //Serial.print(loopTime);
        //Serial.print("\t");
        previousLooptime = currentMillis;    

        // receive readio data
        
        if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
                    previousSafetyMillis = currentMillis; 
        }

        // check for dodgy radio issues

        if (mydata_remote.checkit == 1 && mydata_remote.checkit2 == 100) { 

            stickLLR = mydata_remote.LLR;
            stickRFB = mydata_remote.RFB;
            stickRLR = mydata_remote.RLR;
            toggleTop = mydata_remote.toggleTop;
            toggleBottom = mydata_remote.toggleBottom;
            toggle1 = mydata_remote.toggle1;
            toggle2 = mydata_remote.toggle2;
        }

       // check if remote has become disconnected

        if(currentMillis - previousSafetyMillis > 200) {         
            Serial.println("*no data* ");
            stickRLR = 512;
            stickRFB = 512;
            stickLLR = 512;
        }    

        // read switches

        switch1 = digitalRead(22);    // init
        switch2 = digitalRead(23);    // init2


        // init ODrives

        if (switch1 == 0) {     // init Odrive
            OdriveInit();
        }

        if (switch2 == 0) {     // init Odrive
            OdriveInit2();
        }

        // deal with remote scaling

        RFB = map(stickRFB,0,1023,1023,0);   
        LLR = map(stickLLR,0,1023,1023,0);   

        RFB = (float) (RFB - 512) / 120;
        LLR = ((LLR - 512)  * 100);

        // threshhold drive stick        

        if (RFB <= 0.25 && RFB >= -0.25) {
            RFB = 0;
        }  

        // digital trim

        if (mydata_remote.menuDown == 1 && menuFlag == 0) {
            SetpointTrim1 = SetpointTrim1 - 0.1;
            menuFlag = 1;
        }

        else if (mydata_remote.menuUp == 1 && menuFlag == 0) {
            SetpointTrim1 = SetpointTrim1 + 0.1;
            menuFlag = 1;
        }

        if (mydata_remote.menuDown == 0 && mydata_remote.menuUp == 0) {
            menuFlag = 0;
        }

        // threshold steering stick

        if (LLR < 5000 && LLR > -5000) {
          LLR = 0;
        }

        if (LLR < -5000) {
          LLR = LLR + 5000;
        }

        else if (LLR > 5000) {
          LLR = LLR - 5000;
        }

        // get two numbers for the two leg actuators

        legs2 = map(stickRLR,0,1023,1023,0);      // right hand stick
        legs2 = ((legs2 - 512)  * 100);

        legs = map(stickLLR,0,1023,1023,0);       // left hand stick
        legs = ((legs - 512)  * 100);

        legs = legs + legs2;                      // sum both

        // threshhold legs sticks
        
        if (legs < 5000 && legs > -5000) {
          legs = 0;
        }

        if (legs < -5000) {
          legs =  legs + 5000;
        }

        else if (legs > 5000) {
          legs = legs - 5000;
        }

        if (toggle2 == 1) {       // use the IMU for stability
            Input2 = roll*3000;
        }

        else if (toggle2 == 0) {       // don't use the IMU for stability
            Input2 = 0;
        }

        if (toggle1 == 1) {       // use the sticks to control the leg actuators
            Setpoint2 = legs;
        }

        else if (toggle1 == 0) {       // don't use the sticks to control the leg actuators
            Setpoint2 = 0;
        }

        
        PID2.Compute();     
        
        // move the legs       

        if (Output2 > 0) {
          leg1 = (Output2);
        }
        else {
          leg1 = 0;
        }
  
        if (Output2 < 0) {
          leg2 = abs(Output2);
        }  
        else {
          leg2 = 0;
        }

        // filter leg motions - manual steering
        
        leg1Filtered = filter(leg1, leg1Filtered, 50);
        leg2Filtered = filter(leg2, leg2Filtered, 50);

        // filter leg motions - loadcells
        
        leg3Filtered = filter(Output3, leg3Filtered, 22);
        leg4Filtered = filter(Output4, leg4Filtered, 22);

        // combine leg data

        if (toggleBottom == 1){     // turn on leg load cell data

            leg1Output = leg1Filtered + leg4Filtered;
            leg2Output = leg2Filtered + leg3Filtered;
        }

        else if (toggleBottom == 0) {   // turn off leg load cell data
          
            leg1Output = leg1Filtered;
            leg2Output = leg2Filtered;
        }        


        // zero CAN bus data when the switch is pressed

        if (mydata_remote.Select == 1) {
          bitVar1Z = bitVar1;
          bitVar2Z = bitVar2;
        }

        // receive CAN data and combines bits

        bitVar1 = bitVar1 - bitVar1Z;
        bitVar2 = bitVar2 - bitVar2Z;

        bitVar1 = constrain(bitVar1,0,8000);
        bitVar2 = constrain(bitVar2,0,8000);

        // leg actuator stuff

        //Setpoint3 = roll * 200;
        //Setpoint4 = roll * -200;

        Setpoint3 = 0;
        Setpoint4 = 0;
        
        Input3 = bitVar1*-1;    // invert data
        Input4 = bitVar2*-1;

        PID3.Compute();
        PID4.Compute();

        // see if IMU data is ready based on the interrupt
          
        if (IMUdataReady == 1) {
          readAngles();             // call the function on the other tab to get the IMU data
        }

        // 'agressive driving' stuff - *****experimental*****

        outputDiv = Output1 / 80000;   // 300000
        accum =  (outputDiv + RFB);
        accum = constrain(accum,-3,3);

        // convert angles to degrees
       
        pitch = (ypr[1] * 180/M_PI);           
        roll = (ypr[2] * 180/M_PI)+2;      

        // balancing PID stuff

        Setpoint1 = RFB + SetpointTrim1 + accum;
        Input1 = pitch;
        PID1.Compute();       

        //steering - differential drive
        
        drive1 = Output1 + LLR;
        drive2 = (Output1*-1) + LLR;  

        // drive ODrives

        if (toggleTop == 1) {     // motor enable
                        
              odrive.SetVelocity(0, drive1); 
              odrive.SetVelocity(1, drive2);
        }

        else if (toggleTop == 0) {     // motor dis-enable

              odrive.SetVelocity(0, 0); 
              odrive.SetVelocity(1, 0);
        }

        leg1Output = constrain(leg1Output, 0,100000);   // make sure we don't run the end stops
        leg2Output = constrain(leg2Output, 0,100000);

        odrive2.SetPosition(1, leg1Output);       // drive leg actuators
        odrive2.SetPosition(0, leg2Output); 

        // drive arm servos

        arm1 = map(stickLLR,0,1023,180,0);
        arm2 = map(stickLLR,0,1023,180,0);

        arm1Filtered = filter(arm1, arm1Filtered, 15);
        arm2Filtered = filter(arm2, arm2Filtered, 15);

        servo1.write(arm1Filtered);
        servo2.write(arm2Filtered);


      }     // end of timed event          
   
}           // end  of main loop


// filter function

float filter(float lengthOrig, float currentValue, int filter) {
  float lengthFiltered =  (lengthOrig + (currentValue * filter)) / (filter + 1);
  return lengthFiltered;  
}

// CAN Bus receive function

void canSniff(const CAN_message_t &msg) {
  //Serial.print(" ID: "); Serial.print(msg.id, HEX);
  //Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    //Serial.print(msg.buf[i], DEC); Serial.print(" ");
  }

  if (msg.id == 0x222) {
      bitVar1 = ((msg.buf[1] * 256) + msg.buf[0]*1.5);   // right leg - scale for difference in values
  }
  else if (msg.id == 0x221) {
      bitVar2 = ((msg.buf[1] * 256) + msg.buf[0]*1.5);   // left leg - scale for difference in values
  }
  
}



