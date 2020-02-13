#include <IFCT.h>
#include <kinetis_flexcan.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_NAU8702

NAU7802 myScale; //Create instance of the NAU7802 class

long startupReading;    // used for zeroing
long currentReading;    // the actual reading

int8_t bitVar2;    
int8_t bitVar3;

int scaledReading;

//int msg.buf[1];
int led = 13;
// create CAN object
IFCT CAN0(500000);
static CAN_message_t msg;

void setup()
{

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000); //Qwiic Scale is capable of running at 400kHz if desired

  if (myScale.begin() == false)
  {
    Serial.println("Scale not detected. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Scale detected!");

  

  myScale.setSampleRate(NAU7802_SPS_320); //Increase to max sample rate
  myScale.calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channel   

  // init CAN bus
  CAN0.begin();
  Can0.setBaudRate(500000);
  pinMode(led, OUTPUT);

  delay (1000);

  startupReading = myScale.getReading();      // get startup value to zero offset

  delay (1000);

}

void loop()
{


  if (myScale.available() == true)
  {
    currentReading = myScale.getReading();
    currentReading = currentReading - startupReading;   // remove offset / zero the scale

    scaledReading = currentReading / 10;    // make the number smal enough it's a 16-bit value

    scaledReading = constrain(scaledReading,0,65535);   // constrain the value to 16-bit

    bitVar2 = lowByte(scaledReading);                 // chop it into two 8-bit values
    bitVar3 = highByte(scaledReading);

    Serial.print("Reading: ");
    Serial.print(currentReading);
    Serial.print(" , ");
    Serial.print(scaledReading);

    Serial.print("  Sending: ");
    msg.id = 0x222;
    msg.len = 2;
    msg.buf[0] = bitVar2;
    msg.buf[1] = bitVar3;
    Serial.println("");
    CAN0.write(msg);
    digitalWrite(led, !digitalRead(led));
    delay(1);

  }

  delay(5);
}



