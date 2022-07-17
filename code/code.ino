/* Used to initialize SD card*/
#include "SdFat.h"
#include <SPI.h>

/*Used to initialize time module*/
#include "RTClib.h"

/*Used to initialize Accelerometer*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


//------------------------------------------------------------------------------
// SD module
#define SD_CS_PIN SS
SdFat SD;
File myFile;
int loop_output[20][6];
int i = 0;
int k = 0;

//------------------------------------------------------------------------------
// Time module
RTC_DS3231 rtc;

//------------------------------------------------------------------------------
// Accelerometer module
MPU6050 mpu(0x69);

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container

volatile bool record = false;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
//------------------------------------------------------------------------------

void setup() {
  // join I2C bus
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  Serial.begin(9600);
  
  // Wait for USB Serial
  while (!Serial) {
    yield();
  }

  //------------------------------------------------------------------------------
  // SD module
  // initialize SD card
  Serial.print(F("Initializing SD card..."));

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("initialization failed!"));
    return;
  }
  Serial.println(F("initialization done."));

  // open the file
  myFile = SD.open("output.txt", FILE_WRITE);

  //------------------------------------------------------------------------------
  // Time module
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
    while (1) delay(10);
  }

  // output the date it was started
  DateTime now = rtc.now();
  myFile.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  myFile.flush();
  
  //------------------------------------------------------------------------------
  // Accelerometer module

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

    //Free Fall interrupt
    mpu.setIntMotionEnabled(1);
    mpu.setMotionDetectionThreshold(5);
    mpu.setMotionDetectionDuration(1);
    mpu.setDHPFMode(7);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
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
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    if (k == 0) {
      if (mpu.getIntMotionStatus() == 1) {
        record = true;
      }
    }     

      
    if(record == true) {
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
    
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      
      loop_output[i][0] = aaReal.x;
      loop_output[i][1] = aaReal.y;
      loop_output[i][2] = aaReal.z;
      loop_output[i][3] = (euler[0] * 180/M_PI) * 100;
      loop_output[i][4] = (euler[1] * 180/M_PI) * 100;
      loop_output[i][5] = (euler[2] * 180/M_PI) * 100;
     
      if (i == 19) {
        //Write to file
        k++;
        for (int j = 0; j < 20; j++) {
          myFile.print(loop_output[j][0]);
          myFile.print(F("\t"));
          myFile.print(loop_output[j][1]);
          myFile.print(F("\t"));
          myFile.print(loop_output[j][2]);
          myFile.print(F("\t"));
          myFile.print(loop_output[j][3]);
          myFile.print(F("\t"));
          myFile.print(loop_output[j][4]);
          myFile.print(F("\t"));
          myFile.print(loop_output[j][5]);
          
          if ((k == 50 or k == 1) and j == 0) {
            DateTime now = rtc.now();
            
            myFile.print(F("\t"));
            myFile.print(now.timestamp(DateTime::TIMESTAMP_TIME));
            myFile.print(F("\t"));
            myFile.print(rtc.getTemperature());
            myFile.println(F(" C"));
            
            if(k == 50) {
              record = false;
              k = 0;
            }
          }
          else {
            myFile.println();
          }
        }
        myFile.flush();
        i = 0;
      } else {
        i++;
      }
      
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
  }
}
