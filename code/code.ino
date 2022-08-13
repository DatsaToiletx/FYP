/* Used to initialize SD card*/
#include "SdFat.h"
#include <SPI.h>
#define SD_CS_PIN SS
SdFat SD;
File myFile;
//int loop_output[15][6];
//int i = 0;
//int k = 0;

/*Used to initialize time module*/
#include "RTClib.h"
RTC_DS3231 rtc;

/*Used to initialize Accelerometer*/
#include <avr/sleep.h>

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;


#define SIGNAL_PATH_RESET 0x68
#define I2C_SLV0_ADDR 0x37
#define ACCEL_CONFIG 0x1C
#define MOT_THR 0x1F // Motion detection threshold bits [7:0]
#define MOT_DUR 0x20 // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MOT_DETECT_CTRL 0x69
#define INT_ENABLE 0x38
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68
#define INT_STATUS 0x3A
#define MPU6050_ADDRESS 0x69 // Device address when ADO = 1

int wakePin = 2; // pin used for waking up  
int led = 13;
volatile int count = 0;

// Interrupt function
void wakeUpNow() { 
  Serial.println("WOKEN UP !!!!!!!!!!");
  count = 200;
}

// Used to configure accelerometer
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.begin();
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress); // Put slave register address in Tx buffer
  Wire.write(data); // Put data in Tx buffer
  Wire.endTransmission(); // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress); // Put slave register address in Tx buffer
  Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1); // Read one byte from slave register address 
  data = Wire.read(); // Fill Rx buffer with result
  return data; // Return data read from slave register
}

void setup() {
  Serial.begin(115200);
  //--------------------------------------------------------------------------------
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
  rtc.begin();

  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, let's set the time!"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // output the date it was started
  DateTime now = rtc.now();
  myFile.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  myFile.flush();

  //------------------------------------------------------------------------------
  // Accelerometer module
  // Initialize mpu
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Set setting for mpu
  writeByte(MPU6050_ADDRESS, 0x6B, 0x00);
  writeByte(MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0x07); //Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
  writeByte(MPU6050_ADDRESS, I2C_SLV0_ADDR, 0x20); //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x01); //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
  writeByte(MPU6050_ADDRESS, MOT_THR, 5); //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).  
  writeByte(MPU6050_ADDRESS, MOT_DUR, 10); //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate  
  writeByte(MPU6050_ADDRESS, MOT_DETECT_CTRL, 0x15); //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )   
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x40); //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.     
  writeByte(MPU6050_ADDRESS, 0x37, 0); // now INT pin is active low

//  Serial.println(readByte(MPU6050_ADDRESS, 0x6B));
//  Serial.println(readByte(MPU6050_ADDRESS, 0x68));
//  Serial.println(readByte(MPU6050_ADDRESS, 0x37));
//  Serial.println(readByte(MPU6050_ADDRESS, 0x1C));
//  Serial.println(readByte(MPU6050_ADDRESS, 0x1F));
//  Serial.println(readByte(MPU6050_ADDRESS, 0x20));
//  Serial.println(readByte(MPU6050_ADDRESS, 0x69));
//  Serial.println(readByte(MPU6050_ADDRESS, 0x38));
  
  pinMode(2, INPUT); // sets the digital pin 7 as input

  pinMode(wakePin, INPUT_PULLUP); // wakePin is pin no. 2
  pinMode(led, OUTPUT); //   led is pin no. 13
  
  // attachInterrupt(0, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function wakeUpNow when pin 2 gets LOW
}

void sleepNow() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here  
  sleep_enable(); // enables the sleep bit in the mcucr register  
  delay(500);
  Serial.println("About to sleep");
  delay(500);
  attachInterrupt(digitalPinToInterrupt(2), wakeUpNow, CHANGE); // use interrupt 0 (pin 2) and run function  
  delay(500);
  Serial.println("Interupt attached");
  delay(500);
  sleep_mode(); // here the device is actually put to sleep...!!

  // THE PROGRAM CONTINUES FROM HERE AFTER INTERRUPT IS CLOSED
  delay(500);
  Serial.println("Continuing main program after interupt");
  delay(500);

  sleep_disable(); // first thing after waking from sleep: disable sl¯eep...  
  delay(500);
  Serial.println("Sleep disabled");
  delay(500);
}

void loop() {
  if (count != 0) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
  
    /* Print out the values */
    Serial.print(count);
    Serial.print("\t");
    Serial.print(a.acceleration.x);
    Serial.print("\t");
    Serial.print(a.acceleration.y);
    Serial.print("\t");
    Serial.print(a.acceleration.z);
    Serial.print("\t");
  
    Serial.print(g.gyro.x);
    Serial.print("\t");
    Serial.print(g.gyro.y);
    Serial.print("\t");
    Serial.print(g.gyro.z);
    Serial.print("\t");
    
    Serial.print(temp.temperature);
    Serial.println(" degC");

    myFile.print("\t");
    myFile.print(a.acceleration.x);
    myFile.print("\t");
    myFile.print(a.acceleration.y);
    myFile.print("\t");
    myFile.print(a.acceleration.z);
    myFile.print("\t");
    myFile.print(g.gyro.x);
    myFile.print("\t");
    myFile.print(g.gyro.y);
    myFile.print("\t");
    myFile.print(g.gyro.z);

    if (count == 200 || count == 1) {
      DateTime now = rtc.now();
      
      myFile.print(F("\t"));
      myFile.print(now.timestamp(DateTime::TIMESTAMP_TIME));
      myFile.print("\t");
      myFile.print(temp.temperature);
      myFile.print(" degC");
    }
    
    myFile.println();
    myFile.flush();
    count--;
  } else {
    sleepNow(); // sleep function called here 
  }  
}
