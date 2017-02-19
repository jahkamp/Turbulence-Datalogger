/*
 MSU BOREALIS 
 High-Altitude Weather Balloon IMU Datalogger

 This code is designed to log data from 
 an Absolute Orient IMU Fusion Breakout
 to an SD card on the MSU BOREALIS 
 Video payload package.

 Created October 19, 2016
 by Jared Kamp for Montana Space Grant Consortium

 Last edited Februrary 8, 2017 by Jared Kamp
 Version 2.0.1

 + 2.0.0 - Significantly increased the rate at which IMU data is gathered and logged. Version 1 was taking readings at a rate of 1/5 sec, while version 2 reads once every 52.6 msec.
 + 2.0.1 - Combining the code from 3 files (Iridium, Still Image, & Video) into 1 program. This will help maintain the same code across all IMU dataloggers. The only difference is file naming for the data log, so that is now read in from a setup file.
*/

// Include the necessary libraries for input/output, SD cards, hardware I2C, and the IMU.
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdio.h>

// Set the pins used
#define VBATPIN A9                // for adafruit feather battery voltage
#define cardSelect 4              // for microSD card
#define greenLED 8                //operating indicator
#define redLED 13                //failure indicator
volatile boolean SDConnect = false;     //boolean to control if SD card writing is available
volatile boolean inertialDetect = false; //boolean for data available from IMU

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
//IMU object
Adafruit_BNO055 bno = Adafruit_BNO055(55);
//File objects
File imufile;
File setupfile;

long t;   
bool columnHeaders = true;

// blinks message codes on green and red LEDs
void message(uint8_t errno, uint8_t led) {
  uint8_t i;
  for (i=0; i<errno; i++) {
    digitalWrite(led, HIGH);
    delay(250);
    digitalWrite(led, LOW);
    delay(250);
  }
  delay(500);
}
void setup(){
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  char imulog[15];
  t = millis();
  message(4, greenLED); //We're on!
  delay(5000);// Safe shutdown delay
  /* Initialise the sensor */
  if (!bno.begin())
  {
    // There was a problem detecting the BNO055. Blink 10 times for this error. 
    inertialDetect = false;
    message(1, redLED);
  }
  else {
    //Good 2 go
    inertialDetect = true;
    message(1, greenLED);
    delay(1000);
    bno.setExtCrystalUse(true);
  }
  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    // had an issue with accessing SD card. Blink 5 times for this.
    SDConnect = false;
    message(2, redLED);
  } else {
    //Good 2 go
    SDConnect = true;
    //Read the setup file to determine what IMU this is and what the file name should be saved as.
    
    setupfile = SD.open("IMUSETUP.txt", FILE_READ);
    if( ! setupfile ) {
      message(4, redLED);
    }
    for(uint8_t i = 0; i < 12; i++){
      imulog[i] = setupfile.read();
    }
    setupfile.close();
    imulog[12] = 0;
    strcpy(imulog, imulog);
    for (uint8_t i = 0; i < 100; i++) {
      imulog[6] = '0' + i/10;
      imulog[7] = '0' + i%10;
      // create if does not exist, do not open existing, write, sync after write
      if (! SD.exists(imulog)) {
        break;
      }
    } 
    imufile = SD.open(imulog, FILE_WRITE); 
    if(columnHeaders){
      imufile.print(F("Battery Voltage"));
      imufile.print(F(","));
      imufile.print(F("Temperature"));
      imufile.print(F(","));
      imufile.print(F("Acceleration X"));
      imufile.print(F(","));
      imufile.print(F("Acceleration Y"));
      imufile.print(F(","));
      imufile.print(F("Acceleration Z"));
      imufile.print(F(","));
      imufile.print(F("Degree X"));
      imufile.print(F(","));
      imufile.print(F("Degree Y"));
      imufile.print(F(","));
      imufile.print(F("Degree Z"));
      imufile.print(F(","));
      imufile.print(F("Orientation X"));
      imufile.print(F(","));
      imufile.print(F("Orientation Y"));
      imufile.print(F(","));
      imufile.print(F("Orientation Z"));
      imufile.print(F(","));
      imufile.print(F("System Calibration"));
      imufile.print(F(","));
      imufile.print(F("Gyroscope Calibration"));
      imufile.print(F(","));
      imufile.print(F("Accelerometer Calibration"));
      imufile.print(F(","));
      imufile.print(F("Magnetometer Calibration"));
      imufile.print(F(","));
      imufile.print(F("Hours of Operation"));    
      imufile.print(F(","));
      imufile.print(F("Minutes"));    
      imufile.print(F(","));
      imufile.println(F("Seconds"));    
    }
    message(2, greenLED);
 }
  //turn on and leave on the green LED to indicate operation
  digitalWrite(greenLED, HIGH);
}
// This loop runs infinitely. The code here starts after the code above has finished.
void loop() {
 //check to see if we have an IMU log file good 2 go on the sd card.
  if( ! imufile ) {
    //no file can mean no sd card inserted, or it's broken or corrupted. Blink 3 times for this.
    message(3, redLED);
  }  else {
//    Record battery voltage
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    imufile.print(measuredvbat);
    imufile.print(F(","));
    
    //Record IMU data
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> oreo = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Quaternion quat = bno.getQuat();
    int xaccel = accel.x();
    int yaccel = accel.y();
    int zaccel = accel.z();
    int xdeg = (((gyro.x())*180)/3.14159265359);
    int ydeg = (((gyro.y())*180)/3.14159265359);
    int zdeg = (((gyro.z())*180)/3.14159265359);
    int xore = (oreo.x());
    int yore = (oreo.y());
    int zore = (oreo.z());
    if (!inertialDetect) {
       //IMU was not accessible. 
      message(5, redLED);
    } else {
      /* Get a new sensor event */
      sensors_event_t event;
      bno.getEvent(&event);
      uint8_t sys, gyro, accel, mag = 0;
      bno.getCalibration(&sys, &gyro, &accel, &mag);
      imufile.print(bno.getTemp());
      imufile.print(F(","));
      imufile.print(xaccel);
      imufile.print(F(","));
      imufile.print(yaccel);
      imufile.print(F(","));
      imufile.print(zaccel);
      imufile.print(F(","));
      imufile.print(xdeg);
      imufile.print(F(","));
      imufile.print(ydeg);
      imufile.print(F(","));
      imufile.print(zdeg);
      imufile.print(F(","));
      imufile.print(xore);
      imufile.print(F(","));
      imufile.print(yore);
      imufile.print(F(","));
      imufile.print(zore);
      imufile.print(F(","));
      imufile.print(sys);
      imufile.print(F(","));
      imufile.print(gyro);
      imufile.print(F(","));
      imufile.print(accel);
      imufile.print(F(","));
      imufile.print(mag);
      imufile.print(F(","));
      imufile.print(((millis()-t)/3600000)%24);
      imufile.print(F(","));
      imufile.print(((millis()-t)/60000)%60);
      imufile.print(F(","));
      imufile.println(((millis()-t)/1000)%60);
    }
    //This next line ensures the data is saved to the SD card at the end of every reading.
    imufile.flush(); 
  }
}
