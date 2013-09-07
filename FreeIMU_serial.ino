/**
 * FreeIMU library serial communication protocol
*/

#include <ADXL345.h>
#include <bma180.h>
#include <HMC58X3.h>
#include <ITG3200.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>

//#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>

#include "FilteringScheme.h"

KalmanFilter kFilters[4];
int k_index = 4;

float q[4];
int raw_values[9];
float ypr[3]; // yaw pitch roll
char str[256];
float val[9];


// Set the FreeIMU object
FreeIMU my3IMU = FreeIMU();
//The command from the PC
char cmd;

void setup() {
  Serial.begin(38400);
  Wire.begin();
  
  float qVal = 0.125; //Set Q Kalman Filter(process noise) value between 0 and 1
  float rVal = 32.; //Set K Kalman Filter (sensor noise)
  
  for(int i = 0; i <= k_index; i++) { //Initialize Kalman Filters for 10 neighbors
  //KalmanFilter(float q, float r, float p, float intial_value);
      kFilters[i].KalmanInit(qVal,rVal,5.0,0.5);
  }
  
  my3IMU.init(true);
  
  // LED
  pinMode(13, OUTPUT);
}


void loop() {
  if(Serial.available()) {
    cmd = Serial.read();
    if(cmd=='v') {
      sprintf(str, "FreeIMU library by %s, FREQ:%s, LIB_VERSION: %s, IMU: %s", FREEIMU_DEVELOPER, FREEIMU_FREQ, FREEIMU_LIB_VERSION, FREEIMU_ID);
      Serial.print(str);
      Serial.print('\n');
    }
    else if(cmd=='r') {
      uint8_t count = serial_busy_wait();
      for(uint8_t i=0; i<count; i++) {
        my3IMU.getRawValues(raw_values);
        sprintf(str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,", raw_values[0], raw_values[1], raw_values[2], raw_values[3], raw_values[4], raw_values[5], raw_values[6], raw_values[7], raw_values[8], raw_values[9], raw_values[10]);
        Serial.print(str);
        Serial.print('\n');
      }
    }
    else if(cmd=='b') {
      uint8_t count = serial_busy_wait();
      for(uint8_t i=0; i<count; i++) {
        #if HAS_ITG3200()
          my3IMU.acc.readAccel(&raw_values[0], &raw_values[1], &raw_values[2]);
          my3IMU.gyro.readGyroRaw(&raw_values[3], &raw_values[4], &raw_values[5]);
        #else // MPU6050
          my3IMU.accgyro.getMotion6(&raw_values[0], &raw_values[1], &raw_values[2], &raw_values[3], &raw_values[4], &raw_values[5]);
        #endif
        writeArr(raw_values, 6, sizeof(int)); // writes accelerometer and gyro values
        #if IS_9DOM()
          my3IMU.magn.getValues(&raw_values[0], &raw_values[1], &raw_values[2]);
          writeArr(raw_values, 3, sizeof(int));
        #endif
        Serial.println();
      }
    }
    else if(cmd == 'q') {
      uint8_t count = serial_busy_wait();
      for(uint8_t i=0; i<count; i++) {
        my3IMU.getQ(q);
        serialPrintFloatArr(q, 4);
        Serial.println("");
      }
    }
        else if(cmd == 'z') {
         float temp1[16]; 
         temp1[13] = 0; temp1[14] = 0; temp1[15] = millis();
         uint8_t count = serial_busy_wait();
         for(uint8_t i=0; i<count; i++) {
                        my3IMU.getQ(q);
	    my3IMU.getValues(val);
	    temp1[7] = (val[3] * M_PI/180);
	    temp1[8] = (val[4] * M_PI/180);
	    temp1[9] = (val[5] * M_PI/180);
	    temp1[4] = (val[0]);
	    temp1[5] = (val[1]);
	    temp1[6] = (val[2]);
	    temp1[10] = (val[6]);
	    temp1[11] = (val[7]);
	    temp1[12] = (val[8]);
	    temp1[0] = (q[0]);
	    temp1[1] = (q[1]);
	    temp1[2] = (q[2]);
	    temp1[3] = (q[3]);
	    temp1[15] = millis();

	    #if HAS_MS5611() 
	    // with baro
		temp1[13] = (my3IMU.getBaroTemperature());
		temp1[14] = (my3IMU.getBaroPressure());
	      #endif		 

            serialPrintFloatArr(temp1,16);
            Serial.print('\n');
      }	  
    }
    else if(cmd == 'a') {
	float temp[16]; 
	temp[13] = 0; temp[14] = 0; temp[15] = millis();
        uint8_t count = serial_busy_wait();
        for(uint8_t i=0; i<count; i++) {
            my3IMU.getQ(q);
            my3IMU.getValues(val);
	    temp[7] = (val[3] * M_PI/180);
	    temp[8] = (val[4] * M_PI/180);
	    temp[9] = (val[5] * M_PI/180);
	    temp[4] = (val[0]);
	    temp[5] = (val[1]);
	    temp[6] = (val[2]);
	    temp[10] = (val[6]);
	    temp[11] = (val[7]);
	    temp[12] = (val[8]);
	    temp[0] = kFilters[0].measureRSSI(q[0]);
	    temp[1] = kFilters[1].measureRSSI(q[1]);
	    temp[2] = kFilters[2].measureRSSI(q[2]);
	    temp[3] = kFilters[3].measureRSSI(q[3]);
	    temp[15] = millis();

	    #if HAS_MS5611() 
	    // with baro
		temp[13] = (my3IMU.getBaroTemperature());
		temp[14] = (my3IMU.getBaroPressure());
	       #endif

	    serialPrintFloatArr(temp, 16);
            Serial.println("");
          }
     }

    #ifndef CALIBRATION_H
    else if(cmd == 'c') {
      const uint8_t eepromsize = sizeof(float) * 6 + sizeof(int) * 6;
      while(Serial.available() < eepromsize) ; // wait until all calibration data are received
      EEPROM.write(FREEIMU_EEPROM_BASE, FREEIMU_EEPROM_SIGNATURE);
      for(uint8_t i = 1; i<(eepromsize + 1); i++) {
        EEPROM.write(FREEIMU_EEPROM_BASE + i, (char) Serial.read());
      }
      my3IMU.calLoad(); // reload calibration
      // toggle LED after calibration store.
      digitalWrite(13, HIGH);
      delay(1000);
      digitalWrite(13, LOW);
    }
    else if(cmd == 'x') {
      EEPROM.write(FREEIMU_EEPROM_BASE, 0); // reset signature
      my3IMU.calLoad(); // reload calibration
    }
    #endif
    else if(cmd == 'C') { // check calibration values
      Serial.print("acc offset: ");
      Serial.print(my3IMU.acc_off_x);
      Serial.print(",");
      Serial.print(my3IMU.acc_off_y);
      Serial.print(",");
      Serial.print(my3IMU.acc_off_z);
      Serial.print("\n");
      
      Serial.print("magn offset: ");
      Serial.print(my3IMU.magn_off_x);
      Serial.print(",");
      Serial.print(my3IMU.magn_off_y);
      Serial.print(",");
      Serial.print(my3IMU.magn_off_z);
      Serial.print("\n");
      
      Serial.print("acc scale: ");
      Serial.print(my3IMU.acc_scale_x);
      Serial.print(",");
      Serial.print(my3IMU.acc_scale_y);
      Serial.print(",");
      Serial.print(my3IMU.acc_scale_z);
      Serial.print("\n");
      
      Serial.print("magn scale: ");
      Serial.print(my3IMU.magn_scale_x);
      Serial.print(",");
      Serial.print(my3IMU.magn_scale_y);
      Serial.print(",");
      Serial.print(my3IMU.magn_scale_z);
      Serial.print("\n");
    }
    else if(cmd == 'd') { // debugging outputs
      while(1) {
        my3IMU.getRawValues(raw_values);
        sprintf(str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,", raw_values[0], raw_values[1], raw_values[2], raw_values[3], raw_values[4], raw_values[5], raw_values[6], raw_values[7], raw_values[8], raw_values[9], raw_values[10]);
        Serial.print(str);
        Serial.print('\n');
        my3IMU.getQ(q);
        serialPrintFloatArr(q, 4);
        Serial.println("");
        my3IMU.getYawPitchRoll(ypr);
        Serial.print("Yaw: ");
        Serial.print(ypr[0]);
        Serial.print(" Pitch: ");
        Serial.print(ypr[1]);
        Serial.print(" Roll: ");
        Serial.print(ypr[2]);
        Serial.println("");
      }
    }
  }
}

char serial_busy_wait() {
  while(!Serial.available()) {
    ; // do nothing until ready
  }
  return Serial.read();
}

const int EEPROM_MIN_ADDR = 0;
const int EEPROM_MAX_ADDR = 511;

void eeprom_serial_dump_column() {
  // counter
  int i;

  // byte read from eeprom
  byte b;

  // buffer used by sprintf
  char buf[10];

  for (i = EEPROM_MIN_ADDR; i <= EEPROM_MAX_ADDR; i++) {
    b = EEPROM.read(i);
    sprintf(buf, "%03X: %02X", i, b);
    Serial.println(buf);
  }
}
