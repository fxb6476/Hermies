//L3GD20 Interface functions.

#include <Wire.h>
#define gadd 107  //Gyroscope I2C address

float gyro_reads[3] = {0,0,0};
float gyro_cals[3] = {0,0,0};

void setup(){
  Wire.begin();
  Serial.begin(9600);
  
  //Configure Gyro...
  Wire
