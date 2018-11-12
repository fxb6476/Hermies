#include <Wire.h>

#define gadd 107

float gyro_reads[3] = {0, 0, 0};
float gyro_cals[3] = {0, 0, 0};

void setup() {
  Wire.begin();
  Serial.begin(9600);

  //Configuring Gyro...
  Wire.beginTransmission(gadd);
  Wire.write(0x20);               //Writing to CTRL_REG1 - 
  Wire.write(0x0F);               //Enabling axis and "normal" operation mode
  Wire.endTransmission();

  Wire.beginTransmission(gadd);
  Wire.write(0x23);               //Writing to CTRL_REG4 -
  Wire.write(0x90);               // Setting IMU to NOT UPDATE during READS... Scale - 500dps; Little Endian Data Selection set
  Wire.endTransmission();         // The current scale gives .0175 dps/digit

  calibrate_gyro(gyro_cals, 2000);
  print_gyro_cals();    
}

void calibrate_gyro(float pdata[], int limit){
  Serial.println("Caligrating Gyro... Taking 2000 readings.");
  float tmp[] = {0, 0, 0};
  float tmp1[] = {0, 0, 0};
  for(int i=0; i<limit; i++){
    read_gyro(tmp);
    tmp1[0] += tmp[0];
    tmp1[1] += tmp[1];
    tmp1[2] += tmp[2];
  }
    pdata[0] += tmp1[0] / limit;
    pdata[1] += tmp1[1] / limit;
    pdata[2] += tmp1[2] / limit;
    Serial.println("Done!\n");
}

//FUNCTION DEFINITIONS!!!
void read_gyro(float pdata[]){
  byte lowbyte, highbyte;
  Wire.beginTransmission(gadd);
  Wire.write(168);
  Wire.endTransmission();
  Wire.requestFrom(gadd, 6);
  while(Wire.available() < 6);

  //Multiplying by dps resolution to get back degrees per second...
  lowbyte = Wire.read();
  highbyte = Wire.read();
  pdata[0] = (((highbyte<<8)|lowbyte) * .0175) - gyro_cals[0]; //Roll

  lowbyte = Wire.read();
  highbyte = Wire.read();
  pdata[1] = (((highbyte<<8)|lowbyte) * .0175) - gyro_cals[1]; //Pitch

  lowbyte = Wire.read();
  highbyte = Wire.read();
  pdata[2] = (((highbyte<<8)|lowbyte) * .0175) - gyro_cals[2]; //Yaw
}

void print_gyro(){
  Serial.print("Roll: ");
  Serial.print(gyro_reads[0]);
  Serial.print("\t");
  Serial.print("Pitch: ");
  Serial.print(gyro_reads[1]);
  Serial.print("\t");  
  Serial.print("Yaw: ");
  Serial.print(gyro_reads[2]);
  Serial.print("\n");
}

void print_gyro_cals(){
  Serial.println("Current Calibration values!!!");
  Serial.print("Roll: ");
  Serial.print(gyro_cals[0]);
  Serial.print("\t");
  Serial.print("Pitch: ");
  Serial.print(gyro_cals[1]);
  Serial.print("\t");  
  Serial.print("Yaw: ");
  Serial.print(gyro_cals[2]);
  Serial.print("\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  read_gyro(gyro_reads);
  delay(100);
}

