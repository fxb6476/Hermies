#include <Wire.h>
#include <ros.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define ESC1 7  //GPIO 5 - Top left motor
#define ESC2 4  //GPIO 6 - Bot left motor
#define ESC3 2  //GPIO 7 - Top right motor
#define ESC4 3  //GPIO 8 - Bot right motor

#define kp_x 1  //Constants for PID Controller for x-axis
#define ki_x .001
#define kd_x 1

#define kp_y 1  //Constants for PID Controller for y-axis
#define ki_y .001
#define kd_y 1

#define kp_z 1  //Constants for PID Controller for z-axis
#define kd_z 1

Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t event; 

int throttle = 1000;
unsigned int looptime;
unsigned int speedloop;
unsigned int masterloop;

float lastErr_x, lastErr_y, lastErr_z = 0;              //Hold Previous Error Measurment, used to calculate d/dt of (error)
float sumErr_x, sumErr_y = 0;                           //Hold sum of error so far, used to calculate integral of (error)
float error_x, error_y, error_z = 0;                    //Holds most recent error measuments for all 3 axis.
float desired_x, desired_y, desired_z = 0;              //These are the desired angular change in all 3 axis.
float pid_x, pid_y, pid_z = 0;                          //Outputs for each PID controller. Default of 0 means nothing happens until the compute_PID function is called.

void calibrate_ESC(void){
  looptime = micros();

  //Measuring HIGH pulse.
  Serial.println("Starting HIGH measurments");
  while(micros() - looptime < 8000000){
    speedloop = micros();
    GPIOD_PDOR |= (1<<ESC1);
    GPIOD_PDOR |= (1<<ESC2);
    GPIOD_PDOR |= (1<<ESC3);
    GPIOD_PDOR |= (1<<ESC4);
    int wait_time = 2000;
    while( GPIOD_PDOR > 4){
      if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC1);
      if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC2);
      if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC3);
      if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC4);
    }
    delay(4);
  }

  looptime = micros();

  Serial.println("Starting LOW measurments");
  while(micros() - looptime < 8000000){
    speedloop = micros();
    GPIOD_PDOR |= (1<<ESC1);
    GPIOD_PDOR |= (1<<ESC2);
    GPIOD_PDOR |= (1<<ESC3);
    GPIOD_PDOR |= (1<<ESC4);
    int wait_time = 1000;
    while( GPIOD_PDOR > 4){
      if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC1);
      if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC2);
      if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC3);
      if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC4);
    }
    delay(4);
  }  
}

void compute_PID(void)
{
  sumErr_x += error_x * .004;
  sumErr_y += error_y * .004;
    
  pid_x = (kp_x * error_x) + (ki_x * sumErr_x) + (kd_x * (error_x - lastErr_x) / .004);
  pid_y = (kp_y * error_y) + (ki_y * sumErr_y) + (kd_y * (error_y - lastErr_y) / .004);
  pid_z = (kp_z * error_z) + (kd_z * (error_z - lastErr_z) / .004);
  
  lastErr_x = error_x;
  lastErr_y = error_y;
  lastErr_z = error_z;
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() {
  
  Serial.begin(115200);
  Serial.println("PID Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor. Set up sensor. */
  displaySensorDetails();
  bno.setExtCrystalUse(true);

  pinMode(5, OUTPUT);  pinMode(6, OUTPUT);   pinMode(7, OUTPUT);  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);

  calibrate_ESC();
  
  digitalWrite(13, HIGH);
  throttle = 1500;
  masterloop = micros();

}

void loop() {
  // put your main code here, to run repeatedly:
  while(micros() - masterloop < 3000000){
    looptime = micros();
  
    bno.getEvent(&event); //Getting angles

    error_x = event.orientation.z - desired_x;
    error_y = event.orientation.y - desired_y;
    error_z = 1; //desired_z - event.orientation.x;
  
    compute_PID();
    
    int spead1 = throttle + pid_x - pid_y; // + pid_z;
    int spead2 = throttle - pid_x + pid_y; // - pid_z; 
    int spead3 = throttle + pid_x - pid_y; // - pid_z;
    int spead4 = throttle - pid_x + pid_y; // + pid_z;
    if (spead1 > 2000) spead1 = 2000;
    if (spead2 > 2000) spead2 = 2000;
    if (spead3 > 2000) spead3 = 2000;
    if (spead4 > 2000) spead4 = 2000;
    if (spead1 < 1000) spead1 = 1000;
    if (spead2 < 1000) spead2 = 1000;
    if (spead3 < 1000) spead3 = 1000;
    if (spead4 < 1000) spead4 = 1000;

    speedloop = micros();
    GPIOD_PDOR |= (1<<ESC1);
    GPIOD_PDOR |= (1<<ESC2);
    GPIOD_PDOR |= (1<<ESC3);
    GPIOD_PDOR |= (1<<ESC4);
  
    while( GPIOD_PDOR > 4){
      if(micros() - speedloop > spead1) GPIOD_PDOR &= ~(1<<ESC1);
      if(micros() - speedloop > spead2) GPIOD_PDOR &= ~(1<<ESC2);
      if(micros() - speedloop > spead1) GPIOD_PDOR &= ~(1<<ESC3);
      if(micros() - speedloop > spead2) GPIOD_PDOR &= ~(1<<ESC4);
    }
    while( 
      (micros() - looptime) < 4000); 
  }
  GPIOD_PDOR |= (1<<ESC1);
  GPIOD_PDOR |= (1<<ESC2);
  GPIOD_PDOR |= (1<<ESC3);
  GPIOD_PDOR |= (1<<ESC4);
  int wait_time = 1000;
  while( GPIOD_PDOR > 4){
    if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC1);
    if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC2);
    if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC3);
    if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC4);
  }
  delay(4);
}
