#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <math.h>

#define ESC1 7  //GPIO 5 - Top left motor
#define ESC2 4  //GPIO 6 - Bot left motor
#define ESC3 2  //GPIO 7 - Top right motor
#define ESC4 3  //GPIO 8 - Bot right motor

#define kp_x 1  //Constants for PID Controller for x-axis
#define ki_x 1
#define kd_x 1

#define kp_y 1  //Constants for PID Controller for y-axis
#define ki_y 1
#define kd_y 1

#define kp_z 1  //Constants for PID Controller for z-axis
#define kd_z 1

Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
sensors_event_t event; 

int throttle = 950;
unsigned int looptime;
unsigned int speedloop;

float lastErr_x, lastErr_y, lastErr_z = 0;              //Hold Previous Error Measurment, used to calculate d/dt of (error)
float sumErr_x, sumErr_y = 0;                           //Hold sum of error so far, used to calculate integral of (error)
float error_x, error_y, error_z = 0;                    //Holds most recent error measuments for all 3 axis.
float desired_x, desired_y, desired_z = 0;              //These are the desired angular change in all 3 axis.
float pid_x, pid_y, pid_z = 0;                          //Outputs for each PID controller. Default of 0 means nothing happens until the compute_PID function is called.


void compute_PID(void)
{
  sumErr_x += error_x * .004
  sumErr_y += error_y * .004
    
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
  gyro.getSensor(&sensor);
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
  
  Serial.begin(9600);
  Serial.println("Gyroscope Test"); Serial.println("");
  
  /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  
  /* Initialise the sensor */
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();

  pinMode(5, OUTPUT);  pinMode(6, OUTPUT);   pinMode(7, OUTPUT);  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  GPIOD_PDOR |= (1<<ESC1);
  delayMicroseconds(throttle);
  GPIOD_PDOR &= ~(1<<ESC1);
  delay(4);
  GPIOD_PDOR |= (1<<ESC1);
  delayMicroseconds(2000);
  GPIOD_PDOR &= ~(1<<ESC1);

  throttle = 1200;

}

void loop() {
  // put your main code here, to run repeatedly:
    looptime = micros();
  
    gyro.getEvent(&event); //Getting rad/s

    error_x = desired_x - event.gyro.x * ( 360/ (2*3.14));
    error_y = desired_y - event.gyro.y * ( 360/ (2*3.14));
    error_z = desired_z - event.gyro.z * ( 360/ (2*3.14));
  
    compute_PID();

    int spead1 = throttle - pid_x - pid_y + pid_z;
    int spead2 = throttle + pid_x - pid_y - pid_z; 
    int spead3 = throttle - pid_x + pid_y - pid_z;
    int spead4 = throttle + pid_x + pid_y + pid_z;

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
