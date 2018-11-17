#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <math.h>

#define ESC1 7  //GPIO 5
#define ESC2 4  //GPIO 6
#define ESC3 2  //GPIO 7
#define ESC4 3  //GPIO 8

Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
sensors_event_t event; 

int throttle = 950;
unsigned int looptime;
unsigned int speedloop;

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

    float x_gyro = event.gyro.x * ( 360/ (2*3.14));
    float y_gyro = event.gyro.y * ( 360/ (2*3.14));
    float z_gyro = event.gyro.z * ( 360/ (2*3.14));

    int spead1 = throttle - x_gyro - y_gyro + z_gyro;
    int spead2 = throttle + x_gyro - y_gyro - z_gyro ; 
    int spead3 = throttle - x_gyro + y_gyro - z_gyro;
    int spead4 = throttle + x_gyro + y_gyro + z_gyro;

    speedloop = micros();
    GPIOD_PDOR |= (1<<ESC1);
    GPIOD_PDOR |= (1<<ESC2);
    while( GPIOD_PDOR > 4){
      if(micros() - speedloop > spead1) GPIOD_PDOR &= ~(1<<ESC1);
      if(micros() - speedloop > spead2) GPIOD_PDOR &= ~(1<<ESC2);
    }
    
    while( (micros() - looptime) < 4000); 

}
