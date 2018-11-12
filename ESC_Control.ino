#define ESC1 7  //GPIO 5
#define ESC2 4  //GPIO 6
#define ESC3 2  //GPIO 7
#define ESC4 3  //GPIO 8

void setup(){
  pinMode(5, OUTPUT); pinMode(6, OUTPUT); pinMode(7, OUTPUT); pinMode(8, OUTPUT);
}

void loop(){
  GPIOD_PDOR |= (1<<ESC1); //Turning gpio 5 high
  delayMicroseconds(1400);  //Keep on for 1.4 mS
  GPIOD_PDOR &= ~(1<<ESC1); //Turn gpio 5 low
  delay(4);                 //Next loop in 4ms
}
  
