#define ESC1 7  //GPIO 5 - Top left motor
#define ESC2 4  //GPIO 6 - Bot left motor
#define ESC3 2  //GPIO 7 - Top right motor
#define ESC4 3  //GPIO 8 - Bot right motor

int state = 0;
unsigned int looptime, speedloop;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Starting Callibration!");
  pinMode(5, OUTPUT);  
  pinMode(6, OUTPUT);   
  pinMode(7, OUTPUT);  
  pinMode(8, OUTPUT);
  
}


void loop() {
  // put your main code here, to run repeatedly:
  looptime = micros();
  
  if (state == 0){
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
    state = 1;
  }else if(state == 1){
    //Measuring LOW pulse.
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
    state = 2;
  }else if(state == 2){
    //Placing pulse in the middle.
    Serial.println("Starting MIDDLE measurments");
    while(micros() - looptime < 8000000){
      speedloop = micros();
      GPIOD_PDOR |= (1<<ESC1);
      GPIOD_PDOR |= (1<<ESC2);
      GPIOD_PDOR |= (1<<ESC3);
      GPIOD_PDOR |= (1<<ESC4);
      int wait_time = 1500;
      while( GPIOD_PDOR > 4){
        if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC1);
        if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC2);
        if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC3);
        if(micros() - speedloop > wait_time) GPIOD_PDOR &= ~(1<<ESC4);
      }
      delay(4);
    }
    state = 3;
  }else if(state == 3){
    //Testing callibration.
    Serial.println("Done, testing motors.");
    for(int i = 0; i < 1001; i=i+1){
      speedloop = micros();
      GPIOD_PDOR |= (1<<ESC1);
      GPIOD_PDOR |= (1<<ESC2);
      GPIOD_PDOR |= (1<<ESC3);
      GPIOD_PDOR |= (1<<ESC4);
      int wait_time = 1000;
      while( GPIOD_PDOR > 4){
        if(micros() - speedloop > (wait_time + i)) GPIOD_PDOR &= ~(1<<ESC1);
        if(micros() - speedloop > (wait_time + i)) GPIOD_PDOR &= ~(1<<ESC2);
        if(micros() - speedloop > (wait_time + i)) GPIOD_PDOR &= ~(1<<ESC3);
        if(micros() - speedloop > (wait_time + i)) GPIOD_PDOR &= ~(1<<ESC4);
      }
      delay(4);
    }
  }

}
