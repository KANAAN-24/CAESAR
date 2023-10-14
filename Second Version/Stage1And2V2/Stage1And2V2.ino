
#include"mode.h"
#include"Pins.h"
#include"RGBAndMode.h"
#include"Ultrasonic.h"
#include"DriveCar.h"
#include"Rotation.h"

#include"setSteering.h"
#include"SerialComm.h"
#include"level1.h"
#include"level2.h"

unsigned long displayTime=0;

void setup() {
 
  Serial.begin(115200);
  initRGBAndMode();
  led(RED);
  initCar();
  init_mpu();
  //initBT();
  initUltrasonic();
  
  initSteering();
  delay(500);
  readUltrasonic();
  initLevel1();
  motion_mode=STANDBY;
  checkLevel();
  driveTime=millis();
  
}

void loop() {
  
  //if(pidState){steering.write(steeringPID); }
  
   //debugUltrasonic();
  //Serial.print("steps to turn="); Serial.println((1000/46)*(initF-15));
  readAngle();
 // readBT();
  checkStartButton();
  if (start){
    if(level){ readDistance(readDistanseFrom);  doLevel2();}
    else{readUltrasonic();  doLevel1();}
  }
  else{  readUltrasonic(); encoder_count=0; if(level){initLevel2(); } else{initLevel1();} }

  //if(millis()-displayTime>200){debugUltrasonic();/* debugLevel1(); debugRotation();*/ displayTime=millis();}

}
