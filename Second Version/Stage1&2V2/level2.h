

#define R 0
#define L 1
#define LERARN_TIMEOUT 5000
unsigned long startLearn;
unsigned long runTime2=0;               
char preCornerPos;
int C,U;
int readDistanseFrom;
unsigned long encoderCounterPreTurn=0;
bool testdirectionMore,near;
int encoderLeft,encoderRight,encoderStright,encoderReviece;
int roundNo,turnNo;
int rDirection;
String  obstacles[4];
int obIndex;

bool isStarted2;
int dist,preTurnDist;
double firstroundDest=0.0;

#include"betweenOb.h"
void debugLevel2(){
  Serial.print("encoder_count: "); Serial.print( encoder_count);
  Serial.print("rounCounter: "); Serial.print( rounCounter);
  Serial.print("setpoint: "); Serial.print( setpoint);
  Serial.println();
  }
void initLevel2(){
  obIndex=0;
   C=U=0;
   readDistanseFrom=LEFT_RIGHT;
   //readDistanseFrom=LIFT_FORWARD ;
   //readDistanseFrom=RIGHT_FORWARD ;
   rDirection=0;
   near=false;
   testdirectionMore=false;
   isStarted2=false;
   currentAngle=(int)mpu.getAngleZ();
   initAngle=currentAngle;
   angle0=currentAngle;
   motion_mode=STANDBY;
   initR= rangeR;
   initL= rangeL;
   initF= rangeF;
   roundNo=turnNo=0;
   rounCounter=0;
   rotionDirection=UNKNOWN_DIRECTION;
   setpoint = initR;
   dist=40;
  }
bool dalta(int side){
  int del;
  if (side==R){ del =initR-rangeR; if(abs(del)>=20)return true; else return false;}
  else{  del = initL-rangeL;  if(abs(del)>=20) return true; else return false;}
  }

  
void detectDirection(){debugUltrasonic();
   if(rangeR>110 ){C++; led(WHITE); testdirectionMore=true;}
   else if(rangeL >110){U++; rDirection=rDirection+1;  led(WHITE); testdirectionMore=true;}

   
    if( testdirectionMore){stop();
      for (int i=0;i<2;i++){
         readDistanseFrom=LEFT_RIGHT; delay(20);
         readDistance(readDistanseFrom); led(WHITE);
         if(dalta(R)){ C++;  rDirection=rDirection+0; led(WHITE); testdirectionMore=true;}
         if(dalta(L)){ U++;  rDirection=rDirection+1;  led(WHITE); testdirectionMore=true;}
        }
    
        if(U>C){rotionDirection=ANTI_CLOCKWIZE; Serial.println("Direction UntiClock"); forward(SPEED2);}
        else if (C>U){rotionDirection=CLOCKWIZE; Serial.println("Direction CLOCK wize"); forward(SPEED2);}
        else {rotionDirection=UNKNOWN_DIRECTION;}

        
        if(rotionDirection==ANTI_CLOCKWIZE){ Serial.println("ANTI CLOCK WIZE");if(initL<25)near=true; readDistanseFrom=RIGHT_FORWARD;}
        else if(rotionDirection==CLOCKWIZE){ Serial.println("CLOCK WIZE");if(initR<25)near=true; setpoint=initL; readDistanseFrom=LIFT_FORWARD ;}
       
    }
  }

void turnInCorner(){
  
    forward(SPEED2);
   
    if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+32; setSteering(steeringAngle);}
    else if (rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40; setSteering(steeringAngle);}
    if(abs(currentAngle-initAngle)>=80 && (encoder_count-encoderCounterPreTurn)>1690){ steering.write(STRAIGHT_STEERING);
    
      setpoint = 13; 
      encoder_count=0;  corner_Stage=REVERCE_;  Serial.println("Finish Turn"); }
    
  }

void cornerCalculations(){
  
    if(rounCounter==0){
      if(rotionDirection==CLOCKWIZE){
        corner_Stage=STRIGHT_ENCODER_;
        if(initL>=9 && initL<=26){preCornerPos='F';encoderLeft=0;encoderRight=0; encoderStright=0; encoderReviece=1180;}
        else if(initL>=41 && initL<=48){preCornerPos='C';encoderLeft=980;encoderRight=1050; encoderStright=0; encoderReviece=1040;}
        else if(initL>=60 && initL<=79){preCornerPos='N';encoderLeft=1150;encoderRight=1100; encoderStright=400; encoderReviece=1150;}
        } 
      else if(rotionDirection==ANTI_CLOCKWIZE){
        corner_Stage=STRIGHT_ENCODER_;
        if(initL>=9 && initL<=26){preCornerPos='N';encoderLeft=1150;encoderRight=1100; encoderStright=200; encoderReviece=1230;}
        else if(initL>=41 && initL<=48){preCornerPos='C';encoderLeft=950;encoderRight=800; encoderStright=0; encoderReviece=1150;}
        else if(initL>=60 && initL<=79){preCornerPos='F';encoderLeft=0;encoderRight=0; encoderStright=0;encoderReviece=1130;}
        }
    }
  
    if(rounCounter>0){
   
    if(rotionDirection==CLOCKWIZE){
        corner_Stage=STRIGHT_ENCODER_;
        if(preCornerPos=='F'){encoderLeft=0;encoderRight=0; encoderStright=0; encoderReviece=1180;}
        else if(preCornerPos=='N'){encoderLeft=1150;encoderRight=1100; encoderStright=400; encoderReviece=1150;}
    } 
    else if(rotionDirection==ANTI_CLOCKWIZE){
        corner_Stage=STRIGHT_ENCODER_;
        if(preCornerPos=='N'){encoderLeft=1150;encoderRight=1100; encoderStright=200; encoderReviece=1230;}
        else if(preCornerPos=='F'){encoderLeft=0;encoderRight=0; encoderStright=0;encoderReviece=1130;}
    }
   
      }
  //  motion_mode=STOP;
    encoder_count=0; 
  }


void preObstacle(){
  
  if(obstacles[obIndex].charAt(0)=='F'){
   
     switch (pre_Ob_Stage){  
         case GO_:
          setpoint=16.0; setSteering(steeringPID);    encoderStright=1300; encoder_count=0; pre_Ob_Stage=GO_STRIGHT_ENCODER;
         break;
         case GO_STRIGHT_ENCODER:
         if(encoder_count<encoderStright){forward(SPEED2);  setSteering(steeringPID); }
         else{pre_Ob_Stage=Go_STRIGHT_NEAR_OB;}
             
         break;
         case  Go_STRIGHT_NEAR_OB:
         if(rotionDirection==ANTI_CLOCKWIZE){ if(rangeL<100){  encoder_count=0; pre_Ob_Stage=GO_STRIGHT_ENCODER2;} else{forward(SPEED2);  setSteering(steeringPID);}}
         else if(rotionDirection==CLOCKWIZE){if(rangeR<100){  encoder_count=0; pre_Ob_Stage=GO_STRIGHT_ENCODER2;} else{forward(SPEED2);  setSteering(steeringPID);}}
          obstacle_Stage=FIRST;
         break;
         case GO_STRIGHT_ENCODER2:
         Serial.println("reach to first ob");
         if(encoder_count<200){forward(SPEED2);  setSteering(steeringPID); }
         else{ betweenOBbstacles(); }
        
         break;
     }
    
    
    }
  else{  
     switch (pre_Ob_Stage){  
         case GO_:
           encoderLeft=1100; encoderRight=1100;
           if(rotionDirection==ANTI_CLOCKWIZE){encoderStright=constrain(rangeR, 15, 26);  encoderStright=map(encoderStright,15,26,500,180); Serial.print("Stright=");Serial.println(encoderStright);}
           else if(rotionDirection==CLOCKWIZE){encoderStright=constrain(rangeL, 15, 26);  encoderStright=map(encoderStright,15,26,550,210); }
           //encoderStright=500;
           encoder_count=0;       
           pre_Ob_Stage=Go_FIRST_TURNE;
        break;
        case Go_FIRST_TURNE:
             Serial.println("First turne.");
          if(rotionDirection==ANTI_CLOCKWIZE){ steeringAngle=STRAIGHT_STEERING-40; setSteering(steeringAngle);
           }
          else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+32; setSteering(steeringAngle);}

          if(encoder_count<encoderLeft){forward(SPEED2);}
          else{pre_Ob_Stage=GO_STRIGHT; encoder_count=0;}
           
        break;
       case Go_SECOND_TURNE:
           if(rotionDirection==ANTI_CLOCKWIZE){ steeringAngle=STRAIGHT_STEERING+32; setSteering(steeringAngle);
           }
          else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40; setSteering(steeringAngle);}
           if(abs(currentAngle-initAngle)>=95){forward(SPEED2);}
           else{pre_Ob_Stage=GO_STRIGHT_ENCODER;}
       break;
       case GO_STRIGHT:
         setSteering(STRAIGHT_STEERING);
         if(encoder_count<encoderStright){forward(SPEED2);}
         else{pre_Ob_Stage=Go_SECOND_TURNE;}
          
       break;
       case GO_STRIGHT_ENCODER:
             //setSteering(STRAIGHT_STEERING);
             pre_Ob_Stage=END_;
              obstacle_Stage=FIRST; 
            // 
       break;
       case GO_REVERCE_ENCODER:
       break;
       case END_:
       betweenOBbstacles();
       break;
    
  }
    
    }
  
 
}
void goToCorner(){
  
  switch (corner_Stage) 
  { 
 
   case START_:
        cornerCalculations();
        break;
   case STRIGHT_ENCODER_:
        if(encoder_count<200 ){ Serial.print("encoder_count=");Serial.println(encoder_count);forward(SPEED2); }
        else{encoder_count=0;
        if(rotionDirection==CLOCKWIZE){corner_Stage=TURNE_L_;}
        else if(rotionDirection==ANTI_CLOCKWIZE) {corner_Stage=TURNE_R_;}
        }
        break;
   case STRIGHT_:
        setSteering(STRAIGHT_STEERING);
        if(encoder_count<encoderStright ){forward(SPEED2); }
        else{
          if(rotionDirection==CLOCKWIZE){corner_Stage=TURNE_R_;}  
          else if(rotionDirection==ANTI_CLOCKWIZE){corner_Stage=TURNE_L_;} 
          encoder_count=0; 
         }
        break;
   case TURNE_L_:
        steeringAngle=STRAIGHT_STEERING-40;
         setSteering(steeringAngle);
         if(encoder_count<encoderLeft ){forward(SPEED2); }
         else{ 
           if(rotionDirection==CLOCKWIZE){ corner_Stage=STRIGHT_;}  
           else if(rotionDirection==ANTI_CLOCKWIZE){ setSteering(STRAIGHT_STEERING); corner_Stage=PRE_TURNE90_; } 
           encoder_count=0;  /* corner_Stage=REVERCE_;*/}
        
        break;
   case TURNE_R_:
         //Serial.println("Turn rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr");
         steeringAngle=STRAIGHT_STEERING+32;
         setSteering(steeringAngle);
         if(encoder_count<encoderRight ){forward(SPEED2); }
         else{ 
           if(rotionDirection==CLOCKWIZE){setSteering(STRAIGHT_STEERING); corner_Stage=PRE_TURNE90_;}  
           else if(rotionDirection==ANTI_CLOCKWIZE){ corner_Stage=STRIGHT_; } 
          // motion_mode=STOP;
           encoder_count=0; }
         break;
   case PRE_TURNE90_:
         
         if(rangeF<32 ){corner_Stage=REVERCE_ENCODER_; encoder_count=0;}
         
         else {stop(); corner_Stage=REDEY_TO_TURNE_; setSteering(STRAIGHT_STEERING); }
        
        break;
   case REVERCE_ENCODER_:
        if(encoder_count<500 ){ setSteering(steeringREvircePID); reverse(SPEED2); }
        else{stop(); corner_Stage=REDEY_TO_TURNE_; setSteering(STRAIGHT_STEERING); encoder_count=0;}
        break;
   case REDEY_TO_TURNE_:
        if(rangeF<50){forward(SLOW_SPEED);}
        else{forward(SPEED2);}
        
        if(rangeF>35){setSteering(steeringPID); }
        else{corner_Stage=TURNE90_; }
        break;
   case TURNE90_:
         forward(SPEED2);
         if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+32; setSteering(steeringAngle);}
         else if (rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-35; setSteering(steeringAngle);}
        if(abs(currentAngle-initAngle)>=80/* && (encoder_count-encoderCounterPreTurn)>1690*/){
             stop();
             steering.write(STRAIGHT_STEERING); setpoint=15;  encoder_count=0; corner_Stage=REVERCE_;}
        break;
   case REVERCE_:
         Serial.println("Revirce mode");
         steering.write(STRAIGHT_STEERING);  
        // steering.write(steeringREvircePID);//setpoint
         if(encoder_count<encoderReviece ){reverse(SPEED2); }
        else{stop(); preCornerPos='F'; obIndex=(obIndex+1)%4; rounCounter++;
             corner_Stage=SEE_OBSTACLE_;  Serial.flush(); 
             if(rotionDirection==ANTI_CLOCKWIZE){Serial.write("A");}
             else {Serial.write("C");}  startLearn=millis();
        }
        
        break;
    case SEE_OBSTACLE_:
        // motion_mode=STOP;
        if(millis()-startLearn>LERARN_TIMEOUT){obstacles[obIndex]="FFF"; corner_Stage=GO_TO_OB_;}
        else{readSerial(); if(msg!=""){obstacles[obIndex]=msg.substring(0,3); corner_Stage=GO_TO_OB_;} }
        preCornerPos='F';
        readDistanseFrom=LEFT_RIGHT;
        pre_Ob_Stage=GO_;
        encoder_count=0;
        break;
    case GO_TO_OB_:
        
        Serial.println("Learn array is ");
        Serial.print("[0] "); Serial.println(obstacles[0]);
        Serial.print("[1] ");Serial.println(obstacles[1]);
        Serial.print("[2] ");Serial.println(obstacles[2]);
        Serial.print("[3] ");Serial.println(obstacles[3]);
       
        preObstacle();
        
        break;
       
     
  }
  //corner_Stage = STRIGHT;
   //if(rangeF<15 && rangeF)
  /*
  if(rotionDirection==ANTI_CLOCKWIZE && near){}
  else if (rotionDirection==ANTI_CLOCKWIZE){}
  else if(rotionDirection==CLOCKWIZE && near ){}
  else if(rotionDirection==CLOCKWIZE){}
  else{stop(); Serial.println("Stop");}
  */
  
  }
bool nearSmall(){
  bool near=false;
  if(rotionDirection==CLOCKWIZE && initL>35){near=true;  preTurnDist=(int)(4980-(22*(initL-25)));} 
  else if(rotionDirection==ANTI_CLOCKWIZE && initR>37) {near=true; preTurnDist=(int)(4980-(22*(initR-25)));
  }
  //Serial.print("turn1 dist is ="); Serial.println(preTurnDist);
  return(near);
  }
///////////////////////////////////////////
void doShiftLeft(int distance){
  
 
       switch (shift_Stages) 
       {
        case TURN1:
             steering.write(STRAIGHT_STEERING-40);
             forward(SPEED2);
             if(encoder_count>1400){
             shift_Stages=TURN2; steering.write(STRAIGHT_STEERING); encoder_count=0;}
        break;
        case TURN2:
             steering.write(STRAIGHT_STEERING+35);
             forward(SPEED2);
             if(encoder_count>1500){
               readUltrasonic();
              shift_Stages=REVERCE; Serial.println("R1"); Serial.println(rangeF);
             
             steering.write(STRAIGHT_STEERING); encoder_count=0;}
       
        break;
        case STRIGHT:
          forward(SPEED2);
          if(encoder_count>distance*3){shift_Stages=FINISH; 
           shift_Stages=TURN2; encoder_count=0;}
        break;
        case REVERCE:
           if(rangeF<35){ reverse(SPEED2); Serial.println("R1");} else{shift_Stages=FINISH;}
        break;
        case FINISH:
       stop();
        forward(SPEED2); motion_mode=FORWARD;
        //motion_mode= STOP;
        break;
    }
  
  }
////////////////////////////////////////////
void doShiftRight(int distance){
 
 
       switch (shift_Stages) 
       {
        case TURN1:
             steering.write(STRAIGHT_STEERING+35);
             forward(SPEED2);
             if(encoder_count>1000){
              shift_Stages=STRIGHT;steering.write(STRAIGHT_STEERING); encoder_count=0;}
        break;
        case TURN2:
         steering.write(STRAIGHT_STEERING-40);
          forward(SPEED2);
          if(encoder_count>1050){
              shift_Stages=FINISH;steering.write(STRAIGHT_STEERING); encoder_count=0;}
        break;
        case STRIGHT:
          forward(SPEED2);
          if(encoder_count>distance*5){shift_Stages=FINISH; 
           shift_Stages=TURN2; encoder_count=0;}
        break;
        case FINISH:
       // stop();
       //STOP
        motion_mode=FORWARD;
    
        break;
    }
  
  
}


void doForwardWithEncoder2(){
  if(encoder_count>1500){motion_mode=FORWARD; }
  else{forward(SPEED2); steering.write(steeringPID); pidState=true;}
  } 

void doTurn2(){
   
    forward(SPEED2);
   
    if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+32; setSteering(steeringAngle);}
    else if (rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-35; setSteering(steeringAngle);}
    if(abs(currentAngle-initAngle)>=80 && (encoder_count-encoderCounterPreTurn)>1690){ steering.write(STRAIGHT_STEERING); rounCounter++; 
      Serial.print("Enc C="); Serial.println(encoder_count);
      Serial.print("Enc pre t="); Serial.println(encoderCounterPreTurn);
      setpoint = 13; 
      encoder_count=0; encoderCounterPreTurn=0;/*stop(); ESP.restart(); */motion_mode=FORWARD_ENCODER; }
    
    
  }

void doParking2(){
  if((rangeF <146  && rangeF!=0 ) && encoder_count> 2180 ){ motion_mode=STOP; led(WHITE); Serial.print("Encoder"); Serial.println(encoder_count);}
  else{forward(SPEED2);steering.write(steeringPID);}
  }
  
///FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
void doForward2(){
  

  
  if(rotionDirection==UNKNOWN_DIRECTION){ forward(SLOW_SPEED); detectDirection(); } // if direction not set
  else{
    if(rounCounter<4 ){
    
    if(rotionDirection== CLOCKWIZE && rangeF<95){initAngle=currentAngle; motion_mode=CORNER; corner_Stage=START_; encoder_count=0;}
    else if(rotionDirection==ANTI_CLOCKWIZE && rangeF<95 ){initAngle=currentAngle;  motion_mode=CORNER; corner_Stage=START_;  encoder_count=0;}
    //***
     if(rotionDirection==ANTI_CLOCKWIZE){setpoint=16; readDistanseFrom=RIGHT_FORWARD;}
     else if(rotionDirection==CLOCKWIZE){setpoint=16; readDistanseFrom=LIFT_FORWARD ;}
    //***
    }
    else{motion_mode=STOP;}
    }
  
  
  
}



//EFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEF
//the main function of level 1
void doLevel2(){
 // if(pidState){steering.write(steeringPID); }
  switch (motion_mode) 
  {
  case STANDBY:
  break;
  case FORWARD:
    doForward2();    
  break;
  case FORWARD_ENCODER:
    doForwardWithEncoder2();
  break;
  case CORNER:
    goToCorner();
  break;
  case TURNE:
    doTurn2();
  break;
  case PARKING:
    doParking2();
  break;
  case STOP:
    stop();
    isStarted=false ; Serial.print("Time is : ");Serial.println((millis()-runTime)/1000);
    ESP.restart();
  break;
  case SHIFT_LEFT:
     doShiftLeft(dist);
  break;
  case SHIFT_RIGHT:
    doShiftRight(dist);
  break;
  }
  }
