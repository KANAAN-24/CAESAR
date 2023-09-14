unsigned long runTime2=0;
bool isStarted2;
int dist;
double firstroundDest=0.0;
void debugLevel2(){
  Serial.print("encoder_count: "); Serial.print( encoder_count);
  Serial.print("rounCounter: "); Serial.print( rounCounter);
  Serial.print("setpoint: "); Serial.print( setpoint);
  Serial.println();
  }
void initLevel2(){
   isStarted2=false;
   initAngle=currentAngle;
   angle0=currentAngle;
   motion_mode=STANDBY;
   initR= rangeR;
   initL= rangeL;
   initF= rangeF;
   currentAngle=(int)mpu.getAngleZ();
   rounCounter=0;
   rotionDirection=UNKNOWN_DIRECTION;
   setpoint = rangeR;
   dist=40;
  }

///////////////////////////////////////////
void doShiftLeft(int distance){
  pidState=false;
  if(rotionDirection==ANTI_CLOCKWIZE){}
  else if(rotionDirection==CLOCKWIZE){
 
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
}
////////////////////////////////////////////
void doShiftRight(int distance){
  pidState=false;
  if(rotionDirection==CLOCKWIZE){}
  else if(rotionDirection==ANTI_CLOCKWIZE){
 
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
}


void doForwardWithEncoder2(){
  if(encoder_count>1000){motion_mode=FORWARD; }
  else{forward(SPEED2); steering.write(steeringPID); pidState=true;}
  } 

void doTurn2(){
   
    forward(SPEED2);
   
    if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+35; setSteering(steeringAngle);}
    else if (rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40; setSteering(steeringAngle);}
    if(abs(currentAngle-initAngle)>=80){ steering.write(STRAIGHT_STEERING); rounCounter++; 
  
      setpoint = 13; 
      encoder_count=0; stop(); ESP.restart(); /*motion_mode=FORWARD_ENCODER;*/ }
    
    
  }
void doPreFinal2(){                       
  
   if((rotionDirection== CLOCKWIZE )&& (rangeF<(initL+25) && rangeF>1) &&  (rangeR>120 ||  rangeR==0)){initAngle=currentAngle; motion_mode=TURNE;}
    else if((rotionDirection==ANTI_CLOCKWIZE )&& (rangeF<(initR+25) && rangeF>1) &&  (rangeL>120 ||  rangeL==0)){initAngle=currentAngle; motion_mode=TURNE;}
    else{ forward(SPEED2);steering.write(steeringPID); pidState=true;} // go straight
  }
void doParking2(){
  if(rangeF <146 && rangeF!=0){ motion_mode=STOP; led(WHITE);}
  else{forward(SPEED2);steering.write(steeringPID);}
  }
///FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
void doForward2(){
  firstroundDest=(1000/48)*(initF-15);
  Serial.println(firstroundDest);
  if(rotionDirection==UNKNOWN_DIRECTION){// if direction not set
    if(rangeR>120 || rangeR==0 ){rotionDirection=CLOCKWIZE; led(WHITE);}
    else if(rangeL>120 || rangeL==0){rotionDirection=ANTI_CLOCKWIZE; led(WHITE);}
    }

 // if(rounCounter==11){motion_mode=PRE_FINAL;}
  if(rounCounter==12){motion_mode=PARKING;}
  else{
       if(rotionDirection==ANTI_CLOCKWIZE && (rangeL>120 || rangeL==0) && initR>50  ){motion_mode=SHIFT_RIGHT; encoder_count=0;}
       else if(rotionDirection==CLOCKWIZE && (rangeR>120 || rangeR==0) && initL>50  ){motion_mode=SHIFT_LEFT; encoder_count=0;}
       else if( rounCounter==0 ){if(encoder_count<(int)firstroundDest){forward(SPEED2);} else {initAngle=angle0;  motion_mode=TURNE;}}
       //else if(rotionDirection== CLOCKWIZE&& (rangeF<37 && rangeF>1) &&  rounCounter==0){initAngle=currentAngle; motion_mode=TURNE; Serial.print("first Turn:"); Serial.println(encoder_count);}
       //else if(rotionDirection==ANTI_CLOCKWIZE && (rangeF<37 && rangeF>1) &&  rounCounter==0){initAngle=currentAngle; motion_mode=TURNE; Serial.print("first Turn:"); Serial.println(encoder_count);}
       
       else if(encoder_count>=4980){initAngle=currentAngle; motion_mode=TURNE; Serial.print("turn>1 :"); Serial.println(encoder_count);}
       else{ forward(SPEED2);
       
       if(rotionDirection!=UNKNOWN_DIRECTION){steering.write(steeringPID);} } // go straight
   
    }
  

}

//EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
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
  case PRE_FINAL:
    doPreFinal2();
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
