int Factor;
int UTurnDirection;
int rotitionEncoder;
int preUTurneAngle, nextAngle;
#define UTURN_LEFT 1
#define UTURN_RIGHT 2

int invertDirection(int currentDirection){
  if (setpoint==71){setpoint=15;}
  else if(setpoint==15){setpoint=71;}
  return(!currentDirection);
  }

void UTurnLeft(){
   switch(move_Stage){
    case STAGE1:
         complementaryMode=false;
         
    break;
    case STAGE2:
    break;
    case STAGE3:
    break;
    case STAGE4:
    break;
  }
}
void UTurnRight(){} 
void UTurnCalculations(char currentPos){
  char nextPos;
  preUTurneAngle=currentAngle;
  if(currentPos=='N'){nextPos='F';} else{nextPos='N';}
  
  //int nextDirection=rotionDirection;

   
      if(rotionDirection==ANTI_CLOCKWIZE){
        if(currentPos=='N'){setpoint=71;  UTurnDirection=UTURN_RIGHT; nextAngle=preUTurneAngle-180; rotitionEncoder=3000;}//U turne right
        else if(currentPos=='F'){setpoint=15; UTurnDirection=UTURN_LEFT; nextAngle=preUTurneAngle+180; rotitionEncoder=3100;} //U turne left
        }
      else if(rotionDirection==CLOCKWIZE){
        if(currentPos=='N'){setpoint=71; UTurnDirection=UTURN_LEFT; nextAngle=preUTurneAngle+180;rotitionEncoder=3100;}//U turne left
        else if(currentPos=='F'){setpoint=15; UTurnDirection=UTURN_RIGHT; nextAngle=preUTurneAngle-180;rotitionEncoder=3000;} //U turne righ
        }
    
  }

void doUTurn(){
 

   switch(uTurne_Stages){
    case CALCULATIONS:
        UTurnCalculations(currentPosition);  uTurne_Stages=STRAIGHT1; setSteering(STRAIGHT_STEERING); encoder_count=0;
    break;
    case STRAIGHT1:
       if (encoder_count<300){forward(SPEED2);}
       else{ uTurne_Stages=STRAIGHT2; encoder_count=0;}          
    break;
    case STRAIGHT2: 
       if (encoder_count<1200){forward(SPEED2);setSteering(steeringPID);}
       else{ stop(); uTurne_Stages=ROTATE; encoder_count=0;} 
                    
    break;
    case ROTATE:
        if(UTurnDirection==UTURN_RIGHT){setSteering(STRAIGHT_STEERING+40);}
        else if  (UTurnDirection==UTURN_LEFT){setSteering(STRAIGHT_STEERING-45);}
        if (encoder_count<3200 && abs(currentAngle-nextAngle)>4){if(encoder_count<2500){forward(SPEED2);} else {forward(SLOW_SPEED3);}}
        else{ stop(); uTurne_Stages=STRAIGHT3;  setSteering(STRAIGHT_STEERING); 
        rotionDirection=invertDirection(rotionDirection);
        encoder_count=0;} 
    break;
    case STRAIGHT3:
       if (encoder_count<500){forward(SPEED2);setSteering(STRAIGHT_STEERING);}
       else{ stop(); uTurne_Stages=FINISH_UTURN; encoder_count=0;} 
    
    break;
    case FINISH_UTURN:
    Serial.print("preUTurneAngle") ; Serial.println(preUTurneAngle); 
    Serial.print("nextAngle") ; Serial.println(nextAngle);
    Serial.print("currentAngle") ; Serial.println(currentAngle);
    ESP.restart();
    break;
   }
   
  } 
// move robote in side outer boarder FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
void fff(){
    currentPosition='F'; setpoint=15;
   if(/*rangeL<120 &&*/ encoder_count<2000 && rotionDirection==ANTI_CLOCKWIZE){if(encoder_count<1700){forward(SPEED2); } else {forward(SLOW_SPEED);}setSteering(steeringPID); }
   else if(/*rangeR<120 && */encoder_count<2000 && rotionDirection==CLOCKWIZE){if(encoder_count<1700){forward(SPEED2); } else {forward(SLOW_SPEED);}setSteering(steeringPID); }
   else{ currentPosition='F'; move_Stage=STAGE1;  obstacle_Stage=FINAL;  }}
// move robote in side inner boarder   NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNn
void nnn(){
 
  currentPosition='N';  //65 must change sensor
       if(rotionDirection==ANTI_CLOCKWIZE){setpoint=73;}
       else if(rotionDirection==CLOCKWIZE){setpoint=73;}
  if(/*rangeL<120 && */encoder_count<2000 && rotionDirection==ANTI_CLOCKWIZE){if(encoder_count<1700){forward(SPEED2); } else {forward(SLOW_SPEED);}  setSteering(steeringPID); }
  else if(/*rangeR<120 && */encoder_count<2000 && rotionDirection==CLOCKWIZE){if(encoder_count<1700){forward(SPEED2); } else {forward(SLOW_SPEED);} setSteering(steeringPID); }
  else{currentPosition='N'; move_Stage=STAGE1; obstacle_Stage=FINAL;}
  }

  
void nff(){
    currentPosition='F';
    switch(move_Stage){
    case STAGE1:
         move_Stage=STAGE2;
    break;
    case STAGE2:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count<200){forward(SPEED2);}
       else{ move_Stage=STAGE3;}
       readDistance(LEFT_RIGHT);
       Serial.print("Range L= ");  Serial.print(rangeL);  Serial.print("   Range R= ");  Serial.print(rangeR);
       //encoderStright=constrain(rangeR, 15, 26);   encoderStrightOb
        if(rotionDirection==ANTI_CLOCKWIZE){encoderStrightOb=constrain(rangeL, 5, 20); encoderStrightOb=map(encoderStrightOb,5,20,400,100);}
        else if(rotionDirection==CLOCKWIZE){encoderStrightOb=constrain(rangeR, 5, 20);  encoderStrightOb=map(encoderStrightOb,5,20,550,300);}
       Serial.print("   encoderStrightOb= ");  Serial.println(encoderStrightOb);
    break;
    case STAGE3:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+32;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count>200 && encoder_count<1500 ){forward(SPEED2);}
       else{ move_Stage=STAGE4; setSteering(STRAIGHT_STEERING); encoder_count=0;}
    break;
    case STAGE4:
           if (encoder_count<encoderStrightOb){forward(SPEED2);}
       else{ stop(); move_Stage=STAGE5; setSteering(STRAIGHT_STEERING);  encoder_count=0; //motion_mode=STOP;
       }
    break;
    case STAGE5:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle); /*turne left*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+40;setSteering(steeringAngle);/*turne Right*/}
       if (encoder_count<1100){forward(SPEED2);}
       else{ stop(); move_Stage=STAGE6; setSteering(STRAIGHT_STEERING);  encoder_count=0;}
    break;
    
    case STAGE6:
         if (encoder_count<200 ){forward(SPEED2);  setSteering(steeringPID);}
         else{move_Stage=STAGE7; setSteering(STRAIGHT_STEERING);}
         currentPosition='F';
    break;
    case STAGE7:
        currentPosition='F';
        motion_mode=FORWARD; move_Stage=STAGE1; motion_mode=STOP;
    break;
    }

  }


  //****
   //********************************************************************************************************
void fnn(){
   switch(move_Stage){
    case STAGE1:
         currentPosition='N';
         move_Stage=STAGE2;
    break;
    case STAGE2:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING; setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING; setSteering(steeringAngle);/*turne left*/}

       if (encoder_count<200){forward(SPEED2);}
       else{ move_Stage=STAGE3;}

       Serial.print("Range L= ");  Serial.print(rangeL);  Serial.print("   Range R= ");  Serial.print(rangeR);
       //encoderStright=constrain(rangeR, 15, 26);   encoderStrightOb
        if(rotionDirection==ANTI_CLOCKWIZE){encoderStrightOb=constrain(rangeL, 5, 20); encoderStrightOb=map(encoderStrightOb,5,20,500,150);}
        else if(rotionDirection==CLOCKWIZE){encoderStrightOb=constrain(rangeR, 5, 20);  encoderStrightOb=map(encoderStrightOb,5,20,550,300);}
       Serial.print("   encoderStrightOb= ");  Serial.println(encoderStrightOb);
    break;
    case STAGE3:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING -40;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING +32;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count>200 && encoder_count<1500 ){forward(SPEED2);}
       else{ move_Stage=STAGE4; setSteering(STRAIGHT_STEERING); encoder_count=0;}
    break;
    case STAGE4:
       if (encoder_count<encoderStrightOb ){forward(SPEED2);}
       else{ move_Stage=STAGE5; setSteering(STRAIGHT_STEERING); encoder_count=0;}
    break;
    case STAGE5:
          currentPosition='N';
          setpoint=71;
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+40;setSteering(steeringAngle); /*turne left*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle);/*turne Right*/}
       if (encoder_count<1000){forward(SPEED2);}
       else{ stop(); move_Stage=STAGE6; setSteering(STRAIGHT_STEERING); encoder_count=0;}
    break;
    case STAGE6:
        //  motion_mode=STOP;
        currentPosition='N';
         if (encoder_count<150 ){forward(SPEED2);  setSteering(steeringPID);
         Serial.print("rangeR="); Serial.println(rangeR);
          Serial.print("setpoint="); Serial.println(setpoint);
          Serial.print("complementary mode="); Serial.println(complementaryMode);
          Serial.print("steeringPID="); Serial.println(steeringPID);
         }
         else{move_Stage=STAGE7;delay(100); /*setSteering(STRAIGHT_STEERING);*/}
         
         //preCornerPos='F';
    break;
    case STAGE7:
        currentPosition='N';
        motion_mode=FORWARD; move_Stage=STAGE1;   motion_mode=STOP; 
    break;
   }
  }

  /*
  //###################################################################################################
void nnf(){
   switch(move_Stage){
    case STAGE1:
         encoder_count=0;
         setSteering(STRAIGHT_STEERING); 
         move_Stage=STAGE2;
         setpoint=71;
         currentPosition='N';
    break;
      case STAGE2:
      if(rotionDirection==ANTI_CLOCKWIZE   && encoder_count<1200){forward(SPEED2); setSteering(steeringPID); }
         else if(rotionDirection==CLOCKWIZE &&  encoder_count<1200){forward(SPEED2); setSteering(steeringPID);}
         else{
         move_Stage=STAGE3;}
       break;
    case STAGE3:
         if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+40;setSteering(steeringAngle); }
         else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle);}
         if(encoder_count> 1200 && encoder_count<2500){forward(SPEED2);}
         else{move_Stage=STAGE4; setSteering(STRAIGHT_STEERING);   }
         
       break;
    case STAGE4:
          if(encoder_count>2500 && encoder_count<2800) {forward(SPEED2);}
          else{move_Stage=STAGE5; setSteering(STRAIGHT_STEERING);} 
    break; 
    case STAGE5:
     
       currentPosition='F';
       Factor=1;
       if(rotionDirection==CLOCKWIZE){Factor=-1;}
       
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle); }
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+40;setSteering(steeringAngle);}
        Serial.print("currentAngle="); Serial.println(currentAngle);
        Serial.print("parking  angle="); Serial.println(((angle0+((rounCounter)*Factor*90))));
        Serial.print("angle0="); Serial.println(angle0);
       if( abs(currentAngle-(angle0+((rounCounter)*Factor*90)))>=3&& (encoder_count<3200)){
        if(encoder_count<2400){forward(SPEED2);}
        else{forward(SLOW_SPEED2);}
        
        }
       else{  stop();  Serial.print("Encoder="); Serial.println(encoder_count);
               delay(100); move_Stage=STAGE6; setSteering(STRAIGHT_STEERING); delay(100); 
              encoder_count=0;
       }
       break;
    case STAGE6:
      if(encoder_count<400) {reverse(SPEED2);}
       else{move_Stage=STAGE7; setSteering(STRAIGHT_STEERING);} 
    break;
    case STAGE7:
        currentPosition='F';
        motion_mode=FORWARD; move_Stage=STAGE1;
   
    }
  }

  */
 
 //************************************************************************************************************
 
/*
void ffn(){
   //NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN
    switch(move_Stage){
    case STAGE1:
         encoder_count=0;
         setSteering(STRAIGHT_STEERING); 
         move_Stage=STAGE2;
         setpoint=16;
         currentPosition='F';
    break;
      case STAGE2:
      if(rotionDirection==ANTI_CLOCKWIZE   && encoder_count<1200){forward(SPEED2); setSteering(steeringPID); }
         else if(rotionDirection==CLOCKWIZE &&  encoder_count<1200){forward(SPEED2); setSteering(steeringPID);}
         else{
         move_Stage=STAGE3;}
       break;
    case STAGE3:
         if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle); }
         else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+40;setSteering(steeringAngle);}
         if(encoder_count> 1200 && encoder_count<2500){forward(SPEED2);}
         else{move_Stage=STAGE4; setSteering(STRAIGHT_STEERING);   }
         
       break;
    case STAGE4:
          if(encoder_count>2500 && encoder_count<2800) {forward(SPEED2);}
          else{move_Stage=STAGE5; setSteering(STRAIGHT_STEERING);} 
    break; 
    case STAGE5:
     
       currentPosition='N';
       Factor=1;
       if(rotionDirection==CLOCKWIZE){Factor=-1;}
       
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+40;setSteering(steeringAngle); }
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle);}
        Serial.print("currentAngle="); Serial.println(currentAngle);
        Serial.print("parking  angle="); Serial.println(((angle0+((rounCounter)*Factor*90))));
        Serial.print("angle0="); Serial.println(angle0);
       if( abs(currentAngle-(angle0+((rounCounter)*Factor*90)))>=3 && (encoder_count<3200)){
        if(encoder_count<3500){forward(SPEED2);}
        else{forward(SLOW_SPEED2);}
        
        }
       else{  stop();  Serial.print("Encoder="); Serial.println(encoder_count);
               delay(100); move_Stage=STAGE6; setSteering(STRAIGHT_STEERING); delay(100); 
              encoder_count=0;
       }
       break;
    case STAGE6:
      if(encoder_count<400) {reverse(SPEED2);}
       else{move_Stage=STAGE7; setSteering(STRAIGHT_STEERING);} 
    break;
    case STAGE7:
        currentPosition='N';
        motion_mode=FORWARD; move_Stage=STAGE1;
   
    }
   //NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN
    
  }

*/
void setObstaclePos(){
  if(obstacles[rounCounter%4].charAt(0)=='F'){currentPosition='F'; complementaryMode=false;setpoint=16;}
  else{currentPosition='N'; complementaryMode=true; setpoint=71;}
  
  if(obstacles[obIndex]=="FFF"){obstacle_Stage=FFF;  Serial.println("FFF");} 
  else if(obstacles[obIndex]=="NFF"){obstacle_Stage=NFF; move_Stage = STAGE1;  Serial.println("NFF");}// 
  else if(obstacles[obIndex]=="NNF"){obstacle_Stage=NNF; move_Stage = STAGE1;Serial.println("NNF"); }//
  else if(obstacles[obIndex]=="NNN"){obstacle_Stage=NNN;Serial.println("NNN");}
  else if(obstacles[obIndex]=="FNN"){obstacle_Stage=FNN; move_Stage = STAGE1; Serial.println("FNN");}//
  else if(obstacles[obIndex]=="FFN"){obstacle_Stage=FFN; move_Stage = STAGE1; Serial.println("FFN");}//
  Serial.print("obstacles pos is"); Serial.println(obstacles[obIndex]);
  encoder_count=0;
  }

void betweenOBbstacles(){
 


  
  switch(obstacle_Stage){  
    
    case FIRST:
             if(currentPosition=='F'){complementaryMode=false;setpoint=16;}
             else if(currentPosition=='N') { complementaryMode=true; setpoint=71;}
             setObstaclePos();
    break;
    case FFF:
       fff();
    break;
    case NFF:
       nff();
    break;
  
    case NNN:
      nnn();
    break;
    case FNN:
      fnn();
    break;
    case FINAL:
    //complementaryMode=false;
        motion_mode=FORWARD;
    break;
    }
 
  }
