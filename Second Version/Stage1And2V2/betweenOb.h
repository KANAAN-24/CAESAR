
void fff(){
   if(rangeL<120 && encoder_count<3000 && rotionDirection==ANTI_CLOCKWIZE){forward(SPEED2);  setSteering(steeringPID); }
   else if(rangeR<120 && encoder_count<3000 && rotionDirection==CLOCKWIZE){forward(SPEED2);  setSteering(steeringPID); }
   else{ preCornerPos='F'; motion_mode=FORWARD;}}
void nnn(){
  if(rangeL<120 && encoder_count<3000 && rotionDirection==ANTI_CLOCKWIZE){forward(SPEED2);  setSteering(steeringPID); }
  else if(rangeR<120 && encoder_count<3000 && rotionDirection==CLOCKWIZE){forward(SPEED2);  setSteering(steeringPID); }
  else{preCornerPos='N'; motion_mode=FORWARD;}
  }

  
void nff(){
    preCornerPos='F';
    switch(move_Stage){
    case STAGE1:
         move_Stage=STAGE2;
    break;
    case STAGE2:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+5;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-5;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count<200){forward(SPEED2);}
       else{ move_Stage=STAGE3;}
    break;
    case STAGE3:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+32;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count>200 && encoder_count<1500 ){forward(SPEED2);}
       else{ move_Stage=STAGE4; setSteering(STRAIGHT_STEERING);}
    break;
    case STAGE4:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle); /*turne left*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+40;setSteering(steeringAngle);/*turne Right*/}
       if (encoder_count>1500 && encoder_count<3000){forward(SPEED2);}
       else{ stop(); move_Stage=STAGE5; setSteering(STRAIGHT_STEERING);}
    break;
    case STAGE5:
         if (encoder_count>3000 && encoder_count<3600 ){forward(SPEED2);  setSteering(steeringPID);}
         else{move_Stage=STAGE6; setSteering(STRAIGHT_STEERING);}
         preCornerPos='F';
    break;
    case STAGE6:
        preCornerPos='F';
        motion_mode=STOP;
    break;
    }

  }
void nnf(){
    switch(move_Stage){
    case STAGE1:
    break;
    case STAGE2:
    break;
    case STAGE3:
    break;
    case STAGE4:
    break;
    case STAGE5:
    break;
    }
  }
  //********************************************************************************************************
void fnn(){
   switch(move_Stage){
    case STAGE1:
         preCornerPos='N';
         move_Stage=STAGE2;
    break;
    case STAGE2:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-5;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+5;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count<300){forward(SPEED2);}
       else{ move_Stage=STAGE3;}
    break;
    case STAGE3:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING -40;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING +32;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count>300 && encoder_count<1600 ){forward(SPEED2);}
       else{ move_Stage=STAGE4; setSteering(STRAIGHT_STEERING);}
    break;
    case STAGE4:
       if (encoder_count>1600 && encoder_count<2000 ){forward(SPEED2);}
       else{ move_Stage=STAGE5; setSteering(STRAIGHT_STEERING);}
    break;
    case STAGE5:
       
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+40;setSteering(steeringAngle); /*turne left*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle);/*turne Right*/}
       if (encoder_count>2000 && encoder_count<3200){forward(SPEED2);}
       else{ stop(); move_Stage=STAGE6; setSteering(STRAIGHT_STEERING);}
    break;
    case STAGE6:
         // motion_mode=STOP;
         if (encoder_count>3200 && encoder_count<3600 ){forward(SPEED2);  setSteering(steeringPID);}
         else{move_Stage=STAGE7; setSteering(STRAIGHT_STEERING);}
         preCornerPos='F';
    break;
    case STAGE7:
        preCornerPos='F';
        motion_mode=STOP;
    break;
   }
  }
 //************************************************************************************************************
void nfn(){
   switch(move_Stage){
    case STAGE1:
         move_Stage=STAGE2;
    break;
    case STAGE2:
    
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+5;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-5;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count<200){forward(SPEED2);}
       else{ move_Stage=STAGE3;}
    break;
    case STAGE3:
      
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+32;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count>200 && encoder_count<1500 ){forward(SPEED2);}
       else{ move_Stage=STAGE4; setSteering(STRAIGHT_STEERING);}
    break;
    case STAGE4:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle); /*turne left*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+40;setSteering(steeringAngle);/*turne Right*/}
       if (encoder_count>1500 && encoder_count<3000){forward(SPEED2);}
       else{ stop(); move_Stage=STAGE5; setSteering(STRAIGHT_STEERING);}
    break;
    case STAGE5:
         if (encoder_count>3000 && encoder_count<3320 ){reverse(SPEED2);}
         else{ stop();delay(100); move_Stage=STAGE6; setSteering(STRAIGHT_STEERING);}
       // motion_mode=STOP;
      
    break;
    case STAGE6:
       
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle); /*turne left*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+40;setSteering(steeringAngle);/*turne Right*/}
       if (encoder_count>3320 && encoder_count<4650){forward(SPEED2);}
       else{  move_Stage=STAGE7; setSteering(STRAIGHT_STEERING);}
    break;
    case STAGE7:
      
      
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+32;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-40;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count>4650 && encoder_count<6400 && abs(currentAngle-initAngle)>=95){forward(SPEED2);}
       else{stop(); delay(50);encoder_count=0; move_Stage=STAGE8; setSteering(STRAIGHT_STEERING);}
    break;
    case STAGE8:
        if (encoder_count<400 ){reverse(SPEED2);}
        else{ stop();delay(50); move_Stage=STAGE9; setSteering(STRAIGHT_STEERING);}
    break;
    case STAGE9:
    preCornerPos='N';
    motion_mode=STOP;
    break;
  }
}
void fnf(){
   switch(move_Stage){
    case STAGE1:
    break;
    case STAGE2:
    break;
    case STAGE3:
    break;
    case STAGE4:
    break;
    case STAGE5:
    break;
  }
}

void ffn(){
    switch(move_Stage){
    case STAGE1:
    break;
    case STAGE2:
    break;
    case STAGE3:
    break;
    case STAGE4:
    break;
    case STAGE5:
    break;
    }
  }


void setObstaclePos(){
  if(obstacles[obIndex].charAt(0)=='F'){preCornerPos='F'; complementaryMode=false;setpoint=16;}
  else{preCornerPos='N'; complementaryMode=true; setpoint=71;}
  
  if(obstacles[obIndex]=="FFF"){obstacle_Stage=FFF; } 
  else if(obstacles[obIndex]=="NFF"){obstacle_Stage=NFF; move_Stage = STAGE1; }// 
  else if(obstacles[obIndex]=="NNF"){obstacle_Stage=NNF; move_Stage = STAGE1; }//
  else if(obstacles[obIndex]=="NNN"){obstacle_Stage=NNN;}
  else if(obstacles[obIndex]=="FNN"){obstacle_Stage=FNN; move_Stage = STAGE1; }//
  else if(obstacles[obIndex]=="FFN"){obstacle_Stage=FFN; move_Stage = STAGE1; }//
  else if(obstacles[obIndex]=="NFN"){obstacle_Stage=NFN; move_Stage = STAGE1; }//
  else if(obstacles[obIndex]=="FNF"){obstacle_Stage=FNF; move_Stage = STAGE1; }//
  Serial.print("obstacles pos is"); Serial.println(obstacles[obIndex]);
  encoder_count=0;
  }

void betweenOBbstacles(){
 
  if(preCornerPos=='F'){complementaryMode=false;setpoint=16;}
  else{ complementaryMode=true; setpoint=71;}

  
  switch(obstacle_Stage){
    
    case FIRST:
    
    setObstaclePos();
    break;
    case FFF:
       fff();
    break;
    case NFF:
       nff();
    break;
    case NNF:
    break;
    case NNN:
    Serial.println("NNN obbbbbb");
      nnn();
    break;
    case FNN:
      fnn();
    break;
    case FFN:
    break;
    case FNF:
    break;
    case NFN:
      nfn();
    break;
    case FINAL:
    break;
    }
 
  }
 //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$44
