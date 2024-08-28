

#define R 0
#define L 1
#define TIME_PRELEARN 1000
#define TIME_AFTERLEARN 500
#define LERARN_TIMEOUT 5000
unsigned long startLearn;      
unsigned long runTime2=0,delayTime;               
char currentPosition,nextPosition;
int C,U;
int Fact;
int strightEncoder1,strightEncoder2,nxtAng;  //encoder values and next angle  for  level2 turn
int stright1PID,stright2PID;  //set point to go stright in corner regon
int readDistanseFrom;
unsigned long encoderCounterPreTurn=0;
bool testdirectionMore,near,printArray;
int encoderLeft,encoderRight,encoderTurn1,encoderTurn2,encoderStright,encoderReviece,encoderStrightOb,encoderTurn;
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
  // currentAngle=(int)mpu.GetAngZ();
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
///

/*
shift_Stages = TURN1
 STRIGHT,
  FINISH,
  steeringAngle=STRAIGHT_STEERING-43;  setSteering(steeringAngle);} //turn Steering  Left

*/
void GoComplementary(){
  
   switch(shift_Stages){
    case BRE_SHIFT:
    
       encoder_count=0;
       complementaryMode=false;
       rotionDirection=CLOCKWIZE;
       setSteering(STRAIGHT_STEERING);
       setpoint=73;
       shift_Stages=TURN1;
    break;
    case TURN1:
     debugUltrasonic();
    if(encoder_count>2000){shift_Stages=STRIGHT;stop();}
    forward(SPEED2);
    setSteering(steeringPID);
    break;
    case STRIGHT:
     motion_mode=STOP;
    break;      
    }
   
   }

void shifLeft(){
  
   switch(shift_Stages){
         case BRE_SHIFT:
         
         shift_Stages=TURN1;
         encoder_count=0;
         break;
         case TURN1:
              steeringAngle=STRAIGHT_STEERING-35;  setSteering(steeringAngle);
              forward(SLOW_SPEED2);
              if (encoder_count>=500){shift_Stages=STRIGHT;stop();encoder_count=0; delay(50);}
         break;
         case STRIGHT:
              
               setSteering(STRAIGHT_STEERING);
               shift_Stages=TURN2;
               encoder_count=0;
               delay(100);
         break;
         case TURN2:
              steeringAngle=STRAIGHT_STEERING+35;  setSteering(steeringAngle);
              forward(SLOW_SPEED2);
              if (encoder_count>=500){shift_Stages=STRIGHT;stop();shift_Stages=FINISH;encoder_count=0; delay(50);}
         break;
         case FINISH:
           setSteering(STRAIGHT_STEERING);
         motion_mode=STOP;
         break;  
   }
   
  
  }
void shiftRight(){
  
   switch(shift_Stages){
         case BRE_SHIFT:
         
         shift_Stages=TURN1;
         encoder_count=0;
         break;
         case TURN1:
              steeringAngle=STRAIGHT_STEERING+35;  setSteering(steeringAngle);
              forward(SLOW_SPEED2);
              if (encoder_count>=500){shift_Stages=STRIGHT;stop();encoder_count=0; delay(50);}
         break;
         case STRIGHT:
              
               setSteering(STRAIGHT_STEERING);
               shift_Stages=TURN2;
               encoder_count=0;
               delay(100);
         break;
         case TURN2:
              steeringAngle=STRAIGHT_STEERING-35;  setSteering(steeringAngle);
              forward(SLOW_SPEED2);
              if (encoder_count>=500){shift_Stages=STRIGHT;stop();shift_Stages=FINISH;encoder_count=0; delay(50);}
         break;
         case FINISH:
           setSteering(STRAIGHT_STEERING);
         motion_mode=STOP;
         break;  
   }
   
  
  }
bool dalta(int side){
  int del;
  if (side==R){ del =initR-rangeR; if(abs(del)>=20)return true; else return false;}
  else{  del = initL-rangeL;  if(abs(del)>=20) return true; else return false;}
  }


// function to find Direction 
void detectDirection(){//debugUltrasonic();
    setSteering(STRAIGHT_STEERING);
   if(rangeR>110 ){C++; led(WHITE); testdirectionMore=true;}
   else if(rangeL >110){U++; rDirection=rDirection+1;  led(WHITE); testdirectionMore=true;}

   
    if( testdirectionMore){stop();
      for (int i=0;i<2;i++){
         readDistanseFrom=LEFT_RIGHT; delay(20);
         readDistance(readDistanseFrom); led(WHITE);
         if(dalta(R)){ C++;  rDirection=rDirection+0; led(WHITE); testdirectionMore=true;}
         if(dalta(L)){ U++;  rDirection=rDirection+1;  led(WHITE); testdirectionMore=true;}
        }
    
        if(U>C){rotionDirection=ANTI_CLOCKWIZE; move_Stage=STAGE1;Serial.println("Direction UntiClock"); forward(SPEED2);}
        else if (C>U){rotionDirection=CLOCKWIZE;move_Stage=STAGE1; Serial.println("Direction CLOCK wize"); forward(SPEED2);}
        else {rotionDirection=UNKNOWN_DIRECTION;}

        
        if(rotionDirection==ANTI_CLOCKWIZE){ Serial.println("ANTI CLOCK WIZE");if(initL<25)near=true; readDistanseFrom=RIGHT_FORWARD;}
        else if(rotionDirection==CLOCKWIZE){ Serial.println("CLOCK WIZE");if(initR<25)near=true; setpoint=initL; readDistanseFrom=LIFT_FORWARD ;}
       
    }
  }



// function to Enter Corner and stop
void doEnterCorner(){
   
  readDistanseFrom=ALL;
  readDistance(readDistanseFrom);
  //forward(SLOW_SPEED);SPEED2
  forward(SPEED2);
  Serial.print("rangeF=");   Serial.print(rangeF);
  encoder_count=0;
  if((rotionDirection==ANTI_CLOCKWIZE && rangeL>120) && rangeF<95){stop();move_Stage=STAGE2;}
  else if((rotionDirection==CLOCKWIZE && rangeR>120) &&  rangeF<95){stop();move_Stage=STAGE2;}
                 
  
  } 



   
    
void doForward2(){
  
  
  
  if(rotionDirection==UNKNOWN_DIRECTION){  setSteering(STRAIGHT_STEERING); forward(SLOW_SPEED); detectDirection(); } // if direction not set
  else{
    if(rounCounter<4 ){

    
    switch(move_Stage){
    case STAGE1:
             readDistanseFrom=ALL;
             readDistance(readDistanseFrom);
             /*
             if(rounCounter==0){setSteering(STRAIGHT_STEERING);}
             else{
                 if(currentPosition=='F'){complementaryMode=false;setpoint=14;}
                 else if(currentPosition=='N') { complementaryMode=false; setpoint=73;}//??????????????????????????
                 setSteering(steeringPID);
             }
*/
          forward(SLOW_SPEED2);
          Serial.print("rangeF=");   Serial.print(rangeF);
          //Serial.print("rangeF=");   Serial.print(rangeF);
          // Serial.print("rangeF=");   Serial.print(rangeF);
          encoder_count=0;
          if((rotionDirection==ANTI_CLOCKWIZE && rangeL>120) && rangeF<95){stop(); complementaryMode=false; move_Stage=STAGE2; Serial.println("Direction detected Anti");}
          else if((rotionDirection==CLOCKWIZE && rangeR>120) &&  rangeF<95){stop(); complementaryMode=false;move_Stage=STAGE2; Serial.println("Direction detected Clock");}
         ///************
         encoder_count=0;
         
    break;
    case STAGE2:
        forward(SLOW_SPEED2);
        CurrentSteeringAng=STRAIGHT_STEERING;
        if(encoder_count>=100){stop(); move_Stage=STAGE3;delayTime=millis(); Serial.println("Forward 100 step");}

    
    break;
    case STAGE3:
    // turn CAM To learn
          if(millis()-delayTime>=TIME_PRELEARN){move_Stage=STAGE4;Serial.println("Look to turn learn");}
          
         if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-43; turnSteering(steeringAngle); /*setSteering(steeringAngle);*/} //turn CAM LEFT    setSteering(STRAIGHT_STEERING);
         else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+35; turnSteering(steeringAngle); /* setSteering(steeringAngle);*/} // turn CAM RIGHT
    break;
    case STAGE4:
         motion_mode=LEARN;                      
         move_Stage=STAGE1;
      // motion_mode=STOP;
    break;
 
    }
    }
    
    else if(rounCounter>=4 && rounCounter<8){
      Serial.println("we  are in second rounds");
      
       //TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT

       
         switch(move_Stage){
          case STAGE1:
             if(currentPosition=='F'){complementaryMode=false;setpoint=16;}
             else if(currentPosition=='N') { complementaryMode=true; setpoint=71;}
             setSteering(steeringPID);
             //setSteering(STRAIGHT_STEERING);
         ///************
          readDistanseFrom=LEFT_RIGHT;
        //  readDistance(readDistanseFrom);
          //forward(SLOW_SPEED);SPEED2
          forward(SPEED2);
          Serial.print("rangeF=");   Serial.print(rangeF);
          encoder_count=0;
          if((rotionDirection==ANTI_CLOCKWIZE && rangeL>120) && rangeF<95){stop(); complementaryMode=false;  move_Stage=STAGE2;}
          else if((rotionDirection==CLOCKWIZE && rangeR>120) &&  rangeF<95){stop();complementaryMode=false;  move_Stage=STAGE2;}
         ///************
         encoder_count=0;
         
    break;
    case STAGE2:
        forward(SLOW_SPEED2);
        if(encoder_count>=100){stop(); move_Stage=STAGE3;delayTime=millis();}
    
    break;
    case STAGE3:
    // turn stearing to correct side
         if(millis()-delayTime>=TIME_AFTERLEARN){move_Stage=STAGE4;}
          //***
          Serial.print("ROUND 2===>>>  "); Serial.print("memory location=");  Serial.println((rounCounter+1)%4); 
          Serial.print("memory Data ="); Serial.println(obstacles[(rounCounter+1)%4]);   
          
         if(obstacles[(rounCounter+1)%4].charAt(0)=='F'){ setSteering(STRAIGHT_STEERING);}
         else{
               if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-43;  setSteering(steeringAngle);} //turn Steering  Left 
               else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+35;  setSteering(steeringAngle);} // turn steering Right
          }
          //***

    break;
    case STAGE4:

         motion_mode=TURNE;
         move_Stage=STAGE1;

      motion_mode=STOP;
    break;
 
    }
    
       //TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
      // motion_mode=STOP;
      }
    else{motion_mode=STOP;}

    
    
  
  
  
}

}


void  OB_Learn(){
  
    switch(move_Stage){
    case STAGE1:
         Serial.println("In Lerarn");
         obIndex=(obIndex+1)%4; 
         move_Stage=STAGE2;
         delayTime=millis();
         
    break;
    case STAGE2:/// learn obesticals and store in memory
          if(millis()-delayTime>=TIME_PRELEARN){move_Stage=STAGE3;  /*setSteering(STRAIGHT_STEERING);*/ delayTime=millis();}
          obstacles[obIndex]="NNN";

          obstacles[1]="NNN";
          obstacles[2]="FFF";
          obstacles[3]="NNN";
          obstacles[0]="FFF";
          
        
    break;
    case STAGE3://just waite untel stearing is straight
         if(millis()-delayTime>=TIME_PRELEARN){move_Stage=STAGE4;}
    break;
    case STAGE4:
         Serial.println("Finish learn and go to corner to turn");
         motion_mode=TURNE;
         move_Stage=STAGE1;
        
    break;
 
    }
  
  }



void turnCalculatons(){
   
    if((rounCounter-1)==0){
      if(rotionDirection==CLOCKWIZE){
        
        if(initL>=1 && initL<=26){currentPosition='F';}
        else if(initL>=41 && initL<=48){currentPosition='C';}
        else if(initL>=60 && initL<=79){currentPosition='N';}
        } 
      else if(rotionDirection==ANTI_CLOCKWIZE){
      
        if(initL>=1 && initL<=26){currentPosition='N';}
        else if(initL>=41 && initL<=48){currentPosition='C';}
        else if(initL>=60 && initL<=79){currentPosition='F';}
        }
    }

      if(obstacles[rounCounter%4].charAt(0)=='F'){nextPosition='F';}
      else{nextPosition='N';}
      
       Serial.print("Memory="); Serial.print(obstacles[rounCounter%4]);
       Serial.print("Iint Range L: ");Serial.println(initL);

        Serial.print("currentPosition: ");Serial.println(currentPosition);
        Serial.print("Next Position: ");Serial.println(nextPosition);

// determine first stage of turn in corner 
      if(currentPosition=='N'  &&  nextPosition=='N'){if(rotionDirection==ANTI_CLOCKWIZE){ encoderTurn=1250;}else{encoderTurn=1230;}
                                                      turne_Stages=DO_TURN;  }
      
      else if(currentPosition=='N'  &&  nextPosition=='F'){turne_Stages=TURN_STRAIGHT1;
                                                           if(rotionDirection==ANTI_CLOCKWIZE){ stright1PID=65; stright2PID=15; encoderTurn=1500;} 
                                                           else{setpoint = rangeL-3; stright1PID=65;  stright2PID=15; encoderTurn=1380;} }
      else if(currentPosition=='F'  &&  nextPosition=='N'){ if(rotionDirection==ANTI_CLOCKWIZE){encoderTurn=1250;}else{encoderTurn=1230;}
                                                        turne_Stages=DO_TURN;  stright2PID=75;}////
      else if(currentPosition=='F'  &&  nextPosition=='F'){turne_Stages=TURN_STRAIGHT1; 
                                                           if(rotionDirection==ANTI_CLOCKWIZE){setpoint = rangeR; stright1PID=rangeR;  stright2PID=15;encoderTurn=1500;} 
                                                           else{setpoint = rangeL; stright1PID=rangeL;  stright2PID=15;encoderTurn=1380;}}
      else if(currentPosition=='C'  &&  nextPosition=='N'){if(rotionDirection==ANTI_CLOCKWIZE){encoderTurn=1250;}else{encoderTurn=1230;}
                                                           turne_Stages=DO_TURN; }  
      else if(currentPosition=='C'  &&  nextPosition=='F'){
                                                            turne_Stages=TURN_STRAIGHT1;  
                                                               if(rotionDirection==ANTI_CLOCKWIZE){setpoint = rangeR; stright1PID=rangeR;  stright2PID=15;encoderTurn=1500;}
                                                              else{setpoint = rangeL; stright1PID=rangeL;  stright2PID=15; encoderTurn=1380;}}
      initAngle=currentAngle; 
      Fact=1;
      if(rotionDirection==CLOCKWIZE){Fact=-1;}
      nxtAng=angle0+((rounCounter)*Fact*90);
      Serial.print("currenr Angle=");Serial.println(currentAngle);
     // Serial.print("Next angle");Serial.println(nxtAng);
      encoder_count=0;
  /*
    if(rounCounter>0){
      
    if(rotionDirection==CLOCKWIZE){
        corner_Stage=STRIGHT_ENCODER_;
        if(currentPosition=='F'){encoderTurn1=0;encoderTurn2=0; encoderStright=0; encoderReviece=1100;}
        else if(currentPosition=='N'){encoderTurn1=1150;encoderTurn2=1200; encoderStright=400; encoderReviece=1050;}
    } 
    else if(rotionDirection==ANTI_CLOCKWIZE){
        corner_Stage=STRIGHT_ENCODER_;
        if(currentPosition=='N'){encoderTurn2=1250;encoderTurn1=1000; encoderStright=100; encoderReviece=1050;}
        else if(currentPosition=='F'){encoderTurn2=0;encoderTurn1=0; encoderStright=0;encoderReviece=1100;}
    }
   
      }
      encoderReviece=850;
  //  motion_mode=STOP;
  */
    encoder_count=0; 
  

  }



/*
 {
  TURN_CALCULATIONS,
  TURN_STRAIGHT,
  TURN_STRAIGHT1,
  TURN_STRAIGHT2,
  DO_TURN,
  FINISH_TURN,
 
} turne_Stages = TURN_CALCULATIONS;
 */

//function to turn in corner 
void turnInCorner(){
    //readDistance(ALL);
    switch(turne_Stages){
      
      case TURN_STRAIGHT1:
 
      

            setSteering(steeringPID);
            forward(SPEED2);
            if(rangeF<35/* add  incoder condition  */){Serial.println("TURN STAGE");
              turne_Stages=DO_TURN; encoder_count=0;}
          
      break;
      
      case TURN_STRAIGHT2:  
            Serial.println("IN STRIGHT 2");
           if((rotionDirection==ANTI_CLOCKWIZE && rangeL<100) && currentPosition=='N'){  turnSteering(STRAIGHT_STEERING);  CurrentSteeringAng=STRAIGHT_STEERING; stop(); delay(100); turne_Stages=CHECK; encoder_count=0; Serial.println("FINISH  Stright in TURN STAGE");/*motion_mode=STOP;*/} 
           else if((rotionDirection==CLOCKWIZE && rangeR<100)&& currentPosition=='N'){   turnSteering(STRAIGHT_STEERING); CurrentSteeringAng=STRAIGHT_STEERING; stop(); delay(100); turne_Stages=CHECK; encoder_count=0; Serial.println("FINISH  Stright in TURN STAGE");/*motion_mode=STOP;*/} 
           else if((rotionDirection==ANTI_CLOCKWIZE && rangeL<100)&&  encoder_count>400){   turnSteering(STRAIGHT_STEERING); CurrentSteeringAng=STRAIGHT_STEERING; stop(); delay(100); turne_Stages=CHECK; encoder_count=0; Serial.println("FINISH  Stright in TURN STAGE");/*motion_mode=STOP;*/}
          else if((rotionDirection==CLOCKWIZE && rangeR<100)&&  encoder_count>400){   turnSteering(STRAIGHT_STEERING); CurrentSteeringAng=STRAIGHT_STEERING; stop(); delay(100); turne_Stages=CHECK; encoder_count=0; Serial.println("FINISH  Stright in TURN STAGE");/*motion_mode=STOP;*/}
          else{
            debugUltrasonic();
           setSteering(steeringPID);
           forward(SLOW_SPEED4);
           }
          // debugUltrasonic(); 
          // setpoint=stright2PID; 
          
           
          // setSteering(STRAIGHT_STEERING); 
      //  turnSteering(steeringAngle);
          
             
      break;
      case DO_TURN:
            forward(SLOW_SPEED2);
            if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-43;  turnSteering(steeringAngle);} 
            else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+38;  turnSteering(steeringAngle);} 
            
          //   
            
            // if(abs(currentAngle-nxtAng)> 4 /*&&  encoder_count<encoderTurn*/)
             if(  (abs(currentAngle-nxtAng)> 4 || encoder_count<(encoderTurn-250)  ) &&   encoder_count<encoderTurn  )
             {
              //forward(SLOW_SPEED3);
             if(encoder_count<(encoderTurn*0.80)){forward(SPEED2);} else {forward(SLOW_SPEED3);}
                   }

         else{
              Serial.println("Finish turn ...........................");
             //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
             Serial.print("Turn Encoder count="); Serial.println(encoder_count);
             Serial.print("currentAngle="); Serial.println(currentAngle);
             stop(); steering.write(STRAIGHT_STEERING); delay(300);
             turne_Stages=TURN_STRAIGHT2;
             readDistanseFrom=LEFT_RIGHT;
              if(rotionDirection==ANTI_CLOCKWIZE){setpoint = rangeR; /*setpoint = 74; */   }
              else{setpoint = rangeL; /*setpoint = 74; */ }
            //  motion_mode=STOP;
             encoder_count=0;
             /*
              * 
              * 
              *  if(rotionDirection==ANTI_CLOCKWIZE){setpoint = rangeR; stright1PID=rangeR;  stright2PID=15;encoderTurn=1420;}
                                                              else{setpoint = rangeL; stright1PID=rangeL;  stright2PID=15;
              if(nextPosition=='F'){setpoint=15; } else{setpoint=80;} encoder_count=0;  //motion_mode=STOP;

              
                if(currentPosition=='N' ){
                  turne_Stages=FINISH_TURN; Serial.println("FINISH TURN STAGE");//motion_mode=STOP;
                  } 
                else{
                  turne_Stages=TURN_STRAIGHT2; Serial.println("STRIGHT 2 STAGE");//motion_mode=STOP;
                  }

                  */
                }
            
      break;
      /*
      case CHECK_ANGLE:
              readAngle();
             if( abs(currentAngle-nxtAng)> 2){
                stop();
                if(rotionDirection==ANTI_CLOCKWIZE){
                     if (currentAngle>nxtAng){steeringAngle=STRAIGHT_STEERING+35; setSteering(steeringAngle);}
                     else{steeringAngle=STRAIGHT_STEERING-43; setSteering(steeringAngle);}
                  }

                 else{
                     if (currentAngle>nxtAng){}
                     else{}
                  }
                  turne_Stages=CORRECT_ANGLE;
                  delay(300);
                }
              else{turne_Stages=CHECK;}
      break;
      case CORRECT_ANGLE:
             forward(SLOW_SPEED);
              readAngle();
             if(abs(currentAngle-nxtAng)<3){stop(); setSteering(STRAIGHT_STEERING);turne_Stages=CHECK}
      //break;
      */
      case CHECK:
           Serial.println("IN CHECK");
            rangeB=readBackDistance(5);
           // Serial.print("Back Distance"); Serial.println(rangeB);
           if(rangeB<85){encoderReviece=100; turne_Stages=FORWARD2; encoder_count=0; }
           else if(rangeB>88){encoderReviece=100; turne_Stages=REVERSE2;}
           else {turne_Stages=FINISH_TURN; }
          
           
      break;
      
      case REVERSE2:
              rangeB=readBackDistance(5);
            Serial.print("in revirse"); Serial.println(rangeB);
           setSteering(STRAIGHT_STEERING);
           reverse(SLOW_SPEED);
           if(rangeB<88/*encoder_count>=encoderReviece*/){stop(); turne_Stages=FINISH_TURN;}
      break;
      case FORWARD2:
           rangeB=readBackDistance(5);
         //  Serial.print("in Forward"); Serial.println(rangeB);
           setSteering(STRAIGHT_STEERING);
           forward(SLOW_SPEED);
           if(rangeB>85/*encoder_count>=encoderReviece*/){stop(); turne_Stages=FINISH_TURN;}
      break;
      
      case FINISH_TURN:
        //  Serial.print("Final Back Distance"); Serial.println(rangeB);KKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK
          obstacle_Stage=FIRST;
          motion_mode=BTWEEN_OB;
         // stop();
          //  rangeB=readBackDistance(5);
           //Serial.print("Back Distance: "); Serial.println(rangeB);
      //   motion_mode=STOP;
          
      break;
 
    }
  }

  // turn in corner and ready to ... between ob.s
void  doTurn2(){
  

  
    switch(move_Stage){
      case STAGE1:
      
       rounCounter++;
       turnCalculatons();
        if(rotionDirection==ANTI_CLOCKWIZE ){ readDistanseFrom=RIGHT_FORWARD;}else {readDistanseFrom=LIFT_FORWARD;}
       readDistance(readDistanseFrom);
       move_Stage=STAGE2;
      // motion_mode=STOP;
      break;
      case STAGE2:
       
       turnInCorner();
             
      break;
      case STAGE3:
      // between obest.....
       
      break;
     
 
    }
  
  
  
  }
  
void doLevel2(){
 // if(pidState){steering.write(steeringPID); } 
  switch (motion_mode)  
  {
  case STANDBY:
 
  break;
  case FORWARD:
     readDistanseFrom=LEFT_RIGHT;
     doForward2(); 
     
     
    // GoComplementary();
  
   
  break;
  case ENTER_CORNER:
       doEnterCorner();
  break;
  case LEARN:
       OB_Learn();
  case CORNER:
    //goToCorner();
  break;
  case TURNE:
   doTurn2();
  break;
  case BTWEEN_OB:
    betweenOBbstacles();
  break;
  case PARKING:
    //doParking2();
  break;
  case STOP:
    stop();
    isStarted=false ; //Serial.print("Time is : ");Serial.println((millis()-runTime)/1000);            
   delay(1000);
   readAngle();
   debugRotation();
    ESP.restart();
  break;

  }
  }
  
