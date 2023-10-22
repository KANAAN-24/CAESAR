

#define R 0
#define L 1

#define LERARN_TIMEOUT 5000
unsigned long startLearn;
unsigned long runTime2 = 0;
char currentPosition, nextPosition;
int C, U;
int Fact;
int strightEncoder1, strightEncoder2, nxtAng;  //encoder values and next angle  for  level2 turn
int readDistanseFrom;
unsigned long encoderCounterPreTurn = 0;
bool testdirectionMore, near, printArray;
int encoderLeft, encoderRight, encoderStright, encoderReviece;
int roundNo, turnNo;
int rDirection;
String obstacles[4];
int obIndex;

bool isStarted2;
int dist, preTurnDist;
double firstroundDest = 0.0;

#include "betweenOb.h"
void debugLevel2() {
  Serial.print("encoder_count: ");
  Serial.print(encoder_count);
  Serial.print("rounCounter: ");
  Serial.print(rounCounter);
  Serial.print("setpoint: ");
  Serial.print(setpoint);
  Serial.println();
}
void initLevel2() {
  obIndex = 0;
  C = U = 0;
  readDistanseFrom = LEFT_RIGHT;
  //readDistanseFrom=LIFT_FORWARD ;
  //readDistanseFrom=RIGHT_FORWARD ;
  rDirection = 0;
  near = false;
  testdirectionMore = false;
  isStarted2 = false;
  currentAngle = (int)mpu.getAngleZ();
  initAngle = currentAngle;
  angle0 = currentAngle;
  motion_mode = STANDBY;
  initR = rangeR;
  initL = rangeL;
  initF = rangeF;
  roundNo = turnNo = 0;
  rounCounter = 0;
  rotionDirection = UNKNOWN_DIRECTION;
  setpoint = initR;
  dist = 40;
}
bool dalta(int side) {
  int del;
  if (side == R) {
    del = initR - rangeR;
    if (abs(del) >= 20) return true;
    else return false;
  } else {
    del = initL - rangeL;
    if (abs(del) >= 20) return true;
    else return false;
  }
}


void detectDirection() {
  debugUltrasonic();
  if (rangeR > 110) {
    C++;
    led(WHITE);
    testdirectionMore = true;
  } else if (rangeL > 110) {
    U++;
    rDirection = rDirection + 1;
    led(WHITE);
    testdirectionMore = true;
  }


  if (testdirectionMore) {
    stop();
    for (int i = 0; i < 2; i++) {
      readDistanseFrom = LEFT_RIGHT;
      delay(20);
      readDistance(readDistanseFrom);
      led(WHITE);
      if (dalta(R)) {
        C++;
        rDirection = rDirection + 0;
        led(WHITE);
        testdirectionMore = true;
      }
      if (dalta(L)) {
        U++;
        rDirection = rDirection + 1;
        led(WHITE);
        testdirectionMore = true;
      }
    }

    if (U > C) {
      rotionDirection = ANTI_CLOCKWIZE;
      Serial.println("Direction UntiClock");
      forward(SPEED2);
    } else if (C > U) {
      rotionDirection = CLOCKWIZE;
      Serial.println("Direction CLOCK wize");
      forward(SPEED2);
    } else {
      rotionDirection = UNKNOWN_DIRECTION;
    }


    if (rotionDirection == ANTI_CLOCKWIZE) {
      Serial.println("ANTI CLOCK WIZE");
      if (initL < 25) near = true;
      readDistanseFrom = RIGHT_FORWARD;
    } else if (rotionDirection == CLOCKWIZE) {
      Serial.println("CLOCK WIZE");
      if (initR < 25) near = true;
      setpoint = initL;
      readDistanseFrom = LIFT_FORWARD;
    }
  }
}


void cornerCalculations() {

  if (rounCounter == 0) {
    if (rotionDirection == CLOCKWIZE) {
      corner_Stage = STRIGHT_ENCODER_;
      if (initL >= 9 && initL <= 26) {
        currentPosition = 'F';
        encoderLeft = 0;
        encoderRight = 0;
        encoderStright = 0;
        encoderReviece = 1100;
      } else if (initL >= 41 && initL <= 48) {
        currentPosition = 'C';
        encoderLeft = 980;
        encoderRight = 1050;
        encoderStright = 0;
        encoderReviece = 950;
      } else if (initL >= 60 && initL <= 79) {
        currentPosition = 'N';
        encoderLeft = 1150;
        encoderRight = 1100;
        encoderStright = 400;
        encoderReviece = 1050;
      }
    } else if (rotionDirection == ANTI_CLOCKWIZE) {
      corner_Stage = STRIGHT_ENCODER_;
      if (initL >= 9 && initL <= 26) {
        currentPosition = 'N';
        encoderLeft = 1000;
        encoderRight = 1000;
        encoderStright = 100;
        encoderReviece = 1100;
      } else if (initL >= 41 && initL <= 48) {
        currentPosition = 'C';
        encoderLeft = 950;
        encoderRight = 800;
        encoderStright = 0;
        encoderReviece = 950;
      } else if (initL >= 60 && initL <= 79) {
        currentPosition = 'F';
        encoderLeft = 0;
        encoderRight = 0;
        encoderStright = 0;
        encoderReviece = 1100;
      }
    }
  }

  if (rounCounter > 0) {

    if (rotionDirection == CLOCKWIZE) {
      corner_Stage = STRIGHT_ENCODER_;
      if (currentPosition == 'F') {
        encoderLeft = 0;
        encoderRight = 0;
        encoderStright = 0;
        encoderReviece = 1100;
      } else if (currentPosition == 'N') {
        encoderLeft = 1150;
        encoderRight = 1100;
        encoderStright = 400;
        encoderReviece = 1050;
      }
    } else if (rotionDirection == ANTI_CLOCKWIZE) {
      corner_Stage = STRIGHT_ENCODER_;
      if (currentPosition == 'N') {
        encoderLeft = 1150;
        encoderRight = 1100;
        encoderStright = 200;
        encoderReviece = 1050;
      } else if (currentPosition == 'F') {
        encoderLeft = 0;
        encoderRight = 0;
        encoderStright = 0;
        encoderReviece = 1100;
      }
    }
  }
  encoderReviece = 850;
  //  motion_mode=STOP;
  encoder_count = 0;
}


void preObstacle() {

  if (obstacles[obIndex].charAt(0) == 'F') {

    switch (pre_Ob_Stage) {
      case GO_:
        setpoint = 16.0;
        setSteering(steeringPID);
        encoderStright = 1300;
        encoder_count = 0;
        pre_Ob_Stage = GO_STRIGHT_ENCODER;
        break;
      case GO_STRIGHT_ENCODER:
        if (encoder_count < encoderStright) {
          forward(SPEED2);
          setSteering(steeringPID);
        } else {
          pre_Ob_Stage = Go_STRIGHT_NEAR_OB;
          encoder_count = 0;
        }

        break;
      case Go_STRIGHT_NEAR_OB:
        if (rotionDirection == ANTI_CLOCKWIZE) {
          if (rangeL < 100 || encoder_count > 300) {
            encoder_count = 0;
            pre_Ob_Stage = GO_STRIGHT_ENCODER2;
          } else {
            forward(SPEED2);
            setSteering(steeringPID);
          }
        } else if (rotionDirection == CLOCKWIZE) {
          if (rangeR < 100 || encoder_count > 300) {
            encoder_count = 0;
            pre_Ob_Stage = GO_STRIGHT_ENCODER2;
          } else {
            forward(SPEED2);
            setSteering(steeringPID);
          }
        }
        obstacle_Stage = FIRST;
        break;
      case GO_STRIGHT_ENCODER2:

        // Serial.println("reach to first ob");
        if (encoder_count < 250) {
          forward(SPEED2);
          setSteering(steeringPID);
        } else {
          currentPosition = 'F';
          pre_Ob_Stage = END_;
          stop();
          delay(300);
          Serial.print("Back Distance=");
          Serial.println(readBackDistance(3));
        }

        break;
      case END_:
        // motion_mode=STOP;
        betweenOBbstacles();
        break;
    }


  } else {
    switch (pre_Ob_Stage) {
      case GO_:
        encoderLeft = 1100;
        encoderRight = 1100;
        if (rotionDirection == ANTI_CLOCKWIZE) {
          encoderStright = constrain(rangeR, 15, 26);
          encoderStright = map(encoderStright, 15, 26, 500, 180);
          Serial.print("Stright=");
          Serial.println(encoderStright);
        } else if (rotionDirection == CLOCKWIZE) {
          encoderStright = constrain(rangeL, 15, 26);
          encoderStright = map(encoderStright, 15, 26, 550, 210);
        }
        //encoderStright=500;
        encoder_count = 0;
        pre_Ob_Stage = Go_FIRST_TURNE;
        break;
      case Go_FIRST_TURNE:
        Serial.println("First turne.");
        if (rotionDirection == ANTI_CLOCKWIZE) {
          steeringAngle = STRAIGHT_STEERING - 40;
          setSteering(steeringAngle);
        } else if (rotionDirection == CLOCKWIZE) {
          steeringAngle = STRAIGHT_STEERING + 32;
          setSteering(steeringAngle);
        }

        if (encoder_count < encoderLeft) {
          forward(SPEED2);
        } else {
          pre_Ob_Stage = GO_STRIGHT;
          encoder_count = 0;
        }

        break;
      case Go_SECOND_TURNE:
        if (rotionDirection == ANTI_CLOCKWIZE) {
          steeringAngle = STRAIGHT_STEERING + 32;
          setSteering(steeringAngle);
        } else if (rotionDirection == CLOCKWIZE) {
          steeringAngle = STRAIGHT_STEERING - 40;
          setSteering(steeringAngle);
        }
        if (abs(currentAngle - initAngle) >= 95) {
          forward(SPEED2);
        } else {
          pre_Ob_Stage = GO_STRIGHT_ENCODER;
        }
        break;
      case GO_STRIGHT:
        setSteering(STRAIGHT_STEERING);
        if (encoder_count < encoderStright) {
          forward(SPEED2);
        } else {
          pre_Ob_Stage = Go_SECOND_TURNE;
        }

        break;
      case GO_STRIGHT_ENCODER:
        setSteering(STRAIGHT_STEERING);
        pre_Ob_Stage = END_;
        obstacle_Stage = FIRST;
        stop();
        delay(300);
        //
        break;
      case GO_REVERCE_ENCODER:
        break;

      case END_:
        currentPosition = 'N';
        Serial.print("Back Distance=");
        Serial.println(readBackDistance(3));
        // motion_mode=STOP;

        betweenOBbstacles();
        break;
    }
  }
}

void correctDirection() {
  int dirFactor = 1;
  int expectedAngle = angle0 + ((rounCounter)*dirFactor * 90);
  if (rotionDirection == CLOCKWIZE) { dirFactor = -1; }
  Serial.println("____________________________________________________________");
  Serial.print(" initAngle =");
  Serial.println(initAngle);
  Serial.print("Round Counter =");
  Serial.println(rounCounter);
  Serial.print("Expected angle =");
  Serial.println(expectedAngle);
  Serial.print("current  angle =");
  Serial.println(currentAngle);
  readAngle();
  if (abs(currentAngle - expectedAngle) > 3) {
    while (abs(currentAngle - expectedAngle) > 3) {
      readAngle();
      Serial.print("dalta angle =");
      Serial.println(currentAngle - expectedAngle);
      if (currentAngle > expectedAngle) { setSteering(STRAIGHT_STEERING + 15); }  //right
      else if (currentAngle < expectedAngle) {
        setSteering(STRAIGHT_STEERING - 15);
      }  //left
      delay(50);
      forward(SLOW_SPEED3);
      if (millis() - runTime2 > 3000) { break; }  //time out
    }
  }
}
void revirceToCorner(int d) {

  while (readBackDistance(1) > d && millis() - runTime2 < 2500) {
    reverse(160);
    delay(10);
    Serial.println(readBackDistance(1));
  }
  stop();
  Serial.print("T");
  Serial.println(millis() - runTime2);
}


void goToCorner() {

  switch (corner_Stage) {

    case START_:
      cornerCalculations();
      break;
    case STRIGHT_ENCODER_:
      if (encoder_count < 200) {
        Serial.print("encoder_count=");
        Serial.println(encoder_count);
        forward(SPEED2);
      } else {
        encoder_count = 0;
        if (rotionDirection == CLOCKWIZE) {
          corner_Stage = TURNE_L_;
        } else if (rotionDirection == ANTI_CLOCKWIZE) {
          corner_Stage = TURNE_R_;
        }
      }
      break;
    case STRIGHT_:
      setSteering(STRAIGHT_STEERING);
      if (encoder_count < encoderStright) {
        forward(SPEED2);
      } else {
        if (rotionDirection == CLOCKWIZE) {
          corner_Stage = TURNE_R_;
        } else if (rotionDirection == ANTI_CLOCKWIZE) {
          corner_Stage = TURNE_L_;
        }
        encoder_count = 0;
      }
      break;
    case TURNE_L_:
      steeringAngle = STRAIGHT_STEERING - 40;
      setSteering(steeringAngle);
      if (encoder_count < encoderLeft) {
        forward(SPEED2);
      } else {
        if (rotionDirection == CLOCKWIZE) {
          corner_Stage = STRIGHT_;
        } else if (rotionDirection == ANTI_CLOCKWIZE) {
          setSteering(STRAIGHT_STEERING);
          corner_Stage = PRE_TURNE90_;
        }
        encoder_count = 0; /* corner_Stage=REVERCE_;*/
      }

      break;
    case TURNE_R_:
      //Serial.println("Turn rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr");
      steeringAngle = STRAIGHT_STEERING + 32;
      setSteering(steeringAngle);
      if (encoder_count < encoderRight) {
        forward(SPEED2);
      } else {
        if (rotionDirection == CLOCKWIZE) {
          setSteering(STRAIGHT_STEERING);
          corner_Stage = PRE_TURNE90_;
        } else if (rotionDirection == ANTI_CLOCKWIZE) {
          corner_Stage = STRIGHT_;
        }
        // motion_mode=STOP;
        encoder_count = 0;
      }
      break;
    case PRE_TURNE90_:

      if (rangeF < 32) {
        corner_Stage = REVERCE_ENCODER_;
        encoder_count = 0;
      }

      else {
        stop();
        corner_Stage = REDEY_TO_TURNE_;
        if (rotionDirection == CLOCKWIZE) {
          setpoint = rangeL;
        } else if (rotionDirection == ANTI_CLOCKWIZE) {
          setpoint = rangeR;
        }
        setSteering(STRAIGHT_STEERING);
      }

      break;
    case REVERCE_ENCODER_:
      Serial.println("Revirce with encoder  mode");
      if (encoder_count < 500) {
        setSteering(steeringREvircePID);
        reverse(SPEED2);
      } else {
        stop();
        corner_Stage = REDEY_TO_TURNE_;
        setSteering(STRAIGHT_STEERING);
        if (rotionDirection == CLOCKWIZE) {
          setpoint = rangeL;
        } else if (rotionDirection == ANTI_CLOCKWIZE) {
          setpoint = rangeR;
        }
        encoder_count = 0;
      }
      break;
    case REDEY_TO_TURNE_:
      Serial.println("Redy to turne mode");
      if (rangeF < 50) {
        forward(SLOW_SPEED3);
        setSteering(steeringPID);
      } else {
        forward(SPEED2);
        setSteering(steeringPID);
      }

      if (rangeF > 35) {
        setSteering(steeringPID);
      } else {
        corner_Stage = TURNE90_;
        encoder_count = 0;
      }
      break;
    case TURNE90_:
      Serial.println("Turne 90 Mode");

      Fact = 1;
      if (rotionDirection == CLOCKWIZE) { Fact = -1; }
      nxtAng = angle0 + ((rounCounter + 1) * Fact * 90);

      if (rotionDirection == CLOCKWIZE) {
        steeringAngle = STRAIGHT_STEERING + 32;
        setSteering(steeringAngle);
      } else if (rotionDirection == ANTI_CLOCKWIZE) {
        steeringAngle = STRAIGHT_STEERING - 35;
        setSteering(steeringAngle);
      }
      if (encoder_count < 1350) {
        forward(SPEED2);
      } else {
        forward(SLOW_SPEED3);
      }

      if (abs(currentAngle - nxtAng) < 3 /* && (encoder_count-encoderCounterPreTurn)>1690*/) {

        Serial.print("Turn Encoder count=");
        Serial.println(encoder_count);
        stop();
        steering.write(STRAIGHT_STEERING);
        setpoint = 15;
        encoder_count = 0;
        corner_Stage = REVERCE_;
      }
      break;
    case REVERCE_:
      //motion_mode=STOP;
      Serial.println("Revirce modeeeee");
      steering.write(STRAIGHT_STEERING);
      // steering.write(steeringREvircePID);//setpoint
      if (encoder_count < encoderReviece) {
        reverse(SPEED2);
      }

      else {
        stop();
        delay(50);
        currentPosition = 'F';
        obIndex = (obIndex + 1) % 4;
        rounCounter++;
        corner_Stage = ALIGNMENT_;
      }
      runTime2 = millis();
      break;
    case ALIGNMENT_:
      //correctDirection();
      stop();
      steering.write(STRAIGHT_STEERING);
      delay(50);
      corner_Stage = CORRECT_POS;
      break;
    case CORRECT_POS:
      revirceToCorner(7);
      if (rotionDirection == ANTI_CLOCKWIZE) {
        Serial.write("@ANTI");
      } else {
        Serial.write("@CLOCK");
      }
      startLearn = millis();
      corner_Stage = SEE_OBSTACLE_;
      delay(200);  // motion_mode=STOP;

      break;
    case SEE_OBSTACLE_:  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&#####################
                         //Serial.print("init Angle="); Serial.println(angle0); //rounCounter++;
                         //Serial.print("Current angle Angle="); Serial.println(currentAngle);
      // motion_mode=STOP;
      if (millis() - startLearn > LERARN_TIMEOUT) {
        obstacles[obIndex] = "FFF";
        corner_Stage = GO_TO_OB_;
      } else {
        readSerial();
        if (msg != "") {
          obstacles[obIndex] = msg.substring(0, 3);
          corner_Stage = GO_TO_OB_;
        }
      }
      currentPosition = 'F';
      readDistanseFrom = LEFT_RIGHT;
      pre_Ob_Stage = GO_;
      encoder_count = 0;
      printArray = true;
      break;
    case GO_TO_OB_:
      if (printArray) {
        Serial.println("Learn array is ");
        Serial.print("[0] ");
        Serial.println(obstacles[0]);
        Serial.print("[1] ");
        Serial.println(obstacles[1]);
        Serial.print("[2] ");
        Serial.println(obstacles[2]);
        Serial.print("[3] ");
        Serial.println(obstacles[3]);
        printArray = false;
      }

      preObstacle();

      break;
  }
}

////////////////////////////////////////////

void setPIDSettings(char pos) {
  if (pos == 'N') {
    setpoint = 71;
    complementaryMode = true;
  } else {
    setpoint = 15;
    complementaryMode = false;
  }
}

void findNextPosition() {
  int i = (rounCounter + 1) % 4;
  if (obstacles[i].charAt(0) == 'N' || obstacles[i].charAt(0) == 'F') {
    nextPosition = obstacles[i].charAt(0);
  } else {
    if (obstacles[i].charAt(1) == 'N' || obstacles[i].charAt(1) == 'F') {
      nextPosition = obstacles[i].charAt(1);
    } else {
      if (obstacles[i].charAt(2) == 'N' || obstacles[i].charAt(2) == 'F') {
        nextPosition = obstacles[i].charAt(2);
      } else {
        nextPosition = 'F';
      }
    }
  }
}

void turnCalculations() {
  Fact = 1;
  if (rotionDirection == CLOCKWIZE) { Fact = -1; }
  findNextPosition();

  nxtAng = angle0 + 90;
  // nxtAng=angle0+(rounCounter*Fact*90);
  Serial.print("Current position: ");
  Serial.println(currentPosition);
  Serial.print("Next position: ");
  Serial.println(nextPosition);
  Serial.print("Next angle : ");
  Serial.println(nxtAng);
  Serial.print("Current angle : ");
  Serial.println(currentAngle);

  if (currentPosition == 'N' && nextPosition == 'N') {
    setpoint = 70;
    strightEncoder1 = 0;
    strightEncoder2 = 0;
  } else if (currentPosition == 'N' && nextPosition == 'F') {
    setpoint = 70;
    strightEncoder1 = 1650;
    strightEncoder2 = 0;
  } else if (currentPosition == 'F' && nextPosition == 'N') {
    setpoint = 15;
    strightEncoder1 = 100;
    strightEncoder2 = 800;
  } else if (currentPosition == 'F' && nextPosition == 'F') {
    setpoint = 15;
    strightEncoder1 = 1650;
    strightEncoder2 = 800;
  }
}

/*
 rounCounter=7;
 rotionDirection=ANTI_CLOCKWIZE;
 currentPosition='F';
 move_Stage=STAGE1;
 obstacles[0]="FFF";
 obstacles[0]="FFF";
 obstacles[0]="FFF";

 */
void doTurn2() {

  switch (turne_Stages) {
    case TURN_CALCULATIONS:
      turnCalculations();
      // setSteering(STRAIGHT_STEERING);
      turne_Stages = TURN_STRAIGHT1;
      encoder_count = 0;
      break;
    case TURN_STRAIGHT1:
      readForwardDistance(1);
      if (encoder_count < strightEncoder1 && rangeF > 34) {
        if (rangeF < 50) {
          forward(SLOW_SPEED3);
        } else {
          forward(SPEED2);
        }
        setSteering(steeringPID);
      }

      else {
        Serial.print("Encoder Counter pre turn=");
        Serial.println(encoder_count);
        turne_Stages = DO_TURN;
        encoder_count = 0;
      }
      break;
    case DO_TURN:

      //Serial.print(" currentAngle=");  Serial.println(currentAngle);
      if (rotionDirection == ANTI_CLOCKWIZE) { setSteering(STRAIGHT_STEERING - 40); } /*turn left*/
      else if (rotionDirection == CLOCKWIZE) {
        setSteering(STRAIGHT_STEERING + 40);
      } /*turn right*/
      if (abs(currentAngle - nxtAng) >= 3 && encoder_count < 2500) {
        if (encoder_count < 1450) {
          forward(SPEED2);
        } else {
          forward(SLOW_SPEED3);
        }
      } else {
        Serial.print("Encoder Counter after turn=");
        Serial.println(encoder_count);
        setPIDSettings(nextPosition);
        turne_Stages = TURN_STRAIGHT2;
        encoder_count = 0;
      }
      break;
    case TURN_STRAIGHT2:

      if (encoder_count < strightEncoder2) {
        forward(SPEED2);
        setSteering(steeringPID);
      } else {
        turne_Stages = FINISH_TURN;
        encoder_count = 0;
      }
      break;

    case FINISH_TURN:
      motion_mode = STOP;
      currentPosition = nextPosition;
      rounCounter++;

      break;
  }
}


void doParking2() {
  if ((rangeF < 146 && rangeF != 0) && encoder_count > 2180) {
    motion_mode = STOP;
    led(WHITE);
    Serial.print("Encoder");
    Serial.println(encoder_count);
  } else {
    forward(SPEED2);
    steering.write(steeringPID);
  }
}
void doEnterCorner() {
  // correctDirection();
  setPIDSettings(currentPosition);


  readDistanseFrom = LEFT_RIGHT;
  readDistance(readDistanseFrom);
  forward(SLOW_SPEED);
  steering.write(steeringPID);
  rangeF = readForwardDistance(2);
  Serial.print("rangeF=");
  Serial.print(rangeF);
  if ((rotionDirection == ANTI_CLOCKWIZE && rangeL > 120) && rangeF < 95) { /*stop();*/
    move_Stage = STAGE2;
  } else if ((rotionDirection == CLOCKWIZE && rangeR > 120) && rangeF < 95) { /*stop();*/
    move_Stage = STAGE2;
  }
}
///FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
void doForward2() {



  if (rotionDirection == UNKNOWN_DIRECTION) {
    forward(SLOW_SPEED);
    detectDirection();
  }  // if direction not set
  else {
    if (rounCounter < 4) {


      switch (move_Stage) {
        case STAGE1:
          doEnterCorner();
          break;
        case STAGE2:
          readForwardDistance(1);
          if (rotionDirection == CLOCKWIZE && rangeF < 95) {
            initAngle = currentAngle;
            motion_mode = CORNER;
            corner_Stage = START_;
            encoder_count = 0;
          } else if (rotionDirection == ANTI_CLOCKWIZE && rangeF < 95) {
            initAngle = currentAngle;
            motion_mode = CORNER;
            corner_Stage = START_;
            encoder_count = 0;
          }
          //***
          if (rotionDirection == ANTI_CLOCKWIZE) {
            setpoint = 16;
            readDistanseFrom = RIGHT_FORWARD;
          } else if (rotionDirection == CLOCKWIZE) {
            setpoint = 16;
            readDistanseFrom = LIFT_FORWARD;
          }
          //***
          break;
      }
    }

    else if (rounCounter >= 4 && rounCounter < 8) {
      Serial.println("we  are in second rounds");
      switch (move_Stage) {

        case STAGE1:
          Serial.println("we  are in stage1 ");
          doEnterCorner();
          break;
        case STAGE2:
          readForwardDistance(1);
          if (rotionDirection == CLOCKWIZE && rangeF < 95) {
            initAngle = currentAngle;
            motion_mode = TURNE;
            turne_Stages = TURN_CALCULATIONS;
            encoder_count = 0;
          } else if (rotionDirection == ANTI_CLOCKWIZE && rangeF < 95) {
            initAngle = currentAngle;
            motion_mode = TURNE;
            turne_Stages = TURN_CALCULATIONS;
            encoder_count = 0;
          }

          break;
      }


      /*motion_mode=STOP;*/
    } else {
      motion_mode = STOP;
    }
  }
}

//EFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEFEF
//the main function of level 1
void doLevel2() {
  // if(pidState){steering.write(steeringPID); }
  switch (motion_mode) {
    case STANDBY:
      break;
    case FORWARD:
      doForward2();
      break;
    case ENTER_CORNER:
      doEnterCorner();
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
      isStarted = false;
      Serial.print("Time is : ");
      Serial.println((millis() - runTime) / 1000);
      ESP.restart();
      break;

  }
  }
  
