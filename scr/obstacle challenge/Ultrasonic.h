#define ALL 0
#define LEFT_RIGHT 1
#define LIFT_FORWARD 2
#define RIGHT_FORWARD 3


#define TIMEOUT 16500
HardwareSerial ultraSensor(1); 
unsigned int rangeR,rangeL,rangeF;
unsigned int initR,initL,initF;
unsigned int prevRangeR,prevRangeL,prevRangeF;

unsigned long UltraSonicUpdateTime;
float distance;
unsigned long period;
void initUltrasonic(){
pinMode(ECHO_R,INPUT);
pinMode(TRIG_R,OUTPUT);
pinMode(ECHO_L,INPUT);
pinMode(TRIG_L,OUTPUT);
pinMode(ECHO_F,INPUT);
pinMode(TRIG_F,OUTPUT);
}

int findeDist(byte E,byte T){
int d;
digitalWrite(T,LOW);
delayMicroseconds(5);
digitalWrite(T,HIGH);
delayMicroseconds(10);
digitalWrite(T,LOW);
period=pulseIn(E,HIGH,17492);
distance=float(period)/58.8;
d=(int)distance; 
d=constrain(d, 0, 301);
if(d==0){d=300;}
return(d);
}

void readUltrasonic()
{
  prevRangeR=rangeR;
  prevRangeL=rangeL;
  prevRangeF=rangeF;
  rangeR=findeDist(ECHO_R,TRIG_R);
  rangeF=findeDist(ECHO_F,TRIG_F);
  rangeL=findeDist(ECHO_L,TRIG_L);
 // rangeF=findeDist(ECHO_F,TRIG_F);
  
}

void readDistance(int readCase){
  prevRangeR=rangeR;
  prevRangeL=rangeL;
  prevRangeF=rangeF;

  switch (readCase){
    case  ALL:
      rangeR=findeDist(ECHO_R,TRIG_R);
      rangeF=findeDist(ECHO_F,TRIG_F);
      rangeL=findeDist(ECHO_L,TRIG_L);
      delay(1);
    break; 
    case LEFT_RIGHT :
      rangeR=findeDist(ECHO_R,TRIG_R);
      delay(2);
      rangeL=findeDist(ECHO_L,TRIG_L);
      delay(2);
      
    break; 
    case LIFT_FORWARD:
      rangeF=findeDist(ECHO_F,TRIG_F);
      delay(2);
      rangeL=findeDist(ECHO_L,TRIG_L);
      delay(2);
    break;
    case RIGHT_FORWARD:
      rangeR=findeDist(ECHO_R,TRIG_R);
      delay(2);
      rangeF=findeDist(ECHO_F,TRIG_F);
      delay(2);
    break; 
    }
  
  }

void debugUltrasonic(){
  Serial.print("R: ");Serial.print(rangeR);
  Serial.print("  L: ");Serial.print(rangeL);
  Serial.print("  F: ");Serial.print(rangeF);
  Serial.println();}

/*
char c;
String readString;

void initUltrasonic(){
  ultraSensor.begin(9600,SERIAL_8N1,RXD2,TXD2);
 
  rangeR=rangeL=rangeF=0;
  UltraSonicUpdateTime=0;
  
   }
void readUltrasonic(){
    readString="";
    while (ultraSensor.available()) {
    c = ultraSensor.read();
    readString += c;
    Serial.println(c);
  }
  if (readString.length() > 0) {Serial.print(readString);}
  }
*/
