#include <Encoder.h>
#include <math.h>
#include <Wire.h> 
#define ADDRESS 0x08
int nD2 = 4; // Motor Tristate
int motor1Dir = 7; //Better realized as Voltage Sign
int motor2Dir = 8; //Better reawlized as Voltage Sign
int motor1Speed = 9; //Better realized as Voltage of M1
int motor2Speed = 10; //Better realized as Voltage of M2

int nSF = 12; //Status Flag Indicator

long oldPosition  = -999;
const float pi = 3.1415926535898;
double angularPositionNew;
double angularPositionOld;
double angularVelocity;
int data = 0;
int toSend = 0;




//void sendData(){
  //Wire.write(data);
//}


Encoder myEnc(2, 3);

unsigned long currentTime = 0;
int samplePeriod = 10;
double deltaT;
unsigned long oldTime;
void setup() {  

  
  //Pins 4 Digital 4 - nD2 - triState
  pinMode(nD2, OUTPUT);
  
  //7,8 - Voltage Sign
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  //9, 10 Motor Voltage
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
  
  //Pins 12 for input - nSF - Status flag indicator
  pinMode(nSF, INPUT);

  digitalWrite(nD2, HIGH);
  digitalWrite(motor1Dir, HIGH); //High for CW
  digitalWrite(motor2Dir, HIGH); //Low for CCW

  //analogWrite(motor1Speed, HIGH);
  //analogWrite(motor2Speed, HIGH);

  Serial.begin(9600);
  Wire.begin(ADDRESS);
  Serial.println("Basic Encoder Test:");
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

}

void loop() {

  currentTime = millis();

  
  
  //Anolog write for pin 9
  if (currentTime <= 3000){
    analogWrite(motor2Speed, 0);
  }
  else if(currentTime >= 3000) {
    analogWrite(motor2Speed, 128);
  }

  else{
    analogWrite(motor2Speed, 0);
  }

  
  
  //4.4
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    
    oldPosition = newPosition;
    angularPositionNew = (newPosition * 2 * pi) / 3200;
    
    //Serial.println(angularPositionNew - angularPositionOld);
    angularVelocity = ((angularPositionNew - angularPositionOld) / (samplePeriod)) * pow(10,3);
    angularPositionOld = angularPositionNew;

  //  Serial.print((currentTime-2000));
    //Serial.print("\t");
    //Serial.print(angularVelocity);
    //Serial.println();
    
    
  }
  
  while(millis() < currentTime + samplePeriod){
    
  }
  
}
  void receiveData(int byteCount){
  while(Wire.available()){
    data = Wire.read();
    Serial.println(data);
  }
}
void sendData(){
  toSend = angularPositionNew;
  Wire.write(toSend);
}
