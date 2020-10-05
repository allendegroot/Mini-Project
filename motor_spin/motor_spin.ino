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
int toSend;

long newPosition;


int controlResult = 0;
const float Kp = 0.430620391450113;
const float Ki = 0.032853330385174;
float thetaDesired = 0;
float totalError = 0;
float controlSignal;
int finalControlSignal;


//void sendData(){
  //Wire.write(data);
//}


Encoder myEnc(2, 3);

unsigned long currentTime = 0;
int samplePeriod = 10;
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
  analogWrite(motor2Speed, controlResult);
  
  //4.4
  newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    
    oldPosition = newPosition;
    angularPositionNew = (newPosition * 2 * pi) / 3200;
    //Serial.println(angularPositionNew);
    
    //Serial.println(angularPositionNew - angularPositionOld);
    angularVelocity = ((angularPositionNew - angularPositionOld) / (samplePeriod)) * pow(10,3);
    angularPositionOld = angularPositionNew;

      
  }

  controlResult = controller(angularPositionNew);
  
  
  while(millis() < currentTime + samplePeriod){
    
  }
  
}
void receiveData(int byteCount){
  while(Wire.available()){
    data = Wire.read();
    //Serial.println(data);
  }
  totalError = 0;
  
  if(data == 1){
    thetaDesired = (2*pi)+angularPositionNew;
  }else if(data == 2){
    thetaDesired = (pi/2)+angularPositionNew;
  }else if(data == 3){
    thetaDesired = (pi)+angularPositionNew;
  }else if(data == 4){
    thetaDesired = (3*pi/2)+angularPositionNew;
  }else if(data == 5){
    // This is to result the wheel to the 0 position
    long currentPosition = myEnc.read();
    float currentAngularPosition = (currentPosition * 2* pi) / 3200;

    //Serial.println(currentAngularPosition);
    currentPosition = 3200 - (currentPosition % 3200);
    float angularPositionDesired = currentAngularPosition + ((currentPosition * 2 * pi) / 3200);
    
   //Serial.println(angularPositionDesired);
    thetaDesired = angularPositionDesired;
  }else{
    thetaDesired == 0;
  }
}
void sendData(){
  toSend = 2*pi*(newPosition%3200) / 3200;
  Wire.write(toSend);
}

int controller(double angularPositionNew){
  float newError = thetaDesired - angularPositionNew;  
  totalError += newError;
  controlSignal = (Kp*newError)  + (Ki*(samplePeriod/1000)*totalError);  
  finalControlSignal = controlSignal * 255;
  if(finalControlSignal > 255){
    finalControlSignal = 255;
  }else if(finalControlSignal < 0){
    finalControlSignal = 0;
  }
  return finalControlSignal;
}



  
  
