// Includes the necessary libraries
#include <Encoder.h>
#include <math.h>
#include <Wire.h> 
// Defines the address for the LCD screen
#define ADDRESS 0x08
// Defines the necessary constants for the motor
int nD2 = 4; // Motor Tristate
int motor1Dir = 7; //Better realized as Voltage Sign
int motor2Dir = 8; //Better reawlized as Voltage Sign
int motor1Speed = 9; //Better realized as Voltage of M1
int motor2Speed = 10; //Better realized as Voltage of M2
// This line defines a constant with the value of pi to be used throughout the program
const float pi = 3.1415926535898;
int nSF = 12; //Status Flag Indicator
// Defines global variables and sets them to initial values if required
long oldPosition  = -999;
double angularPositionNew;
double angularPositionOld;
double angularVelocity;

int data = 0;
int toSend;
long newPosition;
int controlResult = 0;
unsigned long currentTime = 0;
unsigned long oldTime;

// Defines constants necessary for the PI controller
const float Kp = 0.430620391450113;
const float Ki = 0.032853330385174;
// Variable that holds the position of where the motor should turn to
float thetaDesired = 0;
float totalError = 0;
float controlSignal;
int finalControlSignal;

// Defines the sampling period for the program
int samplePeriod = 10;

// Initializes the encoder
Encoder myEnc(2, 3);



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

// Initializes the serial monitor and sets up the Arduino for I2C communication
  Serial.begin(9600);
  Wire.begin(ADDRESS);
  Serial.println("Basic Encoder Test:");
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

}

void loop() {
  // Samples the current time
  currentTime = millis();
  //Anolog write for pin 9
  analogWrite(motor2Speed, controlResult);
  
  //4.4
  // Reads the current position of the encoder
  newPosition = myEnc.read();
  // Updates the current position and velocity of the motor based on new data from the encoder
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    angularPositionNew = (newPosition * 2 * pi) / 3200;
    
    angularVelocity = ((angularPositionNew - angularPositionOld) / (samplePeriod)) * pow(10,3);
    angularPositionOld = angularPositionNew;     
  }

// Gives the new position to the PI controller to be used to determine the new PWM value
  controlResult = controller(angularPositionNew);
  
  // Ensures the program continues to operate at a constant sampling rate
  while(millis() < currentTime + samplePeriod){  }
}


void receiveData(int byteCount){
  // Reads in data over the I2C communication line when data is available
  while(Wire.available()){
    data = Wire.read();
    //Serial.println(data);
  }
  // Resets the total error term for the PI controller
  totalError = 0;
  // This maps the position of the Aruco image to a desired position for the wheel

  // If a 1 is received turn the wheel 2 pi radians
  if(data == 1){                                  // If a 1 is received turn the wheel 2 pi radians
    thetaDesired = (2*pi)+angularPositionNew;
  }else if(data == 2){                            // If a 2 is received turn the wheel pi/2 radians
    thetaDesired = (pi/2)+angularPositionNew;
  }else if(data == 3){                            // If a 3 is received turn the wheel pi radians
    thetaDesired = (pi)+angularPositionNew;
  }else if(data == 4){                            // If a 4 is received turn the wheel  3*pi/2 radians
    thetaDesired = (3*pi/2)+angularPositionNew;
  }else if(data == 5){                            // If a 5 is received return the wheel to the 0 position
    // This is to result the wheel to the 0 position
    long currentPosition = myEnc.read();
    float currentAngularPosition = (currentPosition * 2* pi) / 3200;
    currentPosition = 3200 - (currentPosition % 3200);
    float angularPositionDesired = currentAngularPosition + ((currentPosition * 2 * pi) / 3200);
    thetaDesired = angularPositionDesired;
  }else{
    thetaDesired == 0;
  }
}

// This is the function that sends data over the I2C communication line
void sendData(){
  // Sends the current position in radians over the communication line to be displayed on the LCD screen
  toSend = 2*pi*(newPosition%3200) / 3200;
  Wire.write(toSend);
}


// This is the implementation of the PI controller
int controller(double angularPositionNew){
  // New error is used in the proportional term
  float newError = thetaDesired - angularPositionNew; 
  // Total error is used in the integration term 
  totalError += newError;
  // Calculates the new value based on the current and desired position
  controlSignal = (Kp*newError)  + (Ki*(samplePeriod/1000)*totalError);
  // Scales the result to the scale for PWM values  
  finalControlSignal = controlSignal * 255;
  // Ensures the PWM signal stays within the proper bounds
  if(finalControlSignal > 255){
    finalControlSignal = 255;
  }else if(finalControlSignal < 0){
    finalControlSignal = 0;
  }
  return finalControlSignal;
}



  
  
