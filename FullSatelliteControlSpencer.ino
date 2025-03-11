/* Full Satellite Control Spencer
 *  This adds in mass balancer movement functionality based on MassBalancerArduino
 *  This should be modified slightly to view the IMU reading, as in FullSatelliteControlHunter,
 *  and send serial commands which can be read by the Python script for actual commands in the
 *  Simulink model. This is a rough draft.
*/

/****************************************************************************
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* Accelerometer.ino
* Date: 2014/09/09
* Revision: 3.0 $
*
* Usage:        Example code to stream Accelerometer data
*
****************************************************************************
/***************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
*/

#include "Arduino_NineAxesMotion-master.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

//IMU SETUP

NineAxesMotion mySensor;                 //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 50;          //To stream at 25Hz without using additional timers (time period(ms) =1000/frequency(Hz))
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream
unsigned long lastTime = 0;    // To store the last streamed timestamp
const int Period = 1000;       // Time period(ms) =1000/frequency(Hz)
int count1 = 0;
int count2 = 0;
int count3 = 0;
float RPS1 = 0;
float RPS2 = 0;
float RPS3 = 0;
int lastState1 = LOW;                 // Store the last state of the pin
int lastState2 = LOW;                 // Store the last state of the pin
int lastState3 = LOW;                 // Store the last state of the pin

//THRUSTER SETUP

#define VALVE_1 48
#define VALVE_2 49
#define VALVE_3 50
#define VALVE_4 51
#define VALVE_5 52
#define VALVE_6 53

String CommandRaw;
String valves;
String motor1Command;
String motor2Command;
String motor3Command;
byte valveCommandRaw;
int valveCommand1 = 0;
int motor1CommandRaw;
int motor2CommandRaw;
int motor3CommandRaw;

//MOTOR SETUP

const int pwmPins[] = {44, 45, 46};         // Pins where the motor PWM signals are connected
const int dirPins[] = {36, 38, 40};           // Pins where the motor direction signals are connected

// MASS BALANCER SETUP, see if Rob changed this (the pins for one mass balancer should at least work, check if taken though)
const int MBspeed[] = {5, 6, 7};          // Speed = Purple Wire. Motor direction connected to digital pin 5 (PWM)
const int MBdirection[] = {24, 25, 26};     // Direction = Green Wire. Motor speed connected to digital pin 24.
const int MBrunstop[] = {28,29,30};       // On/Off = White Wire. Run/Stop connected to digital pin 28.
const int MBenable[] = {32, 33, 34};        // Enable = Blue Wire. Enable connected to digital pin 32.
// const int MBVplus[] = {XX, YY, ZZ} ;     // Voltage = Red Wire. Connected to power supply, will be battery in future.
// const int MBground[] = {XX, YY, ZZ};     // Ground = Black Wire. Connected to power supply.
// const int MBhlspeed[] = {XX, YY, ZZ};    // H/L = Yellow Wire. Unused.   
// const int MBvr[] = {XX, YY, ZZ};         // Vr = Orange Wire. Unused, even in the future.

// ISC02-04 Stepper Motor Constants Given from Spec Sheet
const int stepsPerRev = 200; const int microstepsPerRev = 3200;   // Stepper has 200 steps/rev (360 deg/1.8 deg step) and 3200 microsteps/rev 
// const int commandedRevs = 2;                                      // Change to your desired number of revs.
// const int commandedSteps = commandedRevs*microstepsPerRev;        // Microsteps more accurate, could do steps per rev as well
const int stepDelay = 500;                                        // Short pause needed for discrete stepping

void printFixedLength(float value, int totalLength) 
{
  char buffer[totalLength + 1];
  int precision = 3;
  dtostrf(value, totalLength, precision, buffer);
  int j = 0;
  while (buffer[j]==' '){
    j++;
  }
  if (j > 0) {
    memmove(buffer, buffer + j, strlen(buffer) -j +1);
  }
  int currentLength = strlen(buffer);
  if (currentLength < totalLength)
    {
      for(int i = currentLength; i  < totalLength; i++)
      {
        buffer[i]='0';
      }
      buffer[totalLength] = '\0';
    }
  Serial.print(buffer);
}


void setup() //This code is executed once
{

  //Peripheral Initialization
  Serial.begin(9600);           //Initialize the Serial Port to view information on the Serial Monitor
  Wire.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);  //The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();
  updateSensorData = true;

  pinMode(VALVE_6, OUTPUT);
  pinMode(VALVE_5, OUTPUT);
  pinMode(VALVE_4, OUTPUT);
  pinMode(VALVE_3, OUTPUT);
  pinMode(VALVE_2, OUTPUT);
  pinMode(VALVE_1, OUTPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);

    for (int i = 0; i < 3; i++) {
    pinMode(pwmPins[i], OUTPUT);           // Set the PWM pins as outputs
    pinMode(dirPins[i], OUTPUT);           // Set the DIR pins as outputs

    // ADD MASS BALANCERS OUTPUT HERE
    pinMode(MBspeed[i], OUTPUT);
    pinMode(MBdirection[i], OUTPUT);
    pinMode(MBrunstop[i], OUTPUT);
    pinMode(MBenable[i], OUTPUT);           // I feel like we changed this but I do not know if enable is
    digitalWrite(MBenable[i], HIGH);
    }
}

void moveSteps(int steps, bool direction, int balancer) { // How many microsteps to move, which direction, and which balancer (1-3)
  if (direction) {
    digitalWrite(MBdirection[balancer], HIGH); // Set direction to HIGH (e.g., counterclockwise)
  } else {
    digitalWrite(MBdirection[balancer], LOW); // Set direction to LOW (e.g., clockwise)
  }  for (int i = 0; i < steps; i++) {
    digitalWrite(MBspeed[balancer], HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(MBspeed[balancer], LOW);
    delayMicroseconds(stepDelay);
  }
}

void balanceMass() { 
  float gyroX = mySensor.readGyroX();
  float gyroY = mySensor.readGyroY();
  float gyroZ = mySensor.readGyroZ(); // While included here, we are assuming only Z rotation and we can only correct masses in linear sense. 
  // They are limited in 2D plane

  if (abs(gyroX) < 0.1 && abs(gyroY) < 0.1) {// Arbitrary .1 threshold given. May need to increase based on noise of measurement. 
    // Can test by recording flat for sometime and how fluctuation occurs, take standard dev and do 3 sigma.
    isBalanced = true;
    for (int i = 0; i < 3; i++) {
      digitalWrite(MBenable[i], HIGH); // Setting all stepper motors to off when balanced
    }
  } 
  else {
    isBalanced = false;
    for (int i = 0; i < 3; i++) {
      digitalWrite(MBenable[i], LOW);  // Setting all stepper motors to move if needed
      if (gyroX > 0.1) moveSteps(50, true, i); // Set arbitrary movement of 50 microsteps. This SHOULD be microsteps so always small movement.
      else if (gyroX < -0.1) moveSteps(50, false, i);
      if (gyroY > 0.1) moveSteps(50, true, i);
      else if (gyroY < -0.1) moveSteps(50, false, i);
    }
  }
}

// This functionality is for the Python script? I assum to as we are make numbers from the string array, therefore Python can understand?
int array_to_num2(int arr[],int n){
    char str[2][3];
    int i;
    char number[13] = {'\n'};

    for(i=0;i<n;i++) sprintf(str[i],"%d",arr[i]);
    for(i=0;i<n;i++)strcat(number,str[i]);

    i = atoi(number);
    return i;
} 

int array_to_num3(int arr[],int n){
    char str[3][3];
    int i;
    char number[13] = {'\n'};

    for(i=0;i<n;i++) sprintf(str[i],"%d",arr[i]);
    for(i=0;i<n;i++)strcat(number,str[i]);

    i = atoi(number);
    return i;
} 


void loop() //This code is looped forever
{

  mySensor.updateGyro();
  balanceMass();

  if (isBalanced) {
    
  
    int currentState1 = digitalRead(8); // Read the current state of the pin
    int currentState2 = digitalRead(9); // Read the current state of the pin
    int currentState3 = digitalRead(10); // Read the current state of the pin
    balanceMass();
  
    // Detect rising edge
    if (currentState1 == HIGH && lastState1 == LOW) {
      count1++;
    }
     if (currentState2 == HIGH && lastState2 == LOW) {
      count2++;
    }
     if (currentState3 == HIGH && lastState3 == LOW) {
      count3++;
    }
    lastState1 = currentState1; // Update last state
    lastState2 = currentState2; // Update last state
    lastState3 = currentState3; // Update last state
  
    // Calculate RPS and RPM every stream period
    if ((millis() - lastTime) >= Period) {
      lastTime = millis();      // Update the last stream time
      RPS1 = count1;                   // Set RPS as the count
      RPS2 = count2;                   // Set RPS as the count
      RPS3 = count3;                   // Set RPS as the count
      count1 = 0;                      // Reset count for the next period
      count2 = 0;                      // Reset count for the next period
      count3 = 0;                      // Reset count for the next period
    }
    if (updateSensorData)  //Keep the updating of data as a separate task
    {
      mySensor.updateLinearAccel();  //Update the Linear Acceleration data
      mySensor.updateGyro();
      mySensor.updateMag();
      mySensor.updateEuler();
      mySensor.updateCalibStatus();  //Update the Calibration Status
      updateSensorData = false;
    }
    if ((millis() - lastStreamTime) >= streamPeriod)
    {
      lastStreamTime = millis();
  
    //IMU acceleration data
      printFixedLength(mySensor.readLinearAccelX(),8); //Accelerometer X-Axis data
      Serial.print(',');
      printFixedLength(mySensor.readLinearAccelY(),8);  //Accelerometer Y-Axis data
      Serial.print(',');
      printFixedLength(mySensor.readLinearAccelZ(),8);  //Accelerometer Z-Axis data
      Serial.print(',');
  
      //IMU Gyro data
      printFixedLength(mySensor.readGyroX(),8); 
      Serial.print(',');
      printFixedLength(mySensor.readGyroY(),8); 
      Serial.print(',');
      printFixedLength(mySensor.readGyroZ(),8); 
      Serial.print(',');
  
      //IMU Magnetometer data
      printFixedLength(mySensor.readMagX(),8);
      Serial.print(',');
      printFixedLength(mySensor.readMagY(),8);
      Serial.print(',');
      printFixedLength(mySensor.readMagZ(),8);
      Serial.print(',');
  
    //Roll, Pitch, Yaw
      printFixedLength(mySensor.readEulerPitch(),8);
      Serial.print(',');
      printFixedLength(mySensor.readEulerRoll(),8);
      Serial.print(',');
      printFixedLength(mySensor.readEulerHeading(),8);
      Serial.println();
  
      // Print RPS and RPM
      //Serial.println(RPS);
      // Serial.print(RPS1);
      // Serial.print(", "); 
  
      // Serial.print(RPS2);
      // Serial.print(", ");
  
      // Serial.print(RPS3);
      // Serial.println();
      updateSensorData = true;
    }
  
    if (Serial.available()) { // If data comes in from serial monitor, send it out to XBee
        CommandRaw = Serial.readStringUntil('\n');    // read one character from the I2C
        // valveCommandRaw= 0;
        // motor1CommandRaw = 0;
        //motor2CommandRaw = 0;
        //motor3CommandRaw = 0;
  
    // THRUSTER COMMANDS
      int valves1 = CommandRaw[0];
      //Serial.println(valves1);
      int valves2 = CommandRaw[1];
      //Serial.println(valves2);
      int valveCommandA[2] = {valves1,valves2};
  
      int valveCommand[2] = {0,0};
      for(int iv = 0; iv < 2; iv++){
        valveCommand[iv] = valveCommandA[iv] - '0';
        //Serial.println(valveCommand[iv]);
      }
      
      valveCommandRaw= 0;
      for (int iv2 = 0; iv2 < 2; iv2++){
        valveCommandRaw = 10 * valveCommandRaw + valveCommand[iv2];
        //Serial.println(valveCommandRaw);
      }
  
    // REACTION WHEEL COMMANDS
      int motor1[3] = {0,0,0};
      // int motor1test[3] = {0};
      // motor1test[0] = CommandRaw.substring(0,2).toInt();
      // motor1test[1] = CommandRaw.substring(2,4).toInt();
      // motor1test[2] = CommandRaw.substring(4,6).toInt();
      int CommandRawM1[3]={0};
      CommandRawM1[0] = CommandRaw[3];
      CommandRawM1[1] = CommandRaw[4];
      CommandRawM1[2] = CommandRaw[5];
      for(int i1 = 0; i1 < 3; i1++){
        // n1 = i1 + 3;
        motor1[i1] = CommandRawM1[i1] - '0';
        // motor1[i1] = motor1test[i1] - '0';
      }
      
      motor1CommandRaw = 0;
      for (int im1 = 0; im1 < 3; im1++){
        motor1CommandRaw = 10 * motor1CommandRaw + motor1[im1];
        Serial.println(motor1CommandRaw);
      }
  
      int motor2[3] = {0,0,0};
      int CommandRawM2[3]={0};
      CommandRawM2[0] = CommandRaw[7];
      CommandRawM2[1] = CommandRaw[8];
      CommandRawM2[2] = CommandRaw[9];
      for(int i2 = 0; i2 < 3; i2++){
        // n2 = i2 + 7;
        motor2[i2] = CommandRawM2[i2] - '0';
      }
      motor2CommandRaw = 0;
      for (int im2 = 0; im2 < 3; im2++){
        motor2CommandRaw = 10 * motor2CommandRaw + motor2[im2];
      }
  
      int motor3[3] = {0,0,0};
      int CommandRawM3[3]={0};
      CommandRawM3[0] = CommandRaw[11];
      CommandRawM3[1] = CommandRaw[12];
      CommandRawM3[2] = CommandRaw[13];
      for(int i3 = 0; i3 < 3; i3++){
        //n3 = i3 + 11;
        motor3[i3] = CommandRawM3[i3] - '0';
      }
      motor3CommandRaw = 0;
      for (int im3 = 0; im3 < 3; im3++){
        motor3CommandRaw = 10 * motor3CommandRaw + motor3[im3];
      }
  
      // Serial.println(valveCommandRaw);
      // Serial.println(motor1CommandRaw);
      // Serial.println(motor2CommandRaw);
      // Serial.println(motor3CommandRaw);
    
      char valveCommandArray[9] = {0};
      valveCommandRaw += 128;
      itoa(valveCommandRaw, valveCommandArray, 2);
      char* string = valveCommandArray + 2; //get rid of the most significant digit as you only want 7 bits
      for(byte i = 0; i < 7; i++){
        digitalWrite(48+i,string[i] - '0');
      }
      //Serial.println(valveCommandArray);
  
    int motor1Speed = motor1CommandRaw - 255;
      Serial.println(motor1Speed);
      if (motor1Speed <= 0) {
        digitalWrite(dirPins[0], LOW); // Set motor direction
      }
      if (motor1Speed > 0) {
        digitalWrite(dirPins[0], HIGH); // Set motor direction
      }
      analogWrite(pwmPins[0], abs(motor1Speed)); // Set motor speed
      
    int motor2Speed = motor2CommandRaw - 255;
      if (motor2Speed <= 0) {
        digitalWrite(dirPins[1], LOW); // Set motor direction
      }
      if (motor2Speed > 0) {
        digitalWrite(dirPins[1], HIGH); // Set motor direction
      }
      analogWrite(pwmPins[1], abs(motor2Speed)); // Set motor speed
  
    int motor3Speed = motor3CommandRaw - 255;
      if (motor3Speed <= 0) {
        digitalWrite(dirPins[2], LOW); // Set motor direction
      }
      if (motor3Speed > 0) {
        digitalWrite(dirPins[2], HIGH); // Set motor direction
      }
      analogWrite(pwmPins[2], abs(motor3Speed)); // Set motor speed
    }
  }
}
