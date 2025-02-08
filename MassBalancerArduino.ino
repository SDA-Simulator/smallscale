/* Mass Balancing Arduino
 *  Spencer Bullen, Rob Bergbaum, Juan Pelaez
 *  
 *  Notes: In running the motor for like 30 minutes and testing, the motor got unbelieavably hot. 
 *  So, I would recommend not having a long test where the motor is constantly running lol. My b.
 *  Monitor by going to Tools > Serial Monitor
 *  
 *  Some explantions:
 *    For direction, HIGH = CCW, LOW = CW
 *    For enable, HIGH = stopping the motor.
*/

// Pin Connections
int speed = 5;          // Speed = Purple Wire. Motor direction connected to digital pin 5 (PWM)
int direction = 24;     // Direction = Green Wire. Motor speed connected to digital pin 24.
int runstop = 28;       // On/Off = White Wire. Run/Stop connected to digital pin 28.
int enable = 32;        // Enable = Blue Wire. Enable connected to digital pin 32.
// int Vplus = XX ;     // Voltage = Red Wire. Connected to power supply, will be battery in future.
// int ground = XX;     // Ground = Black Wire. Connected to power supply.
// int hlspeed = XX;    // H/L = Yellow Wire. Unused.   
// int vr = XX;         // Vr = Orange Wire. Unused, even in the future.

// ISC02-04 Stepper Motor Constants Given from Spec Sheet
const int stepsPerRev = 200; const int microstepsPerRev = 3200;   // Stepper has 200 steps/rev (360 deg/1.8 deg step) and 3200 microsteps/rev 
const int commandedRevs = 3;                                      // Change to your desired number of revs.
const int commandedSteps = commandedRevs*microstepsPerRev;        // Microsteps more accurate, could do steps per rev as well
const int stepDelay = 500;                                        // Short pause needed for discrete stepping

// Setting Initial Conditions for Connected Pins
void setup()
{
  Serial.begin(9600); delay(2000);                                // Initialization
  Serial.println("Initial Comm: You're so gorgeous Spencer");
  pinMode(speed, OUTPUT); pinMode(direction, OUTPUT); pinMode(runstop, OUTPUT); pinMode(enable, OUTPUT); 

  // At this point, should be seeing regular CW rotation at set speed (just giving it a voltage does this)
  digitalWrite(enable, HIGH); delay(5000);
  
  // Now, to test working, I switch directions here.
  digitalWrite(direction, HIGH);
  Serial.println("CCW 1 revolution expected.");
  moveSteps(commandedSteps);  // Move the commanded revolutions

  // Switch directions again, CW again now.
  digitalWrite(direction, LOW);     // sets the default direction to be counterclockwise. CCW = HIGH, CW = LOW
  Serial.println("CW 1 revolution expected.");
  moveSteps(commandedSteps);  // Move half the commanded revolutions  
  Serial.println("Done running setup test. Loop function begins now.");

// THIS SNIPPET WAS WORKING BEFORE STEP CONTROL
/*
  digitalWrite(enable, HIGH); delay(5000); digitalWrite(enable, LOW);
  
  digitalWrite(direction, HIGH);                     // CCW = HIGH, CW = LOW
  analogWrite(speed, 200);
  Serial.println("CCW Fast Rotations"); delay(5000); // Run for 5 seconds
  
  analogWrite(speed, 10);
  Serial.println("CCW Slow Rotations"); delay(3000);

  Serial.println("Switching direction.");
  digitalWrite(direction, LOW);  
  analogWrite(speed, 10);                           
  Serial.println("CW Slow Rotations"); delay(5000); // Run for 5 seconds

  analogWrite(speed, 200);                           
  Serial.println("CW Fast Rotations"); delay(5000); // Run for 5 seconds
  Serial.println("Done running setup test. Loop function begins now.");
*/
}

// moveSteps allows for discrete stepping. speed set to HIGH lets it step once, short delay for the driver, then LOW ends the step.
// Loop this over how many steps commanded, since this is discrete steps. To make rotate faster, remove the stepDelay/recrease it.
void moveSteps(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(speed, HIGH); delayMicroseconds(stepDelay); 
    digitalWrite(speed, LOW); delayMicroseconds(stepDelay);
  }
}

// For now, just have loop stopping it. Will have loop with feedback.
void loop()
{
  //digitalWrite(runstop, HIGH);
  digitalWrite(enable, HIGH); // ENABLE ON HIGH = STOP MOVING. This will be an if statement. if(error < tolerance (defined)) then digitalWrite(enable, HIGH)
}
