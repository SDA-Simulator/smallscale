/* Mass Balancing Arduino
 *  Rob Bergbaum, Spencer Bullen, Juan Pelaez
 *  The purpose of this code is to be able to move stepper motors according to a revolution distance as opposed to a given voltage. 
 *  
 *  Notes: In running the motor for like 30 minutes and testing, the motor got unbelieavably hot. 
 *  So, I would recommend not having a long test where the motor is constantly running lol. My b.
*/

// Pin Connections
int speed = 5;          // Speed = Purple Wire. Motor direction connected to digital pin 5 (PWM)
int direction = 24;     // Direction = Green Wire. Motor speed connected to digital pin 24.
int runstop = 28;       // On/Off = White Wire.

/*
// int Vplus = XX ;     // Voltage = Red Wire. Connected to power supply, will be battery in future.
// int ground = XX;     // Ground = Black Wire. Connected to power supply.
// int enable = XX;     // Enable = Blue Wire. Unused.
// int hlspeed = XX;    // H/L = Yellow Wire. Unused.   
// int vr = XX;         // Vr = Orange Wire. Unused, even in the future.
*/

// ISC02-04 Stepper Motor Constants
const int stepsPerRev = 200;  // Stepper has 200 steps/rev (360 deg/1.8 deg step)
const int microstepsPerRev = 3200; // Stepper has 3200 microsteps/rev (given in spec sheet)
const float maxRPM = 1256; // Maximum speed range (given in spec sheet)
const float minRPM = 0.75; // Minimum speed range (given in spec sheet)
const float testRPM = 60;   // This we do not know how fast it actually is rotating. Changes with voltage?

const int commandedRevs = 3;  // Change to your desired number of revs.
const int commandedSteps = commandedRevs*microstepsPerRev; // Microsteps more accurate, could do steps per rev as well
const int stepDelay = 500;
const float secondsOneRev = 60/testRPM;  // Time for one revolution
const float totalRunTime = secondsOneRev*commandedRevs*1000; // Total time for commanded revs. *1000 for millisecond.

// Setting Initial Conditions for Connected Pins
void setup()
{
  Serial.begin(9600);
  delay(2000); // Wait for serial to initialize
  Serial.println("Initial Comm: You're so gorgeous Spencer");

  // Set pins as outputs
  pinMode(speed, OUTPUT);      
  pinMode(direction, OUTPUT);    
  pinMode(runstop, OUTPUT);       

  // pinMode(Vplus, OUTPUT); pinMode(ground, OUTPUT);
  // pinMode(enable, OUTPUT);
  // pinMode(hlspeed, OUTPUT); pinMode(vr, OUTPUT);

  // At this point, should be seeing regular CW rotation at set speed (just giving it a voltage does this)
  delay(5000);
  // Now, to test working, I switch directions here.
  digitalWrite(direction, HIGH);     // sets the default direction to be counterclockwise. CCW = HIGH, CW = LOW
  Serial.println("CCW 1 revolution expected. Not sure how fast.");

  // Test moving to steps instead of set speeds.
  moveSteps(commandedSteps);  // Move half the commanded revolutions
  // delay(5000); // Short pause before switching speed

  // Switch directions again, CW again now.
  digitalWrite(direction, LOW);     // sets the default direction to be counterclockwise. CCW = HIGH, CW = LOW
  Serial.println("CW 1 revolution expected. Not sure how fast.");
  moveSteps(commandedSteps);  // Move half the commanded revolutions  
  // delay(1000); // Short pause 
  // digitalWrite(speed, 0);
  Serial.println("Done running setup test.");
  // digitalWrite(runstop, HIGH); // HIGH RUNSTOP should ground it and not move at all.

/* THIS SNIPPET WAS WORKING BEFORE STEP CONTROL
  // Ok, when first booted up, it will be running CW by default (low). When upload successful, runs CW for 5 secs, then begins my code.
  delay(5000);
  digitalWrite(direction, HIGH);     // sets the default direction to be counterclockwise. CCW = HIGH, CW = LOW
  Serial.println("Should be moving CCW quickly");
  analogWrite(speed, 250);          // Sets the default speed to be 25% of max.
  delay(5000);                      // Run for 5 seconds
  Serial.println("Should be moving CCW slowly");
  analogWrite(speed, 10);
  delay(3000);
  //analogWrite(speed, 0);              // Stop motor
  //delay(1000);                       // Short pause before switching direction
/*  
  Serial.println("Switching direction...");
  digitalWrite(direction, HIGH);  // Set CCW direction
  Serial.println("Running Counterclockwise for 5 seconds...");
  analogWrite(speed, 80);        // Same speed as before
  delay(5000);                      // Run for 5 seconds
  analogWrite(speed, 0);         // Stop motor
  Serial.println("Motor stopped.");

  // The motor is not stopping..... It can switch from CW to CCW, then back to CW.
  
  // digitalWrite(Vplus, LOW); digitalWrite(ground, LOW);
  // digitalWrite(enable, LOW); digitalWrite(runstop, LOW);
  // digitalWrite(hlspeed, LOW); // sets the default speed to be off
  // digitalWrite(vr, LOW);
*/
}

void moveSteps(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(speed, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(speed, LOW);
    delayMicroseconds(stepDelay);
  }
}

void loop()
{
  digitalWrite(runstop, HIGH);
  /*
  // Setup code should have the motor moving at 10% max speed in CCW direction.
  digitalWrite(speed, 0);
  delay(5000);
  digitalWrite(direction, LOW);     // sets the default direction to be counterclockwise. CCW = HIGH, CW = LOW
  Serial.println("Should be moving CW quickly");
  analogWrite(speed, 250);          // Sets the default speed to be 25% of max.
  delay(5000);                      // Run for 5 seconds
  Serial.println("Should be moving CW slowly");
  analogWrite(speed, 20);
  delay(3000);
  */
}
