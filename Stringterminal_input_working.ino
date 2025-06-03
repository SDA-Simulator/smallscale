/*
   ──────────────────────────────────────────────────────────────
   SIMPLE MASS-BALANCER CONTROLLER (PER-MOTOR CALIBRATION)
   - 3 stepper motors, one moves at a time
   - Command format:  B<aaa><bbb><ccc>
   - stepDelayScale slows the micro-step rate (1.0 = original speed)
   - CAL1/CAL2/CAL3 scale each motor’s travel separately
   ─────────────────────────────────────────────────────────────
*/

// ─── Pin map ───────────────────────────────────────────────────
const int speed1     = 3,  runstop1   = 29, dir1 = 28;
const int speed2     = 9,  runstop2   = 31, dir2 = 30;
const int speed3     = 11, runstop3   = 33, dir3 = 32;

// ─── Motion constants ──────────────────────────────────────────
const float diameter       = 0.0164592;                  // m
const float distanceStep   = (PI * diameter) / 3200.0;   // m per micro-step
const float timeStep       = 3.32;                       // ms per micro-step
const float stepDelayScale = 30.0;                       // ↑↑ slows pulses

// ─── Per-motor calibration ────────────────────────────────────
const float CAL1 = 0.86;    // Motor 1 moves at  90% of commanded
const float CAL2 = 0.85;    // Motor 2 moves at  85% of commanded
const float CAL3 = 0.86;    // Motor 3 moves at  95% of commanded

// ─── Position trackers ────────────────────────────────────────
float pos1 = 0.0, pos2 = 0.0, pos3 = 0.0;

// ─── Emergency stop ──────────────────────────────────────────
void disableAll() {
  digitalWrite(runstop1, LOW);
  digitalWrite(runstop2, LOW);
  digitalWrite(runstop3, LOW);
}

// ─── Stepping routine ─────────────────────────────────────────
// Now takes an extra 'calib' argument
void driveStepper(int speedPin, int runPin, int dirPin, float delta, float calib) {
  if (delta == 0.0) return;

  // apply the per-motor calibration
  delta *= calib;

  // compute number of micro-steps
  int steps = int(fabs(delta) / distanceStep + 0.5);

  // set direction
  digitalWrite(dirPin, delta >= 0 ? HIGH : LOW);

  // hold a minimal PWM to energize coils
  analogWrite(speedPin, 1);

  // pulse the runPin at slowed rate
  unsigned int stepDelayUs = (unsigned int)(timeStep * stepDelayScale * 1000.0);
  for (int i = 0; i < steps; ++i) {
    digitalWrite(runPin, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(runPin, LOW);
    delayMicroseconds(stepDelayUs);
  }

  // turn coils off
  analogWrite(speedPin, 0);
}

// ─── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // configure pins
  pinMode(speed1, OUTPUT);  pinMode(runstop1, OUTPUT);  pinMode(dir1, OUTPUT);
  pinMode(speed2, OUTPUT);  pinMode(runstop2, OUTPUT);  pinMode(dir2, OUTPUT);
  pinMode(speed3, OUTPUT);  pinMode(runstop3, OUTPUT);  pinMode(dir3, OUTPUT);

  disableAll();
}

// ─── Main loop ────────────────────────────────────────────────
void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  if (cmd.length() < 10 || cmd[0] != 'B') {
    Serial.println("Bad command!");
    return;
  }

  // parse targets (cm×10 → m)
  float tgt1 = (cmd.substring(1, 4).toFloat() - 80) / 1000.0;
  float tgt2 = (cmd.substring(4, 7).toFloat() - 80) / 1000.0;
  float tgt3 = (cmd.substring(7,10).toFloat() - 80) / 1000.0;

  // move motor 1
  disableAll();
  driveStepper(speed1, runstop1, dir1, tgt1 - pos1, CAL1);
  pos1 = tgt1;

  // move motor 2
  disableAll();
  driveStepper(speed2, runstop2, dir2, tgt2 - pos2, CAL2);
  pos2 = tgt2;

  // move motor 3
  disableAll();
  driveStepper(speed3, runstop3, dir3, tgt3 - pos3, CAL3);
  pos3 = tgt3;

  disableAll();

  // report positions
  Serial.print("Done → ");
  Serial.print(pos1,3); Serial.print(" m, ");
  Serial.print(pos2,3); Serial.print(" m, ");
  Serial.print(pos3,3); Serial.println(" m");
}
