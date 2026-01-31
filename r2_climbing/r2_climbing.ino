// ================= MOTOR PINS =================
int drive_omni[4][2] = {
  {0,1},{4,5},{14,15},{36,37}
};

int climber[2][2] = {
  {2,3},    // front lift
  {12,13}   // rear lift
};

int traction[2][2] = {
  {2,3},    // front lift
  {12,13}   // rear lift
};

// ================= ULTRASONIC =================
enum UltraID {
  US_F1, US_F2, US_FD, US_LF,
  US_LR, US_RD, US_R1, US_R2
};

float FL, FR, BL, BR;
int vx = 650;
int vy = 0;
int omega = 0;
int holding_pwm;

#define ULTRA_COUNT 8
#define ULTRA_SAMPLES 5
const int STAIR_HEIGHT_MM   = 200;
const int HEIGHT_TOLERANCE = 15;
const int EDGE_DELTA_MM    = 40;

const int DRIVE_SLOW = 500;
const int LIFT_SPEED = 2000;
const int TRACTION_SPEED = 10000;

const unsigned long STATE_TIMEOUT = 8000;

enum ClimbState {
  IDLE,
  APPROACH,
  FULL_LIFT,
  FRONT_OVERHANG,
  FRONT_DROP,
  MID_MOVE,
  REAR_DROP,
  DONE,
  ERROR_STATE
};

ClimbState climbState = IDLE;
unsigned long stateStart = 0;
int prevFrontDown = 0;

int trigPins[ULTRA_COUNT] = {22, 23, 24, 25, 26, 27, 28, 29};
int echoPins[ULTRA_COUNT] = {30, 31, 32, 33, 34, 35, 38, 39};

// ================= MOTOR CONTROL =================
void driveMotor(int pwmL, int pwmR, int pwm) {
  analogWrite(pwmL, pwm < 0 ? -pwm : 0);
  analogWrite(pwmR, pwm > 0 ?  pwm : 0);
}

void driveForward(int speed) {
  vx = speed;
  FL = vx - vy - omega;
  FR = vx + vy + omega;
  BL = vx + vy - omega;
  BR = vx - vy + omega;

  runMotor(drive_omni[0][0], drive_omni[0][1], FL);
  runMotor(drive_omni[1][0], drive_omni[1][1], FR);
  runMotor(drive_omni[2][0], drive_omni[2][1], BL);
  runMotor(drive_omni[3][0], drive_omni[3][1], BR);

}

void driveTraction(int speed){
  runMotor(traction[0][0], traction[0][1], spped);
  runMotor(traction[1][0], traction[1][1], speed);

}


void runMotor(int lpwm, int rpwm, int cmd){
  if(cmd > 0){
    analogWrite(lpwm, abs(cmd));
    analogWrite(rpwm, 0);
  }
  else if(cmd < 0){
    analogWrite(rpwm, abs(cmd));
    analogWrite(lpwm, 0);
  }
}

void stopDrive() { 
  driveForward(0); 
}

void liftFrontUp()   { driveMotor(climber[0][0], climber[0][1],  LIFT_SPEED); }
void liftFrontDown() { driveMotor(climber[0][0], climber[0][1], -LIFT_SPEED); }
void liftRearUp()    { driveMotor(climber[1][0], climber[1][1],  LIFT_SPEED); }
void liftRearDown()  { driveMotor(climber[1][0], climber[1][1], -LIFT_SPEED); }

void liftAllUp() {
  liftFrontUp();
  liftRearUp();
}

void stopLifts() {
  driveMotor(climber[0][0], climber[0][1], holding_pwm);
  driveMotor(climber[1][0], climber[1][1], holding_pwm);
}

void stopLifts2() {
  driveMotor(climber[0][0], climber[0][1], 0);
  driveMotor(climber[1][0], climber[1][1], holding_pwm);
}

void stopLifts3() {
  driveMotor(climber[0][0], climber[0][1], 0);
  driveMotor(climber[1][0], climber[1][1], 0);
}

void stopAll() {
  stopDrive();
  stopLifts3();
}

// ================= ULTRASONIC READING =================
int ultrasonicReadRaw(UltraID id) {
  digitalWrite(trigPins[id], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[id], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[id], LOW);

  long duration = pulseIn(echoPins[id], HIGH, 30000);
  if (duration == 0) return 9999;

  return duration / 58; // convert to mm
}

// Median filter for stable readings
int readUltrasonic(UltraID id) {
  int readings[ULTRA_SAMPLES];
  
  for(int i=0; i<ULTRA_SAMPLES; i++) {
    readings[i] = ultrasonicReadRaw(id);
    delay(10);
  }
  
  // Simple bubble sort for median
  for(int i=0; i<ULTRA_SAMPLES-1; i++) {
    for(int j=0; j<ULTRA_SAMPLES-i-1; j++) {
      if(readings[j] > readings[j+1]) {
        int temp = readings[j];
        readings[j] = readings[j+1];
        readings[j+1] = temp;
      }
    }
  }
  
  return readings[ULTRA_SAMPLES/2];
}

// ================= SENSOR LOGIC =================
bool stairDetectedFront() {
  int f1 = ultrasonicReadRaw(US_F1);
  int f2 = ultrasonicReadRaw(US_F2);
  return (f1 < 20 && f1 > 10) && (f2 < 20 && f2 > 10);
}

bool frontEdgeDetected() {
  int curr = ultrasonicReadRaw(US_FD);
  bool edge = (curr - prevFrontDown) > EDGE_DELTA_MM;
  prevFrontDown = curr;
  return edge;
}

bool frontLiftAtStep() {
  int dist = ultrasonicReadRaw(US_LF);
  return abs(dist - STAIR_HEIGHT_MM) < HEIGHT_TOLERANCE;
}

bool rearLiftAtStep() {
  int dist = ultrasonicReadRaw(US_LR);
  return abs(dist - STAIR_HEIGHT_MM) < HEIGHT_TOLERANCE;
}

bool robotLeveled() {
  int frontDist = ultrasonicReadRaw(US_LF);
  int rearDist = ultrasonicReadRaw(US_LR);
  return abs(frontDist - rearDist) < HEIGHT_TOLERANCE;
}

bool rearClearanceOK() {
  return ultrasonicReadRaw(US_RD) > (STAIR_HEIGHT_MM + 50);
}

// ================= BUTTON DEBOUNCE =================
// bool buttonPressed(int pin) {
//   static unsigned long lastPress[3] = {0, 0, 0};
//   static int pinMap[3] = {START_BUTTON_PIN, RESET_BUTTON_PIN, MANUAL_MODE_PIN};
  
//   int index = -1;
//   for(int i=0; i<3; i++) {
//     if(pinMap[i] == pin) {
//       index = i;
//       break;
//     }
//   }
  
//   if(index == -1) return false;
  
//   if(digitalRead(pin) == LOW) {  // Assuming pull-up, button pressed = LOW
//     if(millis() - lastPress[index] > 300) {
//       lastPress[index] = millis();
//       return true;
//     }
//   }
//   return false;
// }

// ================= STATE MACHINE =================
void updateClimb() {
  // Timeout check per state
  if (millis() - stateStart > STATE_TIMEOUT && climbState != IDLE && climbState != DONE) {
    Serial.println("ERROR: State timeout!");
    climbState = ERROR_STATE;
  }

  switch(climbState) {

    case APPROACH:
      driveForward(vx);
      if (stairDetectedFront()) {
        stopDrive();
        delay(100);
        climbState = FULL_LIFT;
        // stateStart = millis();
        Serial.println("State: FULL_LIFT");
      }
      break;

    case FULL_LIFT:
      liftAllUp();
      if (rearClearanceOK()) {
        stopLifts();
        delay(200);
        prevFrontDown = readUltrasonic(US_FD);
        climbState = FRONT_OVERHANG;
        stateStart = millis();
        Serial.println("State: FRONT_OVERHANG");
      }
      break;

    case FRONT_OVERHANG:
      driveTraction(TRACTION_SPEED);// traction
      if (frontEdgeDetected()) {
        // stop after some time
        stopDrive();
        delay(100);
        climbState = FRONT_DROP;
        stateStart = millis();
        Serial.println("State: FRONT_DROP");
      }
      break;

    case FRONT_DROP:
      liftFrontDown();
      if (frontLiftAtStep()) {
        stopLifts2();
        delay(100);
        climbState = MID_MOVE;
        stateStart = millis();
        Serial.println("State: MID_MOVE");
      }
      break;

    case MID_MOVE:
      // driveForward(DRIVE_SLOW);
      driveTraction(TRACTION_SPEED);  traction back
      if (rearLiftAtStep()) {
        // stop after some time
        stopDrive();
        delay(100);
        climbState = REAR_DROP;
        stateStart = millis();
        Serial.println("State: REAR_DROP");
      }
      break;

    case REAR_DROP:
      liftRearDown();
      if (robotLeveled()) {
        stopLifts();
        climbState = DONE;
        Serial.println("State: DONE - Climb complete!");
      }
      break;

    case DONE:
      stopAll();
      break;

    case ERROR_STATE:
      stopAll();
      Serial.println("ERROR STATE - Press reset button");
      break;

    default:
      break;
  }
}

// ================= STATUS LED (Optional) =================
const int STATUS_LED_PIN = 13;  // Built-in LED or external

void updateStatusLED() {
  switch(climbState) {
    case IDLE:
      digitalWrite(STATUS_LED_PIN, LOW);  // OFF
      break;
    case DONE:
      digitalWrite(STATUS_LED_PIN, HIGH);  // Solid ON
      break;
    case ERROR_STATE:
      // Blink fast
      digitalWrite(STATUS_LED_PIN, (millis() / 200) % 2);
      break;
    default:
      // Blink slow during operation
      digitalWrite(STATUS_LED_PIN, (millis() / 500) % 2);
      break;
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial.println("Stair Climbing Robot Starting...");
  Serial.println("=================================");
  
  analogWriteResolution(14);

  // Initialize motor pins
  for(int i=0; i<4; i++) {
    pinMode(drive_omni[i][0], OUTPUT);
    pinMode(drive_omni[i][1], OUTPUT);
  }
  
  for(int i=0; i<2; i++) {
    pinMode(climber[i][0], OUTPUT);
    pinMode(climber[i][1], OUTPUT);
  }

  // Initialize ultrasonic pins
  for(int i=0; i<ULTRA_COUNT; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
  
  // Initialize button pins (with pull-up)
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MANUAL_MODE_PIN, INPUT_PULLUP);
  
  // Initialize status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  Serial.println("Pin Configuration:");
  Serial.println("  Start Button: Pin 40");
  Serial.println("  Reset Button: Pin 41");
  Serial.println("  Manual Mode: Pin 42");
  Serial.println("=================================");
  Serial.println("Initialization complete.");
  Serial.println("Press START button to begin climb sequence");
}

// ================= MAIN LOOP =================
void loop() {
  // Handle reset button
  // if (buttonPressed(RESET_BUTTON_PIN) && (climbState == ERROR_STATE || climbState == DONE)) {
  //   climbState = IDLE;
  //   Serial.println("=================================");
  //   Serial.println("System Reset to IDLE");
  //   Serial.println("Press START to begin new sequence");
  //   Serial.println("=================================");
  // }

  // // Handle start button
  // if (buttonPressed(START_BUTTON_PIN) && climbState == IDLE) {
  //   climbState = APPROACH;
  //   stateStart = millis();
  //   Serial.println("=================================");
  //   Serial.println("CLIMB SEQUENCE STARTED");
  //   Serial.println("State: APPROACH");
  //   Serial.println("=================================");
  // }

  // Execute climb sequence
  if (climbState != IDLE) {
    updateClimb();
  }
  
  // Update status LED
  updateStatusLED();
  
  // Print status every 2 seconds when in IDLE
  static unsigned long lastStatusPrint = 0;
  if(climbState == IDLE && millis() - lastStatusPrint > 2000) {
    Serial.println("Status: IDLE - Waiting for START button");
    lastStatusPrint = millis();
  }
  
  delay(20);
}