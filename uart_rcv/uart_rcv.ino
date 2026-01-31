#include "USBHost_t36.h"
#include <Encoder.h>

// ================== GLOBALS: SENSORS ==================
const int NUM_SENSORS = 12;
int distances[NUM_SENSORS];

// Data logging variables
unsigned long log_interval = 100;  // Log every 100ms
unsigned long last_log_time = 0;
bool header_printed = false;

// ================== GLOBALS: CONTROLLER ==================
USBHost myusb;
JoystickController joystick1(myusb);
BluetoothController bluet(myusb);
// BluetoothController bluet(myusb, true, "0000");

uint32_t buttons_prev = 0;
uint32_t buttons;

int psAxis[64];
int psAxis_prev[64];
bool first_joystick_message = true;

bool up=false, down=false, lastup=false, lastdown=false;

//PID
Encoder myEnc[4] = {Encoder(21, 20), Encoder(26, 27), Encoder(40, 41), Encoder(32, 33)};

long currentCounts[4] = {0,0,0,0};
volatile long lastCount[4] = {0,0,0,0};      
volatile double rpm[4] = {0,0,0,0};          
long positionChange[4] = {0,0,0,0};

//pid constants
float kp[4] = {9.0, 9.0, 9.0, 9.0};
float ki[4] = {165.0, 165.0, 165.0, 165.0};
float kd[4] = {0.5, 0.5, 0.5, 0.5}; 

// volatile float sp[4]={0,0,0,0};
float pid[4] = {0.0, 0.0, 0.0, 0.0};
float err[4] = {0.0, 0.0, 0.0, 0.0};
float prev_err[4] = {0.0, 0.0, 0.0, 0.0};
float integ[4] = {0.0, 0.0, 0.0, 0.0};
float der[4] = {0.0, 0.0, 0.0, 0.0}; 
// float count[4] = {1300.0, 700.0, 1300.0, 1300.0};
float count[4] = {3500.0, 3500.0, 3600.0, 3500.0};



// ================== GLOBALS: MOTORS & PINS ==================
// m2,m4,m7,m5.....,pwmR,pwmL
int drive_omni[4][2] = {
  {4, 5},
  {12, 13},
  {22, 23},
  {0, 1}
};


int rpm_sp[4];
volatile int x = 0, y = 0, w = 0;

// int max_rpm = 32 * 255;
int max_rpm = 200;
int right_y;

IntervalTimer timer;

// ================== SETUP ==================
void setup() {
  Serial.begin(9600);     // USB Serial (For Python Logging)
  Serial8.begin(115200);  // Sensor Serial

  // Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  // if (CrashReport) Serial.print(CrashReport);
  
  myusb.begin();
  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  timer.begin(calcPID, 75000);
  
  delay(2000);  // Wait for connection
  
  // Print CSV header immediately
  Serial.println("Timestamp,US1,US2,US3,US4,US5,US6,US7,US8,US9,US10,US11,US12,PWM1,PWM2,PWM3,PWM4,RPM1,RPM2,RPM3,RPM4,PS4_X,PS4_Y,PS4_W");
  header_printed = true;
}

// ================== MAIN LOOP ==================
void loop() {
  // 1. Handle Controller & Drive Wheels
  // manual();
  ps4_input_IK();

  // 3. Read Sensors & Log to Python
  if (Serial8.available() > 0) {
    String receivedData = Serial8.readStringUntil('\n');
    receivedData.trim();
    
    if (receivedData.length() > 0) {
      parseSensors(receivedData);
      sendDataToPython();
    }
  }
}

// ================== LOGGING & PARSING ==================

void parseSensors(String data) {
  int sensorIndex = 0;
  int startIndex = 0;
  int commaIndex;

  while (sensorIndex < NUM_SENSORS) {
    commaIndex = data.indexOf(',', startIndex);
    String valueStr;

    if (commaIndex == -1) {
      valueStr = data.substring(startIndex);
    } else {
      valueStr = data.substring(startIndex, commaIndex);
    }

    valueStr.trim();
    if (valueStr.length() > 0) {
      distances[sensorIndex] = valueStr.toInt();
      sensorIndex++;
    }

    if (commaIndex == -1) break;
    startIndex = commaIndex + 1;
  }
}

void sendDataToPython() {
  // Only log at specified interval
  if (millis() - last_log_time < log_interval) {
    return;
  }
  last_log_time = millis();
  
  // Timestamp
  Serial.print(millis());
  Serial.print(",");
  
  // Ultrasonic sensors (12 values)
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(distances[i]);
    Serial.print(",");
  }
  
  // PWM values (4 motors)
  for (int i = 0; i < 4; i++) {
    Serial.print((int)pid[i]);
    Serial.print(",");
  }
  
  // Actual RPM (4 motors)
  for (int i = 0; i < 4; i++) {
    Serial.print(rpm[i], 2);
    Serial.print(",");
  }
  
  // PS4 Controller inputs (x, y, w)
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(w);
  
  Serial.println();
}

// ================== CONTROLLER LOGIC ==================

void ps4_input_IK() {
  myusb.Task();
  if (joystick1.available()) {

    for (uint8_t i = 0; i < 64; i++) {
      psAxis_prev[i] = psAxis[i];
      psAxis[i] = joystick1.getAxis(i);
    }

    buttons = joystick1.getButtons();

    y = psAxis[1] - 128;
    x = psAxis[0] - 128;
    w = psAxis[2] - 128;

    if(abs(x)<5) x = 0;
    if(abs(y)<5) y = 0;
    if(abs(w)<5) w = 0;

    // Serial.printf("x: %d",x);
    // Serial.printf("y: %d",y);
    // Serial.printf("w: %d\n",w);

    // // Calculate Omni Wheel Speeds
    // rpm_sp[0] = map(-x + y + w / 2, -255, 255, -max_rpm, max_rpm);
    // rpm_sp[1] = map(+x + y + w / 2, -255, 255, -max_rpm, max_rpm);
    // rpm_sp[2] = map(x - y + w / 2, -255, 255, -max_rpm, max_rpm);
    // rpm_sp[3] = map(-x - y + w / 2, -255, 255, -max_rpm, max_rpm);

    // Serial.printf("sp1:%.2f", rpm_sp[0]);
    // Serial.printf("  sp1:%.2f", rpm_sp[1]);
    // Serial.printf("  sp1:%.2f", rpm_sp[2]);
    // Serial.printf("  sp1:%.2f\n", rpm_sp[3]);

    right_y = psAxis[5] - 128;
    if (abs(right_y) < 5) right_y = 0;

    // Mode Selection
    // if (buttons & 8)      { isFront = true; isRear = false; isBoth = false; }
    // else if (buttons & 2) { isFront = false; isRear = true; isBoth = false; }
    // else if (buttons & 4) { isFront = false; isRear = false; isBoth = true; }
    // else if (buttons & 1) { isFront = false; isRear = false; isBoth = false; }

    // // ---- UP button (L1 = 16)
    // if ((buttons & 16) && !lastup) {
    //   up = true;
    //   lastup = true;
    // }
    // if (!(buttons & 16)) {
    //   lastup = false;
    // }

    // // ---- DOWN button (R1 = 32)
    // if ((buttons & 32) && !lastdown) {
    //   down = true;
    //   lastdown = true;
    // }
    // if (!(buttons & 32)) {
    //   lastdown = false;
    // }
  }
}

void drive(int pwmL, int pwmR, int pwm) {
  analogWrite(pwmL, pwm < 0 ? -1 * pwm : 0);
  analogWrite(pwmR, pwm > 0 ? pwm : 0);
}

// ================== PID & LIFTING ==================

void calcPID(){
  // ps4_input_IK();
  // // Calculate Omni Wheel Speeds
  rpm_sp[0] = map(-x + y + w / 2, -255, 255, -max_rpm, max_rpm);
  rpm_sp[1] = map(+x + y + w / 2, -255, 255, -max_rpm, max_rpm);
  rpm_sp[2] = map(x - y + w / 2, -255, 255, -max_rpm, max_rpm);
  rpm_sp[3] = map(-x - y + w / 2, -255, 255, -max_rpm, max_rpm);

  // Serial.printf("sp1:%.2f", rpm_sp[0]);
  // Serial.printf("  sp1:%.2f", rpm_sp[1]);
  // Serial.printf("  sp1:%.2f", rpm_sp[2]);
  // Serial.printf("  sp1:%.2f\n", rpm_sp[3]);

  // Calculate RPM
  for (int i=0; i<4; i++){
    currentCounts[i] = myEnc[i].read();
    positionChange[i] = currentCounts[i] - lastCount[i];
    rpm[i] = (positionChange[i] / count[i]) * (60 * (1000.0 / 75));
    lastCount[i] = currentCounts[i];
  }

  //PID Control
  for (int i=0; i<4; i++){
    err[i] = rpm_sp[i] - rpm[i];
    integ[i] = integ[i] + (err[i]*0.075);   
    der[i] = (err[i]-prev_err[i])/0.075;

    pid[i] = (kp[i]*err[i]) + (ki[i]*integ[i]) + (kd[i]*der[i]);
    prev_err[i] = err[i];

    pid[i] = constrain(pid[i], -16383, 16383);

  }

  // Drive Omni Wheels
  for (int i = 0; i < 4; i++){
  // Serial.println(pid[i]);
    drive(drive_omni[i][0], drive_omni[i][1], pid[i]);
}

}