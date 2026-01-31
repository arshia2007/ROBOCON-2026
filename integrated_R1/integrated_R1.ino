#include <Servo.h>
#include <Wire.h>
#include <AS5600.h>
#include <Encoder.h>
#include <IntervalTimer.h> 
#include "USBHost_t36.h"

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick1(myusb);
// BluetoothController bluet(myusb, true, "0000");     // Version does connecting to device
BluetoothController bluet(myusb);   // Version does pairing to device

uint32_t buttons;
uint32_t prevButtons;

// ============= drive ================= 
Encoder myEnc[3] = {Encoder(21, 20), Encoder(41, 40), Encoder(26, 27)};

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x = 0, y = 0, leftX = 0;  
int LPWM[3] = {0, 12, 4};   
int RPWM[3] = {1, 13, 5};

long currentCounts[3] = {0,0,0};
volatile long lastCount[3] = {0,0,0};      
volatile double rpm[3] = {0,0,0};          // Stores the calculated RPM
long positionChange[3] = {0,0,0};

//pid constants
float kp[3] = {9.0, 9.0, 9.0};
float ki[3] = {165.0, 165.0, 165.0};
float kd[3] = {0.5, 0.5, 0.5}; 

volatile float sp[3]={0,0,0};
float pid[3] = {0.0, 0.0, 0.0};
float err[3] = {0.0, 0.0, 0.0};
float prev_err[3] = {0.0, 0.0, 0.0};
float integ[3] = {0.0, 0.0, 0.0};
float der[3] = {0.0, 0.0, 0.0}; 

float max_rpm = 200;
IntervalTimer driveTimer; 

// ============= staff picking ================= 
int servo3_pin = 7;
int piston = 8;

Servo staff_servo;

// ============= kfs picking ================= 
int dc_lpwm = 1;    // power window motor
int dc_rpwm = 2;
int servo1_pin = 3;
int servo2_pin = 4;

bool flag1 = false;   // for sevo1
bool flag2 = false;   // for servo2
bool flag3 = false;   // for conveyor
bool flag4 = false;   // for staff - piston
bool flag5 = false;   // for staff - servo

Servo servo1;     // gripping
Servo servo2;

// Using standard Wire (Pin 18 SDA, Pin 19 SCL)
AS5600 as5600(&Wire); 

//pos pid constants
float kfs_kp = 2.0;   
float kfs_ki = 0.0;   
float kfs_kd = 0.0;   

double prevError = 0;         
int kfs_sp = 0;
float kfs_integ=0.0;
float kfs_der=0.0;
float kfs_pid = 0.0;

long rotations = 0;
int prevRawAngle = 0;

IntervalTimer timer;

// ============= conveyor ================= 
int conveyr_lpwm = 5;
int conveyr_rpwm = 6;


void setup() {
  Serial.begin(9600);

  // led on
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // 1. Initialize Standard Wire (Pins 18/19)
  Wire.begin(); 
  
  // 2. Initialize AS5600
  as5600.begin(); 

  // Check connection
  if (!as5600.isConnected()) {
    Serial.println("WARNING: AS5600 not found on Pins 18/19!");
  } else {
    prevRawAngle = as5600.readAngle();
  }

  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  staff_servo.attach(servo3_pin);

  servo1.write(90);
  servo2.write(0);  
  staff_servo.write(0);

  pinMode(piston, OUTPUT);
  digitalWrite(piston, LOW);  

  // Motor control pins setup
  pinMode(dc_lpwm, OUTPUT);
  pinMode(dc_rpwm, OUTPUT);

  // Initialize motors to stop
  analogWrite(dc_lpwm, 0);
  analogWrite(dc_rpwm, 0);

  for (int i = 0; i < 3; i++) {
    pinMode(RPWM[i], OUTPUT);
    pinMode(LPWM[i], OUTPUT);
  }

  for (int i = 0; i < 3; i++) {
    analogWrite(RPWM[i], 0);
    analogWrite(LPWM[i], 0);
  }

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport) Serial.print(CrashReport);
  myusb.begin();
  myusb.Task();

  timer.begin(calcPID, 75000);
  driveTimer.begin(calculateDrivePID, 75000);
  timer.priority(0);
  driveTimer.priority(1);

}

void loop() {
  myusb.Task(); 
  prevButtons = buttons;
  buttons = joystick1.getButtons();

  // =============== kfs picking =================== 
  if ((buttons == 1) && (prevButtons!= 1)){    //sq - servo 1 - gripping
    
    if(!flag1){   // grab kfs
      for(int i=90; i>=40; i--){
        servo1.write(i);
        delay(100);
      }
      flag1 = true;
    }else{      // release kfs
      for(int i=40; i<=90; i++){
        servo1.write(i);
        delay(100);
      }
      flag1 = false;
    }

  }else if ((buttons == 8) && (prevButtons!= 8)){  //tri - servo 2

    if(flag2){   // extend
      for(int i=180; i>=0; i--){
        servo2.write(i);
        delay(10);
      }
      flag2 = false;
    }else{     // retract
      for(int i=0; i<= 180; i++){
        servo2.write(i);
        delay(10);
      }
      flag2 = true;
    }

  } 

  // =============== kfs placing =================== 
  else if ((buttons == 4) && (prevButtons!= 4)){  //circle - conveyor - up

    if(flag3){
      analogWrite(conveyr_rpwm, 0);
      analogWrite(conveyr_lpwm, 10000);
      flag3 = false;
    }else{
      analogWrite(conveyr_lpwm, 0);
      analogWrite(conveyr_rpwm, 0);
      flag3 = true;
    }

  }else if ((buttons == 16) && (prevButtons!= 16)){  //circle - conveyor - down

    if(flag3){
      analogWrite(conveyr_lpwm, 0);
      analogWrite(conveyr_rpwm, 10000);
      flag3 = false;
    }else{
      analogWrite(conveyr_lpwm, 0);
      analogWrite(conveyr_rpwm, 0);
      flag3 = true;
    }

  }

  // =============== staff picking =================== 
  else if ((buttons == 2) && (prevButtons!= 2)){  // piston - open/close

    if(flag4){   // extend
      digitalWrite(piston, HIGH);
      flag4 = false;
    }else{     // retract
      digitalWrite(piston, LOW);
      flag4 = true;
    }

  }else if ((buttons == 32) && (prevButtons!= 32)){  // servo control

    if(flag5){   
      for(int i=90; i>=0; i--){
        staff_servo.write(i);
        delay(10);
      }
      flag5 = false;
    }else{     
      for(int i=0; i<= 90; i++){
        staff_servo.write(i);
        delay(10);
      }
      flag5 = true;
    }

  } 

}

void input() {
  if (Serial.available() > 0) {       
    String input = Serial.readString();
    kfs_sp = input.toInt() / 360.0 * 4096;   // deg -> ticks
  }
  // Serial.printf(" kfs_sp:%f", kfs_sp);
}

void calcPID() {

  input();

  // Read encoder with wrap-around handling
  // int raw = as5600.readAngle();
  
  // // Handle wrap-around
  // if ((raw - prevRawAngle) < -2048) rotations++;
  // else if ((raw - prevRawAngle) > 2048) rotations--;
  
  // long currentCounts = (rotations * 4096L) + raw;
  // prevRawAngle = raw;
 
  // long currentCounts = as5600.readAngle();
  long currentCounts = as5600.getCumulativePosition();

  // PID Control 
  float err = kfs_sp - currentCounts;
  kfs_integ = kfs_integ + (err*0.075);
  kfs_der = (err - prevError)/0.075;

  kfs_pid = (kfs_kp*err) + (kfs_ki*kfs_integ) + (kfs_kd*kfs_der);
  prevError = err;

  runMotor(dc_rpwm, dc_lpwm, kfs_pid);

}

void runMotor(int RPWM, int LPWM, float speed) {
  int pwm = abs(speed);
  pwm = constrain(pwm, 0, 12000);

  if (speed > 0) {      // to check direction: if +ve - HIGH, else LOW
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  } else if (speed < 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}

void calculateDrivePID() {

  if (joystick1.available()) {

    // Left Stick values (axes 0 and 1)
    int leftStickX = joystick1.getAxis(0);
    leftX = map(leftStickX, 0, 255, -100, 100);

    // Right Stick values (axes 2 and 5)
    int rightStickX = joystick1.getAxis(2);
    x = map(rightStickX, 0, 255, -100, 100);

    int rightStickY = joystick1.getAxis(5);
    y = map(rightStickY, 0, 255, 100, -100);

    //to ignore small joystick values
    if (abs(x) < 5) x = 0;
    if (abs(y) < 5) y = 0;
    if (abs(leftX) < 5) leftX = 0;

    // Serial.printf(" x:%d\n",x);
    // Serial.printf(" y:%d",y);
    // Serial.printf(" left x:%d",leftX);
  }

  sp[0] = ((x) * (-0.67) + (y) * 0 + (leftX) * (-0.33));        
  sp[1] = ((x) * (0.33) + (y) * (-0.57) + (leftX) * (-0.33)); 
  sp[2] = ((x) * (0.33) + (y) * (0.57) + (leftX) * (-0.33)); 
  
  // Serial.printf(" sp1:%0.2f", sp[0]);
  // Serial.printf(" sp2:%0.2f", sp[1]);
  // Serial.printf(" sp3:%0.2f", sp[2]);

  sp[0] = map(sp[0], -72, 72, -max_rpm, max_rpm);
  sp[1] = map(sp[1], -72, 72, -max_rpm, max_rpm);
  sp[2] = map(sp[2], -72, 72, -max_rpm, max_rpm);

  // Calculate RPM
  for (int i=0; i<3; i++){
    currentCounts[i] = myEnc[i].read();
    positionChange[i] = currentCounts[i] - lastCount[i];
    rpm[i] = (positionChange[i] / 3300.0) * (60 * (1000.0 / 75));
    lastCount[i] = currentCounts[i];

  }
  // Serial.printf(" rpm1:%f", rpm[0]);
  // Serial.printf(" rpm2:%f", rpm[1]);
  // Serial.printf(" rpm3:%f\n", rpm[2]);

  //PID Control
  for (int i=0; i<3; i++){
    err[i] = sp[i] - rpm[i];
    integ[i] = integ[i] + (err[i]*0.075);   
    der[i] = (err[i]-prev_err[i])/0.075;

    pid[i] = (kp[i]*err[i]) + (ki[i]*integ[i]) + (kd[i]*der[i]);
    prev_err[i] = err[i];

    pid[i] = constrain(pid[i], -16383, 16383);

  }

  // Set motor speeds based on calculated velocities
  runDriveMotor(RPWM[0], LPWM[0], pid[0]);
  runDriveMotor(RPWM[1], LPWM[1], pid[1]);
  runDriveMotor(RPWM[2], LPWM[2], pid[2]);

}

void runDriveMotor(int RPWM, int LPWM, float speed) {
  int pwmValue = int(abs(speed));
  // int pwmValue = constrain(abs(speed),0,200);
  // int pwmValue = map(abs(speed), 0, 127, 0, 16383);

  if (speed > 0) {      // to check direction: if +ve - HIGH, else LOW
    // digitalWrite(EN, HIGH);

    analogWrite(RPWM, pwmValue);
    analogWrite(LPWM, 0);
  } else if (speed < 0) {
    // digitalWrite(EN, LOW);

    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwmValue);
  } else {
    // digitalWrite(EN, LOW);
    
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}
