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


// ============= kfs picking ================= 
int dc_lpwm = 1;    // power window motor
int dc_rpwm = 2;
int servo1_pin = 3;
int servo2_pin = 4;

bool flag1 = false;   // for sevo1
bool flag2 = false;   // for servo2
bool flag3 = false;   // for conveyor

Servo servo1;
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

  servo1.write(0);
  servo2.write(0);  

  // Motor control pins setup
  pinMode(dc_lpwm, OUTPUT);
  pinMode(dc_rpwm, OUTPUT);

  // Initialize motors to stop
  analogWrite(dc_lpwm, 0);
  analogWrite(dc_rpwm, 0);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport) Serial.print(CrashReport);
  myusb.begin();
  myusb.Task();

  timer.begin(calcPID, 75000);
}

void loop() {
  myusb.Task(); 
  prevButtons = buttons;
  buttons = joystick1.getButtons();

  // =============== kfs picking =================== 
  if ((buttons == 1) && (prevButtons!= 1)){    //sq - servo 1 
    
    if(!flag1){   // grab kfs
      for(int i=135; i>=90; i--){
        servo1.write(i);
        delay(10);
      }
      flag1 = true;
    }else{      // release kfs
      for(int i=90; i<= 135; i++){
        servo1.write(i);
        delay(10);
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
  int raw = as5600.readAngle();
  
  // Handle wrap-around
  if ((raw - prevRawAngle) < -2048) rotations++;
  else if ((raw - prevRawAngle) > 2048) rotations--;
  
  long currentCounts = (rotations * 4096L) + raw;
  prevRawAngle = raw;
 
  // long currentCounts = as5600.readAngle();

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