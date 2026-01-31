#include "USBHost_t36.h"
#include <Encoder.h>
USBHost myusb;
JoystickController joystick1(myusb);
// BluetoothController bluet(myusb, true, "0000");
BluetoothController bluet(myusb);
int max_rpm=8000;
uint32_t buttons_prev = 0;
uint32_t buttons;

int psAxis[64];
int psAxis_prev[64];
bool first_joystick_message = true;

Encoder myEncfront(27,26);
Encoder myEncback(21,20);

float spf = 0, spb = 0;
float pidf = 0, pidb = 0;
bool both = true; // or false, depending on logic
int target_f=0,target_b=0;
bool f=false,b=false,bo=false;
int pprf=3400,pprb=3400;
const int NUM_SENSORS = 2;
int distances[NUM_SENSORS];
int trig[2]={30,28},echo[2]={31,29};
int front=0,back=1;
float target_dist=0;

float filt_f = 0;
float filt_b = 0;
int h;
float dist_tickf=67.9,dist_tickb=104.9;
float err_tickf=0,err_tickb=0;

float alpha = 0.4; // light filtering, good for ultrasonic

int drive_trx[2][2]={
  {12,13},
  {22,23}
  };//front,back m3,m8

int climber[2][2]={
  {4,5},
  {0,1}
  };

int drive_omni[4][2] = {
  {4, 5},
  {12, 13},
  {22, 23},
  {0, 1}
};

static inline void enableCycleCounter() {
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}
float kpf = 20, kpb = 20; // height PI
float prev_f=0,prev_b=0;
float prev_tickf=0,prev_tickb=0;
unsigned long lastTime = 0;
int sp=0;
void runMotor(int pwmPinF, int pwmPinR, float speed) {
  // using two PWM pins: pwmPinF (forward), pwmPinR (reverse)
  int pwm = (int)abs(speed);
  pwm = constrain(pwm, 0, max_rpm); // for 14-bit resolution

  if (speed > 0.0) {
    // forward: activate forward PWM, ensure reverse PWM is 0
    analogWrite(pwmPinF, pwm);
    analogWrite(pwmPinR, 0);
  } else if (speed < 0.0) {
    // reverse: activate reverse PWM, ensure forward PWM is 0
    analogWrite(pwmPinF, 0);
    analogWrite(pwmPinR, pwm);
  } else {
    // stop: both low
    analogWrite(pwmPinF, 0);
    analogWrite(pwmPinR, 0);
  }
}
int limit[2]={8,9};//////pinnumber;
bool l=false;
void limitswitch(){
  Serial.println(digitalRead(9));
  if(digitalRead(limit[0])==0){//||distances[back]>40||distances[front]>40) {
    l=true;

  }
  if(digitalRead(9)==0){
    l=true;
  }
  
}

int right_y;
void ps4_input_IK() {
  myusb.Task();
  if (joystick1.available()) {
    for (uint8_t i = 0; i < 64; i++) {
      psAxis_prev[i] = psAxis[i]; 
      psAxis[i] = joystick1.getAxis(i);
    }
    buttons = joystick1.getButtons();
    
    right_y = psAxis[0] - 128; //trx
    right_x = psAxis[1] - 128; //drive

    Serial.println(right_y);
    if(abs(right_y) < 5) right_y = 0;
    if(abs(right_x) < 5) right_x = 0;

    if(buttons & 8) { Serial.println("f");f=true; b=false; bo=false; }
    else if(buttons & 2) { Serial.println("b");f=false; b=true; bo=false; }
    else if(buttons & 4) { Serial.println("bo");f=false; b=false; bo=true; }
    else if(buttons & 1) { Serial.println("no");f=false; b=false; bo=false; }


    if(buttons&16){
      max_rpm=8000;
      if(bo){
        target_f=1.5;
        target_b=1.5;
        
      }
      else if(f){
        target_f=1.5;        
      }
      else if(b){
        target_b=1.5;
        
      }
      else{
        target_f = 0;
        target_b=0;
      }
    }
    else if(buttons&32){
      max_rpm=3000;
      if(bo){
        target_f=-0.3;
        target_b=-0.3;
        
      }
      else if(f){
        target_f=-0.3;        
      }
      else if(b){
        target_b=-0.3;
      }
    }
  } 
}

IntervalTimer timererror;

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport) Serial.print(CrashReport);
  myusb.begin();
    
  Serial.println(l);
  delay(1000);   
  for(int i=0;i<2;i++){
    pinMode(limit[i], INPUT_PULLUP);
    pinMode(trig[i],OUTPUT);
    pinMode(echo[i],INPUT);
    digitalWrite(trig[i], LOW);

  }

  
  enableCycleCounter();

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);
}
uint32_t pulseInFast(uint8_t pin, bool state, uint32_t timeout_us)
{
  const uint32_t timeout_cycles = timeout_us * 600; // 600 MHz
  uint32_t start = ARM_DWT_CYCCNT;

  // Wait for any previous pulse to end
  while (digitalReadFast(pin) == state) {
    if ((ARM_DWT_CYCCNT - start) > timeout_cycles)
      return 0;
  }

  // Wait for pulse to start
  while (digitalReadFast(pin) != state) {
    if ((ARM_DWT_CYCCNT - start) > timeout_cycles)
      return 0;
  }

  // Pulse started
  uint32_t rise = ARM_DWT_CYCCNT;

  // Wait for pulse to end
  while (digitalReadFast(pin) == state) {
    if ((ARM_DWT_CYCCNT - rise) > timeout_cycles)
      return 0;
  }

  uint32_t fall = ARM_DWT_CYCCNT;

  // Convert cycles → microseconds
  return (fall - rise) / 600;
}

void input() {
  if (Serial.available() > 0) { // expected format: 9 digits (kp,ki,kd) then setpoint e.g. "35050030090"
    String input = Serial.readString();
// target_dist = input.toInt() ;
    kpf = input.substring(0,3).toFloat() ;
    kpb = input.substring(3,6).toFloat() ;
    target_dist=input.substring(6).toFloat();
    spf=target_dist*pprf;
    spb=target_dist*pprb;
  }
  
}


int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseInFast(echoPin, HIGH, 30000);
  // return 1;
  if (duration == 0) {
    return -1; // no echo
  }
  
  int distance = duration * 0.0343 / 2; // cm
  return distance;
}
void correct_error(){
  if(!both) return;

  if(distances[front] < 0 || distances[back] < 0) return;

  float est_f = filt_f * dist_tickf;
  float est_b = filt_b * dist_tickb;

  err_tickf += alpha * (est_f - myEncfront.read());
  err_tickb += alpha * (est_b - myEncback.read());
}

void pid(){
  // input();

  spf=target_f*pprf;
  spb=target_b*pprb;
  long currentCountsf = myEncfront.read()+err_tickf;
  long currentCountsb = myEncback.read()+err_tickb;
  float errf = spf - currentCountsf;
  float errb = spb - currentCountsb;
  pidf = (kpf*errf);
  pidb = (kpb*errb);
  Serial.printf("pidf: %.2f     pidb: %.2f\n", pidf, pidb);
  Serial.println(l);
  
  if(!l){
    runMotor(climber[0][0], climber[0][1], pidf);
    runMotor(climber[1][0], climber[1][1], pidb);
  }
  else{
    runMotor(climber[0][0], climber[0][1], 0);
    runMotor(climber[1][0], climber[1][1], 0);
  }
}
void drive(int pwmL,int pwmR ,int pwm)
{
analogWrite(pwmL,pwm<0?-1*pwm:0);
analogWrite(pwmR,pwm>0?pwm:0);
}
void loop() {
  ps4_input_IK();
  limitswitch();
  static unsigned long lastUS = 0;
  if (millis() - lastUS > 30) {
    lastUS = millis();
    distances[0] = readUltrasonic(trig[0], echo[0]);
    distances[1] = readUltrasonic(trig[1], echo[1]);
    if(distances[front] >= 0)
      filt_f = 0.7 * filt_f + 0.3 * distances[front];
    
    if(distances[back] >= 0)
      filt_b = 0.7 * filt_b + 0.3 * distances[back];
    
  }
  Serial.print(myEncfront.read());
  Serial.print(" ");
  Serial.print(distances[front]);
  Serial.print(" ");
  Serial.print(myEncback.read());
  Serial.print(" ");
  Serial.print(distances[back]);
  Serial.print(" ");
  Serial.print(err_tickf);
  Serial.print(" ");
  Serial.print(err_tickb);
  Serial.println();
// runMotor(climber[0][0], climber[0][1], 3000);
  

  
  drive(drive_trx[0][0],drive_trx[0][1],right_y*64);
  drive(drive_trx[1][0],drive_trx[1][1],right_y*64);

  for (int i = 0; i < 4; i++){
    drive(drive_omni[i][0], drive_omni[i][1],right_y*64);
  }


  // put your main code here, to run repeatedly:
  pid();

}