#include <Arduino.h>

#define RADIUS 0.063
int ROBOT_RADIUS[3]  = {0.28,0.3,0.3};

int pwmL_pin[3] = {5, 36, 0};
int pwmR_pin[3] = {4, 37, 1};


int x=0, y=0, w=0; // Robot command velocities

void setup() {
  Serial.begin(115200);
  analogWriteResolution(14);

  for(int i=0; i<3; i++){
    pinMode(pwmL_pin[i], OUTPUT);
    pinMode(pwmR_pin[i], OUTPUT);
  }
}

// Map rad/s → RPM
void inverseKinematics(float vx, float vy, float omega, float* rpms) {
  float w1 = (-sin(0) * vx + cos(0) * vy + ROBOT_RADIUS[0] * omega) / RADIUS;
  float w2 = (-sin(2 * PI / 3) * vx + cos(2 * PI / 3) * vy + ROBOT_RADIUS[1] * omega) / RADIUS;
  float w3 = (-sin(4 * PI / 3) * vx + cos(4 * PI / 3) * vy + ROBOT_RADIUS[2] * omega) / RADIUS;

  rpms[0] = w1 * 60.0 / (2 * PI);
  rpms[1] = w2 * 60.0 / (2 * PI);
  rpms[2] = w3 * 60.0 / (2 * PI);
}

void runMotor(int pwm_val, int pwmLPin, int pwmRPin){
  analogWrite(pwmRPin, (pwm_val <= 0 ? -pwm_val : 0));
  analogWrite(pwmLPin, (pwm_val >= 0 ? pwm_val : 0)); 
}

void drive_ik(){
  float rpm_cmd[3];
  inverseKinematics(x, y, w, rpm_cmd);
  for(int i=0;i<3;i++){
    rpm_cmd[i] = constrain(rpm_cmd[i], -14000, 14000);
    runMotor((int)rpm_cmd[i], pwmL_pin[i], pwmR_pin[i]);
  }
}

void loop() {
  // Read keyboard input via Serial
  if(Serial.available()){
    char c = Serial.read();
    if(c=='W' || c=='w'){
      x = 100; // forward
    }
    else if(c=='S' || c=='s'){
      x = -100; // backward
    }
    else{
      x = 0; // stop
    }
  }

  y = 0; 
  w = 0; 

  drive_ik();
}
