#include "USBHost_t36.h"
#include <Encoder.h>

// ================== GLOBALS & PIN DEFINITIONS ==================
// IntervalTimer rpm_timer;
IntervalTimer pid_timer;

// Motor PWM Pins
int lpin[4] = { 4, 12, 22, 0 };
int rpin[4] = { 5, 13, 23, 1 };

// Encoders
Encoder m[4] = { Encoder(21,20), Encoder(27, 26), Encoder(41, 40), Encoder(32, 33) };


// Variables for RPM & PID
volatile float rpm_rt[4] = { 0, 0, 0, 0 };
volatile long oldPosition[4] = { 0, 0, 0, 0 };
volatile unsigned long count[4] = { 0, 0, 0, 0 }; 
volatile long newPosition[4] = { 0, 0, 0, 0 };
int ppr[4] = { 1300, 700, 1300, 1300 };       // Pulses per revolution

// Settings
int res = 3000;
int max_rpm = 150; 

// PID Constants (Tunable via Serial)
volatile float kp[] = { 09.0, 09.0, 09.0, 09.0 };
volatile float ki[] = { 165.0, 165.0, 165.0, 165.0 };
volatile float kd[] = { 00.50, 00.50, 00.50, 00.50 };

float error[] = { 0, 0, 0, 0 };
float eInt[] = { 0, 0, 0, 0 };
float eDer[] = { 0, 0, 0, 0 };
float lastError[] = { 0, 0, 0, 0 };
volatile int pwm_pid[] = { 0, 0, 0, 0 };
volatile float rpm_sp[] = { 0, 0, 0, 0 };
volatile float val[] = { 0, 0, 0, 0 };

// USB Host (Required for library stability)
USBHost myusb; 

// Sensor Data
const int NUM_SENSORS = 12;
int distances[NUM_SENSORS];

// ================== AUTONOMOUS TUNING ==================
float rotate_kp = 10.0;    // Strength of tilt correction (Higher = sharper turns)
float speed_kp = 2.5;     // Strength of forward force (Higher = faster acceleration)
int max_speed = 128;      // Speed limit (Safety cap)
int min_dist = 20;        // Stop distance from front wall (cm)

// ================== SETUP ==================
void setup() {
  Serial.begin(9600);
  Serial8.begin(115200);
  Serial8.setTimeout(5); // Safety timeout for sensor reading

  myusb.begin();

  for (int i = 0; i < 4; i++) {
    analogWriteFrequency(lpin[i], 9000);
    analogWriteFrequency(rpin[i], 9000);
  }

  // Start Interrupt Timers
  // rpm_timer.priority(1);
  // pid_timer.priority(2);
  
  // rpm_timer.begin(calc_rpm, 75000); // 75ms loop
  pid_timer.begin(pid, 75000);      // 75ms loop
  
  analogWriteResolution(14);
}

// ================== MAIN LOOP ==================
void loop() {
  myusb.Task(); // Keep USB stack alive

  // 1. READ SENSORS FROM SERIAL 8
  if (Serial8.available() > 0) {
    String receivedData = Serial8.readStringUntil('\n');
    receivedData.trim();
    
    if (receivedData.length() > 0) {
      int sensorIndex = 0;
      int startIndex = 0;
      while (sensorIndex < NUM_SENSORS) {
        int commaIndex = receivedData.indexOf(',', startIndex);
        String valueStr;
        if (commaIndex == -1) valueStr = receivedData.substring(startIndex);
        else valueStr = receivedData.substring(startIndex, commaIndex);
        
        valueStr.trim();
        if (valueStr.length() > 0) {
          distances[sensorIndex] = valueStr.toInt();
          sensorIndex++;
        }
        
        if (commaIndex == -1) break;




        startIndex = commaIndex + 1;
      }
    }
  }
  print_log_line();

}
// ================== PID CONTROL ISR ==================
void pid() {

  int S2 = distances[2]; // Left Front
  int S3 = distances[3]; // Left Back
  int S1 = distances[0]; // Front A
  int S8 = distances[1]; // Front B

  // Safety: If critical sensors read 0 (disconnected/error), stop robot
  if (S2 <= 0 || S3 <= 0 || S1 <= 0 || S8 <= 0) {
  // if ( S1 <= 0 || S8 <= 0) {
     //noInterrupts();
     rpm_sp[0]=0; rpm_sp[1]=0; rpm_sp[2]=0; rpm_sp[3]=0;
     //interrupts();
  } 
  else {
    int tilt_error = S2 - S3;
    //  int w = tilt_error;
    float w = rotate_kp * tilt_error;
    w = constrain(w, -80, 80);

     // --- B. FORWARD FORCE (Using 1 & 8) ---
     // Average the front distance
    int front_space = (S1 + S8) / 2;
     
    // Calculate Speed: More space = Higher Speed
    // If space is less than min_dist, speed becomes 0
    int y = ((front_space - min_dist) * speed_kp);
    
    // Limits
    if (y > max_speed) y = max_speed;
    if (y < 0) y = 0; // Prevent reversing, just stop.

    // Strafe (X) is 0 (Unused)
    int x = 0; 
    // w = 0;
    // Serial.print(w); Serial.print(",");
    // Serial.print(y); Serial.print(",");
    // Serial.println(); 
     // --- C. MOTOR OUTPUT ---
     // Omni X Mixing
    val[0] = -x + y + w/2;
    val[1] = +x + y + w/2;
    val[2] = x - y + w/2;
    val[3] = -x - y + w/2;

    // for(int i = 0; i<4; i++){
    //   Serial.printf("ik %d: ", i+1); Serial.print(val[i]); Serial.print("\t");
    // }
    // Serial.println();

    rpm_sp[0] = map(val[0], -300, 300, -max_rpm, max_rpm);
    rpm_sp[1] = map(val[1], -300, 300, -max_rpm, max_rpm);
    rpm_sp[2] = map(val[2], -300, 300, -max_rpm, max_rpm);
    rpm_sp[3] = map(val[3], -300, 300, -max_rpm, max_rpm);

    // for(int i = 0; i<4; i++){
    //   Serial.printf("rpm %d: ", i+1); Serial.print(rpm_sp[i]); Serial.print("\t");
    // }
    // Serial.println();


     //interrupts();
  }

  for (int i = 0; i < 4; i++) {
    newPosition[i] = m[i].read();
    count[i] = abs(newPosition[i] - oldPosition[i]);
    
    // Calculate RPM with float precision
    rpm_rt[i] = ((float)count[i] / ppr[i]) * 600.0 * 4.0 / 3.0;
    
    // Determine Direction
    if (newPosition[i] < oldPosition[i]) {
       rpm_rt[i] *= -1;
    }
    // for(int i = 0; i<4; i++){
    //   Serial.printf("rpm %d: ", i+1); Serial.print(rpm_rt[i]); Serial.print("\t");
    // }
    // Serial.println();
    
    oldPosition[i] = newPosition[i];
  }

// WITHOUT PID
  //   if (rpm_sp[i] >= 0) {
  //     analogWrite(rpin[i], 0);
  //     analogWrite(lpin[i], abs(rpm_sp[i]));
  //   } else {
  //     analogWrite(lpin[i], 0);
  //     analogWrite(rpin[i], abs(rpm_sp[i]));
  //   }
  // }

  
// PID BLOCK

  for (int i = 0; i < 4; i++) {
    error[i] = rpm_sp[i] - rpm_rt[i];
    eDer[i] = (error[i] - lastError[i]) / 0.075;
    eInt[i] = eInt[i] + error[i] * 0.075;

    pwm_pid[i] = ((kp[i] * error[i]) + (ki[i] * eInt[i]) + (kd[i] * eDer[i]));

    // for(int i = 0; i<4; i++){
    //   Serial.printf("pid %d: ", i+1); Serial.print(pwm_pid[i]); Serial.print("\t");
    // }
    // Serial.println();
    pwm_pid[i] = constrain(pwm_pid[i], -3000, 3000);
    

    analogWrite(rpin, pwm_pid[i]>0?pwm_pid[i]:0);
    analogWrite(lpin, pwm_pid[i]>=0?0:-1*pwm_pid[i]);

    // for(int i = 0; i<4; i++){
    //   Serial.printf("pid %d: ", i+1); Serial.print(pwm_pid[i]); Serial.print("\t");
    // }
    // Serial.println();
    lastError[i] = error[i];
  }
}

void print_log_line() {
  // 1) Timestamp in ms
  unsigned long now = millis();
  Serial.print(now);
  Serial.print(",");

  // 2) Dist_0..Dist_11
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(distances[i]);
    Serial.print(",");
  }
  Serial.println();
  // 3) RPM_FL, RPM_FR, RPM_BL, RPM_BR
  //    Assuming index mapping:
  //    0 = Front Left, 1 = Front Right, 2 = Back Left, 3 = Back Right
  // Serial.print(w); Serial.print(",");
  // Serial.print(y); Serial.print(",");
  // Serial.println();  // end of line
}