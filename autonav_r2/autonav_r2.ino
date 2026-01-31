#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Create BNO055 object on Wire2 (SDA2, SCL2)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1); // Use Wire2 for I2C communication
sensors_event_t event;


// ================== GLOBALS & PIN DEFINITIONS ==================

IntervalTimer pid_timer;

// Motor PWM Pins
int lpin[4] = { 0, 4, 12, 22 };
int rpin[4] = { 1, 5, 13, 23 };

// Encoders
Encoder m[4] = { Encoder(20,21), Encoder(27, 26), Encoder(40, 41), Encoder(32, 33) };

// Variables for RPM & PID
volatile float rpm_rt[4] = { 0, 0, 0, 0 };
volatile long oldPosition[4] = { 0, 0, 0, 0 };
volatile unsigned long count[4] = { 0, 0, 0, 0 }; 
volatile long newPosition[4] = { 0, 0, 0, 0 };
int ppr[4] = { 3500, 3500, 3600, 3500 };       // Pulses per revolution

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


// Sensor Data
const int NUM_SENSORS = 3;
int distances[NUM_SENSORS];

// ================== AUTONOMOUS TUNING ==================
float rotate_kp = 10.0;    // Strength of tilt correction (Higher = sharper turns)
float speed_kp = 2.5;     // Strength of forward force (Higher = faster acceleration)
int max_speed = 128;      // Speed limit (Safety cap)
int min_dist = 20;        // Stop distance from front wall (cm)

const float YAW_TOL = 2.0;     // degrees tolerance
float w = 0;

// ================== ARENA ==================
const float ARENA_W = 200.0;   // cm
const float ARENA_H = 200.0;   // cm

// ================== POSE ==================
volatile float bot_x = 0;
volatile float bot_y = 0;
volatile float bot_theta = 0;   // deg

// Target
float target_x = 20;   
float target_y = -20; 


// ================== SETUP ==================
void setup() {
  Serial.begin(9600);
  Serial8.begin(115200);
  Serial8.setTimeout(5); // Safety timeout for sensor reading

//  myusb.begin();

  for (int i = 0; i < 4; i++) {
    analogWriteFrequency(lpin[i], 9000);
    analogWriteFrequency(rpin[i], 9000);
  }

  // Initialize Wire2
  Wire.begin();

  // Initialize BNO055 on Wire2
  while (!bno.begin()) {
    Serial.println("BNO055 not detected. Check connections to SDA2/SCL2!");
    delay(500);
  }
  Serial.println("BNO055 detected!");

  // Optional: Use external crystal for better precision
  bno.setExtCrystalUse(true);

  pid_timer.begin(pid, 75000);      // 75ms loop
  analogWriteResolution(14);
}

// ================== MAIN LOOP ==================
void loop() {
  input();
//  myusb.Task(); // Keep USB stack alive

  bno.getEvent(&event);

  float yaw = event.orientation.x;   // raw yaw from BNO055 (0–360 or -180–180)
//  Serial.printf("yaw: %.2f", yaw);

  if (yaw > 180.0) yaw -= 360.0;
  bot_theta = yaw * DEG_TO_RAD;


  // 1. READ SENSORS FROM SERIAL 8
  if (Serial8.available() > 0) {
    String receivedData = Serial8.readStringUntil('\n');
    // Serial.println(receivedData);
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
  // print_log_line();

}
// ================== PID CONTROL ISR ==================
void pid() {

  // int S2 = distances[1]; // Left Front
  // int S3 = distances[2]; // Left Back
  // int S1 = distances[0]; // Front A
  // int S8 = distances[8]; // Front B

  // int S4 = distances[3]; // back
  // int S5 = distances[5]; // back
  // int S6 = distances[6]; // right back
  // int S7 = distances[7]; // right front 

  int S1 = distances[0]; // fron
  int S2 = distances[1]; // back
  int S3 = distances[2]; // right

  // ===== POSE ESTIMATION =====
  // float left_avg  = (S2 + S3) * 0.5;
  // float front_avg = (S1 + S8) * 0.5;  // back

  // float right_avg  = (S6 + S7) * 0.5;
  // float back_avg = (S4 + S5) * 0.5;  

  // float X = right_avg / 1000.0;  // meters
  // float Y = back_avg  / 1000.0;  // meters

  // float X = right_avg;  // cm
  // float Y = -back_avg;  // 

  float X = S3;  // cm
  float Y = -S2;  // 

 Serial.printf("X: %.2f    y: %.2f\n", X, Y);
  
  bot_x = X * cos(bot_theta) - Y * sin(bot_theta);
  bot_y = X * sin(bot_theta) + Y * cos(bot_theta);
  
  // bot_x = right_avg * cos(bot_theta);
  // bot_y = back_avg * cos(bot_theta);

  // Bound inside arena
//  bot_x = constrain(bot_x, 0, ARENA_W);
//  bot_y = constrain(bot_y, 0, ARENA_H);


  // Safety: If critical sensors read 0 (disconnected/error), stop robot
  if (S2 <= 0 || S3 <= 0 ) {
  // if ( S4 <= 0 || S5 <= 0) {
     //noInterrupts();
     rpm_sp[0]=0; rpm_sp[1]=0; rpm_sp[2]=0; rpm_sp[3]=0;
     //interrupts();
  } 
  else {

    // A. alignment using bno
    // float yaw = event.orientation.x;   // raw yaw from BNO055 (0–360 or -180–180)
    // Serial.printf("yaw: %.2f", yaw);

    // if (yaw > 180.0) {
    //     yaw -= 360.0;
    //     bot_theta = yaw;
    // }
    float yaw_deg = bot_theta * RAD_TO_DEG;
    // Check alignment
    if (abs(yaw_deg) < 0.5) yaw_deg = 0;

    if (abs(yaw_deg) > YAW_TOL) {
      // yaw = constrain(yaw, -30, 30);
      w = rotate_kp * yaw_deg;
      w = constrain(w, -80, 80);
    } else {
        w = 0;
    }

    // -------------------------------------
  
    // --- B. FORWARD FORCE (Using 1 & 8) ---
    // ===== GO TO TARGET =====
    float ex = target_x - bot_x;
    float ey = target_y - bot_y;
  
    // Convert world → robot frame
    float vx =  cos(bot_theta)*ex + sin(bot_theta)*ey;
    float vy = -sin(bot_theta)*ex + cos(bot_theta)*ey;
  
    // Position P controller
    vx = constrain(vx * speed_kp, -max_speed, max_speed);
    vy = constrain(vy * speed_kp, -max_speed, max_speed);
  
    // Stop near target
    if (sqrt(ex*ex + ey*ey) < 10.0) {
      vx = 0;
      vy = 0;
    }


    // Strafe (X) is 0 (Unused)
    // int x = 0; 
     // --- C. MOTOR OUTPUT ---
     // Omni X Mixing
     vx = -vx;
    val[0] = -vx + vy + w;
    val[1] = +vx + vy + w;
    val[2] = vx - vy + w;
    val[3] = -vx - vy + w;

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
    

   analogWrite(rpin[i], pwm_pid[i]>0?pwm_pid[i]:0);
   analogWrite(lpin[i], pwm_pid[i]>=0?0:-1*pwm_pid[i]);

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

void input() {
  if (Serial.available() > 0) {       
  
  String input = Serial.readString();

  target_x = input.substring(0,3).toFloat();
  target_y = input.substring(4).toFloat();

  }
  // Serial.printf(" x:%f\n", target_x);
  // Serial.printf(" y:%f", target_y);
 
}