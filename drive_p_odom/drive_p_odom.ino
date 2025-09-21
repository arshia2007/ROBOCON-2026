#include <Encoder.h>
#include <math.h>

#define RADIUS 0.0635  
#define ROBOT_RADIUS 0.4
float Kp_pos=0.0; 
float Kp_vel=0.0;
float Kp_acc=0.0;
#define WHEEL_MAX_RPM 200

IntervalTimer pidTimer;

int pwmL_pin[3] = { 2, 7, 12 };
int pwmR_pin[3] = { 3, 6, 13 };

Encoder m[3] = { Encoder(21,20), Encoder(26,27), Encoder(40,41) };

volatile long oldPosition[3] = { 0, 0, 0 };
volatile long count[3] = { 0, 0, 0 };  
volatile long newPosition[3] = { 0, 0, 0 };
// float cpr[]={1300.0,1300.0,1300.0};
float cpr[]={1300.0,700.0,700.0};


volatile float wheel_rpm[3] = { 0, 0, 0 };

float dt = 0.05;


// === Encoder Info Struct ===
struct EncoderInfo {
  Encoder encoder;   // Encoder object
  float x;           // X position from robot center (in cm)
  float y;           // Y position from robot center (in cm)
  // float angle;    // Future use
  volatile long prevTick;  // Must be volatile for safety
};

// === Encoder Setup ===
EncoderInfo enc_left  = { Encoder(29, 28),   0.0,  41.6,  0 };
EncoderInfo enc_right = { Encoder(33, 32),   0.0, -41.6,  0 };
EncoderInfo enc_front = { Encoder(31, 30),  -41.6,  0.0,  0 };

// === Constants ===
const float WHEEL_DIAMETER = 5.8;               //5.7 cm
const float TICKS_PER_REV_encx = 2330.0;
const float TICKS_PER_REV_ency = 2330.0;

const float CW_CORRECTION_FACTOR = 360.0 / 363.0;   
const float CCW_CORRECTION_FACTOR = 360.0 / 358.5; 

const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

const float CX = WHEEL_CIRCUMFERENCE / TICKS_PER_REV_encx; // cm per tick
const float CY = WHEEL_CIRCUMFERENCE / TICKS_PER_REV_ency;

// === Computed Geometry ===
float DIST_LR = 0.0;
float FRONT_OFFSET = 0.0;

// === Odometry State ===
volatile float globalX = 0.0;
volatile float globalY = 0.0;
volatile float prevX = 0.0;
volatile float prevY = 0.0;
volatile float prevTheta = 0.0;

volatile float globalTheta = 0;

// === Tick Deltas (global for performance) ===
volatile long deltaLeft = 0;
volatile long deltaRight = 0;
volatile long deltaFront = 0;

void Odom() {
  // === 1. Read current ticks ===
  long currLeft = enc_left.encoder.read();
  long currRight = enc_right.encoder.read();
  long currFront = enc_front.encoder.read();

  // Serial.print("Y1: "); Serial.print(currLeft);
  // Serial.print(" | Y2: "); Serial.print(currRight);
  // Serial.print(" | Y3:  "); Serial.println(currFront);

  // === 2. Compute tick deltas globally ===
  deltaLeft = currLeft - enc_left.prevTick;
  deltaRight = currRight - enc_right.prevTick;
  deltaFront = currFront - enc_front.prevTick;

  // === 3. Update stored ticks ===
  enc_left.prevTick  = currLeft;
  enc_right.prevTick = currRight;
  enc_front.prevTick = currFront;

  // === 4. Convert ticks to distance ===
  float dL = deltaLeft * CY;
  float dR = deltaRight * CY;
  float dF = deltaFront * CX;

  // === 5. Rotation calculation ===
  float deltaTheta = (dR - dL) / DIST_LR;
  float midTheta = globalTheta + (deltaTheta / 2.0);


  // === 6. Local displacement ===
  float dy_local = (dL + dR) / 2.0;
  float dx_local = dF - (FRONT_OFFSET * deltaTheta);

  // === 7. Convert to global frame ===
  float cosT = cos(midTheta);
  float sinT = sin(midTheta);

  float dx_global = cosT * dx_local - sinT * dy_local;
  float dy_global = sinT * dx_local + cosT * dy_local;

  // === 8. Update global pose ===
  globalX += dx_global;
  globalY += dy_global;
  globalTheta += deltaTheta;

  
  while (globalTheta > PI)  globalTheta -= TWO_PI;
  while (globalTheta < -PI) globalTheta += TWO_PI;

  // Serial.print("X: "); Serial.print(globalX, 2);
  // Serial.print(" | Y: "); Serial.print(globalY, 2);
  // Serial.print(" | θ (deg): "); Serial.println(globalTheta * 180.0 / PI, 2);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  analogWriteResolution(14);

  DIST_LR = sqrt(pow(enc_left.x - enc_right.x, 2) + pow(enc_left.y - enc_right.y, 2));
  FRONT_OFFSET = enc_front.x;  // X offset of front encoder from center

  pidTimer.begin(pid, dt*1000000);

}

int max_rpm = 300;

volatile float vx = 0.0, vy = 0.0, omega = 0.0;
volatile float Vx = 0.0, Vy = 0.0;
volatile float ax = 0.0, ay = 0.0;
volatile float X = 0.0, Y = 0.0;


volatile float error_X = 0.0;
volatile float error_Y = 0.0;
volatile float error_theta = 0.0;

volatile float error_Vx = 0.0;
volatile float error_Vy = 0.0;
volatile float error_Vtheta = 0.0;

float V_MAX = 30;      // Example: max 30 m/s
float OMEGA_MAX = 10;  


volatile float error_Ax = 0.0;
volatile float error_Ay = 0.0;
volatile float error_Atheta = 0.0;

// DEsired POsition
volatile float X_d = 0.0; //2.82
volatile float Y_d = 0.0;
volatile float Theta_d = 0.0;


volatile float Vtheta_d = 0.0;

volatile float Ax_d = 0.0;
volatile float Ay_d = 0.0;


volatile float Theta = 0.0;
volatile float Omega = 0.0;




void inverseKinematics(float vx, float vy, float omega, float* rpms) {
  float w1 = (-sin(0) * vx + cos(0) * vy + ROBOT_RADIUS * omega) / RADIUS;
  float w2 = (-sin(2 * PI / 3) * vx + cos(2 * PI / 3) * vy + ROBOT_RADIUS * omega) / RADIUS;
  float w3 = (-sin(4 * PI / 3) * vx + cos(4 * PI / 3) * vy + ROBOT_RADIUS * omega) / RADIUS;
  

  // Convert from rad/s to RPM
  rpms[0] =-1* w1 * 60.0 / (2 * PI);
  rpms[1] = -1* w2 * 60.0 / (2 * PI);
  rpms[2] = -1* w3 * 60.0 / (2 * PI);

}


void pid(){
  Odom();

  // --- State Estimation ---
  X = globalX / 100;
  Y = globalY / 100;
  Theta = globalTheta;

  Vx = (X - prevX) / dt;
  Vy = (Y - prevY) / dt;
  Omega = (Theta - prevTheta) / dt;   // theta(rad) -> angualr velocity

  prevX = X;
  prevY = Y;
  prevTheta = Theta;

  // --- Position Control ---
  float error_x = X_d - X;
  float error_y = Y_d - Y;
  float error_theta = Theta_d - Theta;

  // error_x /= 5;
  // error_y /= 5;

  float Vx_d = Kp_pos * error_x;
  float Vy_d= Kp_pos * error_y;
  float Omega_d = Kp_pos * error_theta;

  // Vx_d = constrain(Vx_d, -V_MAX, V_MAX);
  // Vy_d = constrain(Vy_d, -V_MAX, V_MAX);
  // Omega_d = constrain(Omega_d, -OMEGA_MAX, OMEGA_MAX);

  // // --- Velocity Control ---
  float error_Vx = Vx_d - Vx;
  float error_Vy = Vy_d - Vy;
  float error_Omega = Omega_d - Omega;

  float Ax_d = Kp_vel * error_Vx;
  float Ay_d = Kp_vel * error_Vy; 
  float Alpha_d = Kp_vel * error_Omega;

  Ax_d = constrain(Ax_d, -V_MAX, V_MAX);
  Ay_d = constrain(Ay_d, -V_MAX, V_MAX);
  Omega_d = constrain(Omega_d, -OMEGA_MAX, OMEGA_MAX);

  // // --- Acceleration Control ---
  // float error_Ax = Ax_d - ax;
  // float error_Ay = Ay_d - ay;
  // // float alpha = (Omega - last_Omega) / dt;
  // // last_Omega = Omega;
  // // float error_Alpha = Alpha_d - alpha;
  // float error_Alpha = Alpha_d - omega;

  // float Fx = Kp_acc * error_Ax;
  // float Fy = Kp_acc * error_Ay;
  // float Tau = Kp_acc * error_Alpha;

  float rpm_cmd[3];
  inverseKinematics(Ay_d, Ax_d, 0, rpm_cmd);
  // inverseKinematics(Fx, Fy, Tau, rpm_cmd);
  // inverseKinematics(Vx_d, Vy_d, Omega_d, rpm_cmd);



  for (int i = 0; i < 3; i++) {
    if(rpm_cmd[i]>14000){
      rpm_cmd[i]=14000;
    }
    if(rpm_cmd[i]<-14000){
      rpm_cmd[i]=-14000;
    }
    // rpm_cmd[i] = int(rpm_cmd[i])%14000;
    runMotor(int(rpm_cmd[i]),pwmL_pin[i],pwmR_pin[i]);
  }

  Serial.print("X: "); Serial.print(globalX/100);
  Serial.print(" | Y: "); Serial.print(globalY/100);
  Serial.print(" | Theta: "); Serial.println(Theta * 180.0 / PI);
  Serial.println(Kp_pos);
  Serial.print("Vy: "); Serial.print(Vy);
  Serial.print(" | Vx: "); Serial.print(Vx);
  Serial.print(" | omega: "); Serial.println(Omega_d);
  // Serial.println(rpm_cmd[0]);
  // Serial.println(rpm_cmd[1]);
  // Serial.println(rpm_cmd[2]);

}

void runMotor(int pwm_val, int pwmLPin, int pwmRPin)
{
  analogWrite(pwmLPin, (pwm_val <= 0 ? pwm_val*-1 : 0));
  analogWrite(pwmRPin, (pwm_val >= 0 ? pwm_val : 0));
}

void input() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
    // int i = input.substring(0,1).toInt();
    Kp_pos = input.substring(0,3).toFloat();
    Kp_vel = input.substring(3,6).toFloat();
    Y_d = input.substring(6,9).toFloat();
    // Y_d= input.substring(10,13).toFloat();
    

    // int comma1 = input.indexOf(',');
    // int comma2 = input.indexOf(',', comma1 + 1);
    
    // if (comma1 > 0 && comma2 > comma1) {
    //   kp = input.substring(0, comma1).toFloat();
    //   ki = input.substring(comma1 + 1, comma2).toFloat();
    //   kd = input.substring(comma2 + 1).toFloat();
    // }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  input();

}