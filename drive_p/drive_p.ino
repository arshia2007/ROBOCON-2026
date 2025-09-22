#include <Encoder.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>

#define RADIUS 0.127
#define ROBOT_RADIUS 0.3
float Kp_pos=30.0; 
float Kp_vel=0.0;
float Kp_acc=0.0;
#define WHEEL_MAX_RPM 200


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1); // Use   Wire2 for I2C communication

sensors_event_t event;

IntervalTimer pidTimer;

int pwmL_pin[3] = { 2, 0, 6 };
int pwmR_pin[3] = { 3, 1, 7 };

volatile long oldPosition[3] = { 0, 0, 0 };
volatile long count[3] = { 0, 0, 0 };  
volatile long newPosition[3] = { 0, 0, 0 };
// float cpr[]={1300.0,1300.0,1300.0};
float cpr[]={608.0,608.0,608.0};

Encoder m[3] = { Encoder(21,20), Encoder(26,27), Encoder(41,40) };
volatile float wheel_rpm[3] = { 0, 0, 0 };

float dt = 0.05;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // pinMode(13,OUTPUT);
  // digitalWrite(13,HIGH);
  Wire1.begin();

  // Initialize BNO055 on Wire2
  while (!bno.begin()) {
    Serial.println("BNO055 not detected. Check connections to SDA2/SCL2!");
    // 0
  }
  Serial.println("BNO055 detected!");

  // Optional: Use external crystal for better precision
  bno.setExtCrystalUse(true);

  analogWriteResolution(14);

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

volatile float error_Ax = 0.0;
volatile float error_Ay = 0.0;
volatile float error_Atheta = 0.0;

// DEsired POsition
volatile float X_d = 7.5; //2.82
volatile float Y_d = 0.0;
volatile float Theta_d = 0.0;


volatile float Vtheta_d = 0.0;

volatile float Ax_d = 0.0;
volatile float Ay_d = 0.0;

volatile float gyro_z = 0.0;
volatile float yaw = 0.0;
volatile float Theta = 0.0;
volatile float Omega = 0.0;

// volatile float last_Omega = 0.0;

float rpmToRad(float rpm) {
  return rpm * 2.0 * PI / 60.0;
}


void get_rpm(volatile float* wheel_rpm){

  for (int i = 0; i < 3; i++) {
  newPosition[i] = m[i].read();
  count[i] = abs(newPosition[i] - oldPosition[i]);
  // count=newPosition<oldPosition?-count:count;
  wheel_rpm[i] = count[i] / cpr[i]* 600 * 4.0 / 3;
  wheel_rpm[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
    // Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
  Serial.println("_-------------------------TICKS-------------------------------------");
  Serial.println(newPosition[0]);
  Serial.println(newPosition[1]);
  Serial.println(newPosition[2]);
  count[i] = 0;
  oldPosition[i] = newPosition[i];
  }
  
}

void forwardKinematics(volatile float* wheel_rpm,volatile float* vx,volatile float* vy,volatile float* omega) {
  get_rpm(wheel_rpm);
  float w[3];
  for (int i = 0; i < 3; i++) w[i] = rpmToRad(wheel_rpm[i]);
  *vx = (RADIUS / 3.0) * (-sin(0) * w[0] - sin(2 * PI / 3) * w[1] - sin(4 * PI / 3) * w[2]);
  *vy = (RADIUS / 3.0) * (cos(0) * w[0] + cos(2 * PI / 3) * w[1] + cos(4 * PI / 3) * w[2]);
  *omega = (RADIUS / (3.0 * ROBOT_RADIUS)) * (w[0] + w[1] + w[2]);
}

// void forwardKinematics(float* rpms, float* vx, float* vy, float* omega) {
//   float w[3];
//   for (int i = 0; i < 3; i++) w[i] = rpmToRad(rpms[i]);
//   *vx = (RADIUS / 3.0) * (-sin(0) * w[0] - sin(2 * PI / 3) * w[1] - sin(4 * PI / 3) * w[2]);
//   *vy = (RADIUS / 3.0) * (cos(0) * w[0] + cos(2 * PI / 3) * w[1] + cos(4 * PI / 3) * w[2]);
//   *omega = (RADIUS / (3.0 * ROBOT_RADIUS)) * (w[0] + w[1] + w[2]);
// }


void inverseKinematics(float vx, float vy, float omega, float* rpms) {
  float w1 = (-sin(0) * vx + cos(0) * vy + ROBOT_RADIUS * omega) / RADIUS;
  float w2 = (-sin(2 * PI / 3) * vx + cos(2 * PI / 3) * vy + ROBOT_RADIUS * omega) / RADIUS;
  float w3 = (-sin(4 * PI / 3) * vx + cos(4 * PI / 3) * vy + ROBOT_RADIUS * omega) / RADIUS;
  // float w1 = (-sin(0) * vx + cos(0) * vy + ROBOT_RADIUS * omega);
  // float w2 = (-0.5 * vx - 0.866 * vy + ROBOT_RADIUS * omega)*5;
  // float w3 = (-0.5 * vx + 0.866 * vy + ROBOT_RADIUS * omega)*5;

  rpms[0] = w1 * 60.0 / (2 * PI);
  rpms[1] = w2 * 60.0 / (2 * PI);
  rpms[2] = w3 * 60.0 / (2 * PI);
  // for (int i=0; i<3; i++){
  //   if (rpm_cmd[i] < 600){
  //     rpm_cmd[i] = 0;
  //   }
  // }
  // for (int i =0; i <3; i++){
  //   if(rpms[i]!=0){
  //     rpms[i] = map(rpms[i],0,16383,450,16383);
  //   }
  // }
}


void pid(){
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  ax = accel.x();
  ay = accel.y();
  gyro_z = gyro.z(); 
  yaw = euler.z();
  forwardKinematics(wheel_rpm, &vx, &vy, &omega);

  // --- State Estimation ---
  Vx = vx;
  Vy = vy;
  Omega = gyro_z* PI / 180.0;
  X += Vx * dt;
  Y += Vy * dt;
  Theta = yaw* PI / 180.0;

  // --- Position Control ---
  float error_x = X_d - X;
  float error_y = Y_d - Y;
  float error_theta = Theta_d - Theta;

  float Vx_d = Kp_pos * error_x;
  float Vy_d= Kp_pos * error_y;
  float Omega_d = Kp_pos * error_theta;

  // // --- Velocity Control ---
  // float error_Vx = Vx_d - Vx;
  // float error_Vy = Vy_d - Vy;
  // float error_Omega = Omega_d - Omega;

  // float Ax_d = Kp_vel * error_Vx;
  // float Ay_d = Kp_vel * error_Vy;
  // float Alpha_d = Kp_vel * error_Omega;

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
  inverseKinematics(Vx_d, Vy_d, Omega_d, rpm_cmd);
  // inverseKinematics(Fx, Fy, Tau, rpm_cmd);
  // inverseKinematics(Vx_d, Vy_d, 0, rpm_cmd);



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
  Serial.print("X: "); Serial.print(X);
  Serial.print(" | Y: "); Serial.print(Y);
  Serial.print(" | Theta: "); Serial.println(Theta);
  Serial.println(Kp_pos);
  Serial.println(rpm_cmd[0]);
  Serial.println(rpm_cmd[1]);
  Serial.println(rpm_cmd[2]);

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
    Kp_acc = input.substring(6,9).toFloat();
    

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
  // input();

}