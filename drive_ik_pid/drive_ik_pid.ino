#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Cl\ientServerEthernet.h>
#include <Encoder.h>

#define RADIUS 0.063
int ROBOT_RADIUS[3]  = {0.28,0.3,0.3};

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1); // Use   Wire2 for I2C communication

int pwmL_pin[3] = {5, 36, 0};
int pwmR_pin[3] = {4, 37, 1};

Encoder m[3] = { Encoder(26, 27), Encoder(40, 41), Encoder(21, 20) };

bool data_update=true;
IntervalTimer timer;
float dt = 0.075;
int max_rpm = 500;

// RPM PID
// volatile int pwm_pid[] = { 0, 0, 0 };
// volatile float rpm_sp[] = { 0, 0, 0 };

// volatile float kp[] = { 09.0, 09.0, 09.0 };
// volatile float ki[] = { 165.0, 165.0, 165.0 };
// volatile float kd[] = { 00.50, 00.50, 00.50 };

// float error[] = { 0, 0, 0 };
// float eInt[] = { 0, 0, 0 };
// float eDer[] = { 0, 0, 0 };
// float lastError[] = { 0, 0, 0 };

// volatile long oldPosition[3] = { 0, 0, 0 };
// volatile long count[3] = { 0, 0, 0 };  // use volatile for shared variables
// volatile long newPosition[3] = { 0, 0, 0 };

// volatile float rpm_rt[3] = { 0, 0, 0 };
// float cpr[]={1300.0,1300.0,700.0};

// VELOCITY PID
float error_vx, eDer_vx, eInt_vx, error_vy, eDer_vy, eInt_vy; 
float pid_vx, pid_vy;
float Kp_vx, Kp_vy, Ki_vx, Ki_vy, Kd_vx, Kd_vy;
float lastError_vx, lastError_vy;
float vx = 0, vy = 0;


#pragma pack(push, 1) // save current alignment and set to 1 byte
struct ControllerData { 
  int32_t axis[4]; 
  int32_t l2;
  int32_t r2;
  int16_t r1;
  int16_t l1;
  int16_t cross;
  int16_t square;
  int16_t circle;
  int16_t triangle; 
  int16_t touch_button; 
  int16_t turn_pwm;
  int16_t bldc_pwm;
};
#pragma pack(pop) // restore previous alignment

ControllerData jetdata; // Struct instance to hold incoming controller data
ClientServerEthernet<ControllerData> con; // Instance of the ClientServerEthernet class templated with ControllerData


void setup() {
  Serial.begin(115200);
  // pinMode(13,OUTPUT);
  // digitalWrite(13,HIGH);
  Wire1.begin();

  while (!bno.begin()) {
    Serial.println("BNO055 not detected. Check connections to SDA2/SCL2!");
    // 0
  }
  Serial.println("BNO055 detected!");

  // Optional: Use external crystal for better precision
  bno.setExtCrystalUse(true);

  analogWriteResolution(14);
  
  vector<int> client_ip = {192, 168, 1, 101}; // IP address of this device (client)
  vector<int> server_ip = {192, 168, 1, 102}; // IP address of the server to communicate with
  vector<int> subnet_mask = {255, 255, 255, 0}; // Subnet mask for the network
  con = ClientServerEthernet<ControllerData>(client_ip, subnet_mask, server_ip, &jetdata);

  timer.begin(drive_ik, dt*1000000);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void inverseKinematics(float vx, float vy, float omega, float* rpms) {
  float w1 = (-sin(0) * vx + cos(0) * vy + ROBOT_RADIUS[0] * omega) / RADIUS;
  float w2 = (-sin(2 * PI / 3) * vx + cos(2 * PI / 3) * vy + ROBOT_RADIUS[1] * omega) / RADIUS;   //619.81 at full plus y 
  float w3 = (-sin(4 * PI / 3) * vx + cos(4 * PI / 3) * vy + ROBOT_RADIUS[2] * omega) / RADIUS;

  // rad/s --> RPM
  rpms[0] = w1 * 60.0 / (2 * PI);
  rpms[1] = w2 * 60.0 / (2 * PI);   //103.33 at full plus y
  rpms[2] = w3 * 60.0 / (2 * PI);

  // for(int i = 0; i < 3; i++){
  //   rpms[i] = mapFloat(rpms[i], -175, 175, max_rpm, -max_rpm);    //TO BE CHANGED
  // }
}

void runMotor(int pwm_val, int pwmLPin, int pwmRPin){
  analogWrite(pwmLPin, (pwm_val <= 0 ? pwm_val*-1 : 0));
  analogWrite(pwmRPin, (pwm_val >= 0 ? pwm_val : 0));
}

int y=0;
int x=0;
int w=0;

void drive_ik(){
  
  // for (int i = 0; i < 3; i++) {
  //   newPosition[i] = m[i].read();
  //   ::count[i] = abs(newPosition[i] - oldPosition[i]);
  //   rpm_rt[i] = ::count[i] / cpr[i] * 600 * 4.0 / 3;
  //   rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
  //   Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
  //   ::count[i] = 0;
  //   oldPosition[i] = newPosition[i];
  // }

  if(data_update){
    int psAxisX = 0;
    int psAxisY = 0;
    if (jetdata.axis[0] < 125)
      psAxisX = map(jetdata.axis[0], 125, 0, 0, -255);

    else if (jetdata.axis[0] > 135)
      psAxisX = map(jetdata.axis[0], 135, 255, 0, 255);
    else
      psAxisX = 0;

    if (jetdata.axis[1] > 135)
      psAxisY = map(jetdata.axis[1], 135, 255, 0, -255);

    else if (jetdata.axis[1] < 120)
      psAxisY = map(jetdata.axis[1], 125, 0, 0, 255);
    else
      psAxisY = 0;
    if (jetdata.axis[2] > 135)
      w = map(jetdata.axis[2], 135, 255, 0, 255);

    else if (jetdata.axis[2] < 120)
      w = map(jetdata.axis[2], 125, 0, 0, -255);
    else
      w=0;
    y = psAxisY;
    x = psAxisX;
  }


  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);  // linear vel
  // imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // amgular vel in degree/s
  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);    // heading

  float ax = accel.x();
  float ay = accel.y();

  vx += ax * dt;    // actual vel in x
  vy += ay * dt;    // actual vel in y

  // VELOCITY PID
  error_vx = x - vx;
  eDer_vx = (error_vx - lastError_vx)/dt;
  eInt_vx = eInt_vx + error_vx * dt;
  pid_vx = Kp_vx * error_vx + Ki_vx * eInt_vx + Kd_vx * eDer_vx;
  lastError_vx = error_vx;

  error_vy = y - vy;
  eDer_vy = (error_vy - lastError_vy)/dt;
  eInt_vy = eInt_vy + error_vy * dt;
  pid_vy = Kp_vy * error_vy + Ki_vy * eInt_vy + Kd_vy * eDer_vy;
  lastError_vy = error_vy;

  Serial.printf("vx:%0.2f   vy:%0.2f",pid_vx ,pid_vy);
  Serial.println();

  // float vx_corrected = x + pid_vx;
  // float vy_corrected = y + pid_vy;
  // Serial.printf("vx:%0.2f   vy:%0.2f",vx_corrected ,vy_corrected);
  // Serial.println();

  float rpm_cmd[3];
  inverseKinematics(pid_vx, pid_vy, w, rpm_cmd);

  // float rpm_cmd[3];
  // inverseKinematics(x, y, w, rpm_cmd);

  // comment this when u add pid on rpm
  for (int i = 0; i < 3; i++) {
    // rpm_cmd[i] = int(rpm_cmd[i])%14000;
    rpm_cmd[i] = constrain(rpm_cmd[i], -14000, 14000);
    runMotor((int)rpm_cmd[i],pwmL_pin[i],pwmR_pin[i]);
    // Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_cmd[i]);
    // Serial.println();
  }


  // for (int i = 0; i < 3; i++) {
  //   Serial.printf("RPM_%d_input:%0.2f  ", i + 1, rpm_cmd[i]);
  // }

  // for (int i = 0; i < 3; i++) {
  //   error[i] = rpm_cmd[i] - rpm_rt[i];
  //   eDer[i] = (error[i] - lastError[i]) / dt;
  //   eInt[i] = eInt[i] + error[i] * dt;

  //   pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);

  //   //Serial.printf("pwm_pid:%d \n",pwm_pid[i]);
  //   pwm_pid[i]=pwm_pid[i]%16383;
  //   analogWrite(pwmR_pin[i], pwm_pid[i]>=0?pwm_pid[i]:0);
  //   analogWrite(pwmL_pin[i], pwm_pid[i]<=0?pwm_pid[i]*-1:0);

  //   lastError[i] = error[i];
  //   Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_cmd[i]);
  // }
}


void loop() {
  con.MaintainConnection(false);
  con.getData(true);

}
