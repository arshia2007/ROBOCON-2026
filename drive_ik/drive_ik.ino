#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <ClientServerEthernet.h>

#define RADIUS 0.063
int ROBOT_RADIUS[3]  = {0.3,0.3,0.28};

// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1); // Use   Wire2 for I2C communication

// int pwmL_pin[3] = { 0, 5, 12};
// int pwmR_pin[3] = { 1, 4, 13};

// int pwmL_pin[3] = { 12, 0, 5};
// int pwmR_pin[3] = { 13, 1, 4};

int pwmL_pin[3] = {36, 0, 5};
int pwmR_pin[3] = {37, 1, 4};

bool data_update=true;
IntervalTimer timer;
float dt = 0.05;
int max_rpm = 12000;

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
  // Wire1.begin();

  // while (!bno.begin()) {
  //   Serial.println("BNO055 not detected. Check connections to SDA2/SCL2!");
  //   // 0
  // }
  // Serial.println("BNO055 detected!");

  // Optional: Use external crystal for better precision
  // bno.setExtCrystalUse(true);

  analogWriteResolution(14);
  
  vector<int> client_ip = {192, 168, 1, 101}; // IP address of this device (client)
  vector<int> server_ip = {192, 168, 1, 102}; // IP address of the server to communicate with
  vector<int> subnet_mask = {255, 255, 255, 0}; // Subnet mask for the network
  con = ClientServerEthernet<ControllerData>(client_ip, subnet_mask, server_ip, &jetdata);

  timer.begin(drive_ik, dt*1000000);
  // pidTimer.begin(pid, dt*1000000);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void inverseKinematics(float vx, float vy, float omega, float* rpms) {
  float w1 = (-sin(0) * vx + cos(0) * vy + ROBOT_RADIUS[0] * omega) / RADIUS;
  float w2 = (-sin(2 * PI / 3) * vx + cos(2 * PI / 3) * vy + ROBOT_RADIUS[1] * omega) / RADIUS;
  float w3 = (-sin(4 * PI / 3) * vx + cos(4 * PI / 3) * vy + ROBOT_RADIUS[2] * omega) / RADIUS;

  // rad/s --> RPM
  rpms[0] = w1 * 60.0 / (2 * PI);
  rpms[1] = w2 * 60.0 / (2 * PI);
  rpms[2] = w3 * 60.0 / (2 * PI);

  // for(int i = 0; i < 3; i++){
  //   rpms[i] = mapFloat(rpms[i], -175, 175, max_rpm, -max_rpm);
  //   // Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpms[i]);
  //   // Serial.println();
  // }

}

void runMotor(int pwm_val, int pwmLPin, int pwmRPin)
{
  analogWrite(pwmRPin, (pwm_val <= 0 ? pwm_val*-1 : 0));
  analogWrite(pwmLPin, (pwm_val >= 0 ? pwm_val : 0)); 
}

int y=0;
int x=0;
int w = 0;

void drive_ik(){
  
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
    x = psAxisY;
    y = psAxisX;
  }
  Serial.printf("x:%d  y:%d  w:%d",x,y,w);
  Serial.println();

  // imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);  // linear vel
  // imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // amgular vel in degree/s
  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);    // heading

  // gyro_z = gyro.z(); 
  // Omega = gyro.z()* PI / 180.0;  // degree/s --> rad/s

  float rpm_cmd[3];
  inverseKinematics(x, y, w, rpm_cmd);
  for(int i = 0; i < 3; i++){
    // Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_cmd[i]);
    // Serial.println();
  }

  for (int i = 0; i < 3; i++) {
    // rpm_cmd[i] = int(rpm_cmd[i])%14000;
    rpm_cmd[i] = constrain(rpm_cmd[i], -14000, 14000);
    runMotor((int)rpm_cmd[i],pwmL_pin[i],pwmR_pin[i]);
    // Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_cmd[i]);
    // Serial.println();
  }
}


void loop() {
  con.MaintainConnection(false);  // keep false → non-blocking
  con.getData(true);             
  // Serial.println("ok");
}


