#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joy.h>

#include <Wire.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1); // Use   Wire2 for I2C communication

#define RADIUS 0.063
float ROBOT_RADIUS[3] = {0.28, 0.3, 0.3};

int pwmL_pin[3] = {5, 36, 0};
int pwmR_pin[3] = {4, 37, 1};
int max_rpm = 6000;

rcl_subscription_t subscriber;
sensor_msgs_msg_Joy msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define MAX_AXES 10
#define MAX_BUTTONS 20
float axes_buffer[MAX_AXES];
int32_t buttons_buffer[MAX_BUTTONS];
int32_t last_buttons[MAX_BUTTONS] = {0};

int x = 0, y = 0, w = 0;

// VELOCITY PID
float dt = 0.075;
float error_vx, eDer_vx, eInt_vx, error_vy, eDer_vy, eInt_vy; 
float pid_vx, pid_vy;

float Kp_vx, Kp_vy, Ki_vx, Ki_vy, Kd_vx, Kd_vy;
float lastError_vx, lastError_vy;
float vx = 0, vy = 0;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK) while(1); }

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}

void inverseKinematics(float vx, float vy, float omega, float* rpms) {
  float w1 = (-sin(0)*vx + cos(0)*vy + ROBOT_RADIUS[0]*omega)/RADIUS;
  float w2 = (-sin(2*PI/3)*vx + cos(2*PI/3)*vy + ROBOT_RADIUS[1]*omega)/RADIUS;
  float w3 = (-sin(4*PI/3)*vx + cos(4*PI/3)*vy + ROBOT_RADIUS[2]*omega)/RADIUS;
  rpms[0] = w1*60.0/(2*PI);
  rpms[1] = w2*60.0/(2*PI);
  rpms[2] = w3*60.0/(2*PI);
}

void runMotor(int pwm_val, int pwmLPin, int pwmRPin){
  analogWrite(pwmRPin, (pwm_val <= 0 ? -pwm_val : 0));
  analogWrite(pwmLPin, (pwm_val >= 0 ? pwm_val : 0));
}

void subscription_callback(const void* msgin){
  const sensor_msgs_msgJoy* joy_msg = (const sensor_msgsmsg_Joy*)msgin;

  for(size_t i=0;i<joy_msg->buttons.size;i++){
    int32_t current = joy_msg->buttons.data[i];
    if(current==1 && last_buttons[i]==0) digitalWrite(13,!digitalRead(13));
    last_buttons[i]=current;
  }

  x = joy_msg->axes.data[1]*255;
  y = joy_msg->axes.data[0]*-255;
  w = joy_msg->axes.data[3]*-255;
  if(abs(w)==255) w=0;
  Serial.printf("x:%d  y:%d  w:%d",x,y,w);
  Serial.println();
}

IntervalTimer motorTimer;

void motor_update(){
  input();

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);  // linear vel
  // imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // angular vel in degree/s
  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);    // heading

  float ax = accel.x();
  float ay = accel.y();

  vx += ax * dt;    // actual vel in x
  vy += ay * dt;    // actual vel in y

  Serial.printf("vx:%0.2f   vy:%0.2f",vx ,vy);
  Serial.println();

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
  // inverseKinematics(x,y,w,rpm_cmd);

  // comment this when u add pid on rpm
  for (int i = 0; i < 3; i++) {
    // rpm_cmd[i] = int(rpm_cmd[i])%14000;
    rpm_cmd[i] = constrain(rpm_cmd[i], -14000, 14000);
    runMotor((int)rpm_cmd[i],pwmL_pin[i],pwmR_pin[i]);
    // Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_cmd[i]);
    // Serial.println();
  }
}

void setup(){
  pinMode(13,OUTPUT); 
  digitalWrite(13,HIGH);
  Serial.begin(115200); 
  delay(2000);
  analogWriteResolution(14);

  // Wire1.begin();

  // while (!bno.begin()) {
  //   Serial.println("BNO055 not detected. Check connections to SDA2/SCL2!");
  //   // 0
  // }
  // Serial.println("BNO055 detected!");

  // // Optional: Use external crystal for better precision
  // bno.setExtCrystalUse(true);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support,0,NULL,&allocator));
  RCCHECK(rclc_node_init_default(&node,"teensy_ps4_node","",&support));

  msg.axes.data = axes_buffer; msg.axes.size = 0; msg.axes.capacity = MAX_AXES;
  msg.buttons.data = buttons_buffer; msg.buttons.size = 0; msg.buttons.capacity = MAX_BUTTONS;

  RCCHECK(rclc_subscription_init_default(
    &subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Joy),"joy"));

  RCCHECK(rclc_executor_init(&executor,&support.context,1,&allocator));
  RCCHECK(rclc_executor_add_subscription(&executor,&subscriber,&msg,&subscription_callback,ON_NEW_DATA));

  motorTimer.begin(motor_update,75*1000); // every 50ms
}
// float Kp_vx, Kp_vy, Ki_vx, Ki_vy, Kd_vx, Kd_vy;

void input() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
    // int i = input.substring(0,1).toInt();
    Kp_vx = input.substring(0,3).toFloat();
    Ki_vx = input.substring(3,6).toFloat();
    Kd_vx = input.substring(6,9).toFloat();
    

    // int comma1 = input.indexOf(',');
    // int comma2 = input.indexOf(',', comma1 + 1);
    
    // if (comma1 > 0 && comma2 > comma1) {
    //   kp = input.substring(0, comma1).toFloat();
    //   ki = input.substring(comma1 + 1, comma2).toFloat();
    //   kd = input.substring(comma2 + 1).toFloat();
    // }
  }
}

void loop(){
  rclc_executor_spin_some(&executor,RCL_MS_TO_NS(10));
  // input();
}