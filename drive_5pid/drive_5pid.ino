#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joy.h>

#include <Wire.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1); // Use   Wire2 for I2C communication
#include <Encoder.h>


//#define RADIUS 0.063      //in m  
//float ROBOT_RADIUS[3] = {0.28, 0.3, 0.3};

#define RADIUS 6.3                // in cm
float ROBOT_RADIUS[3] = {28, 30, 30};

int pwmL_pin[3] = {1, 37, 4};
int pwmR_pin[3] = {0, 36, 5};
int max_rpm = 400;

Encoder m[3] = { Encoder(20, 21), Encoder(40, 41), Encoder(27, 26) };


rcl_subscription_t subscriber;
sensor_msgs__msg__Joy msg;  

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

// RPM PID
volatile int pwm_pid[] = { 0, 0, 0 };
volatile float rpm_sp[] = { 0, 0, 0 };

volatile float kp[] = { 09.0, 09.0, 09.0 };
volatile float ki[] = { 165.0, 165.0, 165.0 };
volatile float kd[] = { 00.50, 00.50, 00.50 };

float error[] = { 0, 0, 0 };
float eInt[] = { 0, 0, 0 };
float eDer[] = { 0, 0, 0 };
float lastError[] = { 0, 0, 0 };

volatile long oldPosition[3] = { 0, 0, 0 };
volatile long count[3] = { 0, 0, 0 };  // use volatile for shared variables
volatile long newPosition[3] = { 0, 0, 0 };

volatile float rpm_rt[3] = { 0, 0, 0 };
float cpr[]={700.0,700.0,1300.0};


// === ETHERNET CONFIGURATION FROM CODE 1 ===
#if defined(ARDUINO_TEENSY41)
void get_teensy_mac(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
}
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(13, !digitalRead(13));
    delay(100);
  }
}
// === END ETHERNET CONFIGURATION ===

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}

void inverseKinematics(float vy, float vx, float omega, float* rpms) {
  float w1 = (-sin(0)*vx + cos(0)*vy + ROBOT_RADIUS[0]*omega)/RADIUS;
  float w2 = (-sin(2*PI/3)*vx + cos(2*PI/3)*vy + ROBOT_RADIUS[1]*omega)/RADIUS;
  float w3 = (-sin(4*PI/3)*vx + cos(4*PI/3)*vy + ROBOT_RADIUS[2]*omega)/RADIUS;

  // rad/s --> RPM
  rpms[0] = w1*60.0/(2*PI);
  rpms[1] = w2*60.0/(2*PI);
  rpms[2] = w3*60.0/(2*PI);

//  rpms[0] = map(x + 0.3*w, -175, 175, max_rpm, -max_rpm);
//  rpms[1] = map(-0.5 * x - 0.852 * y + 0.3*w, -175, 175, max_rpm, -max_rpm);
//  rpms[2] = map(-0.5 * x + 0.860 * y + 0.3*w, -175, 175, max_rpm, -max_rpm);

  
}

void runMotor(int pwm_val, int pwmLPin, int pwmRPin){
  analogWrite(pwmRPin, (pwm_val <= 0 ? -pwm_val : 0));
  analogWrite(pwmLPin, (pwm_val >= 0 ? pwm_val : 0));
}

void subscription_callback(const void* msgin){
  const sensor_msgs__msg__Joy* joy_msg = (const sensor_msgs__msg__Joy*)msgin;  // Fixed type

  for(size_t i=0;i<joy_msg->buttons.size;i++){
    int32_t current = joy_msg->buttons.data[i];
    if(current==1 && last_buttons[i]==0) digitalWrite(13,!digitalRead(13));
    last_buttons[i]=current;
  }

  y = joy_msg->axes.data[1]*375;
  x = joy_msg->axes.data[0]*375;
  w = joy_msg->axes.data[5]*-60;
  if(abs(w)==255) w=0;
  Serial.printf("x:%d  y:%d  w:%d",x,y,w);
  Serial.println();
}

IntervalTimer motorTimer;

void motor_update(){
  input();
//  Serial.printf("1.x_afterPID:%d   y_AFTERPID:%d",x ,y);
//  Serial.println();

   for (int i = 0; i < 3; i++) {
     newPosition[i] = m[i].read();
     ::count[i] = abs(newPosition[i] - oldPosition[i]);
     rpm_rt[i] = ::count[i] / cpr[i] * 600 * 4.0 / 3;
     rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
//     Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
//     Serial.println();
     ::count[i] = 0;
     oldPosition[i] = newPosition[i];
   }
//   Serial.printf("2. x_afterPID:%d   y_AFTERPID:%d",x ,y);
//  Serial.println();

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);  // linear vel
  // imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // amgular vel in degree/s
  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);    // heading

  float ax = accel.x();
  float ay = accel.y();
//  Serial.printf("vx:%0.2f   vy:%0.2f",ax ,ay);
//  Serial.println();

  vx += ax * dt*100;    // actual vel in x in cm/s
  vy += ay * dt*100;    // actual vel in y

  Serial.printf("vx:%0.2f   vy:%0.2f   ",vx ,vy);
//  Serial.println();


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

  Serial.printf("x_afterPID:%d   y_AFTERPID:%d",x ,y);
  Serial.println();

  float rpm_cmd[3];
  inverseKinematics(pid_vx, pid_vy, w, rpm_cmd);

//   float rpm_cmd[3];
//   inverseKinematics(x, y, w, rpm_cmd);

//  for (int i = 0; i < 3; i++) {
//    rpm_cmd[i] = constrain(rpm_cmd[i], -14000, 14000);
//    runMotor((int)rpm_cmd[i],pwmL_pin[i],pwmR_pin[i]);
//  }

   for (int i = 0; i < 3; i++) {
     error[i] = rpm_cmd[i] - rpm_rt[i];
     eDer[i] = (error[i] - lastError[i]) / dt;
     eInt[i] = eInt[i] + error[i] * dt;

     pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);

     //Serial.printf("pwm_pid:%d \n",pwm_pid[i]);
//     pwm_pid[i]=pwm_pid[i]%16383;
     pwm_pid[i]=pwm_pid[i]%16383;
     analogWrite(pwmR_pin[i], pwm_pid[i]>=0?pwm_pid[i]:0);
     analogWrite(pwmL_pin[i], pwm_pid[i]<=0?pwm_pid[i]*-1:0);

     lastError[i] = error[i];
//     Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
//     Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_cmd[i]);
   }

}

void setup(){
  pinMode(13,OUTPUT); digitalWrite(13,HIGH);
  Serial.begin(115200); delay(2000);
  analogWriteResolution(14);

  // === ETHERNET SETUP FROM CODE 1 ===
  byte arduino_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };
  #if defined(ARDUINO_TEENSY41)
  get_teensy_mac(arduino_mac);
  #endif

  IPAddress arduino_ip(192, 168, 1, 177);
  IPAddress agent_ip(192, 168, 1, 100);  // Using your agent IP from Code 1
  set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999);
  // === END ETHERNET SETUP ===

   while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);


  // === MICRO-ROS INITIALIZATION ===
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));  // Added args to match Code 1
  RCCHECK(rclc_node_init_default(&node, "teensy_ps4_node", "", &support));  // Fixed name

  // Initialize Joy message buffers (UPDATED)
  msg.axes.data = axes_buffer;
  msg.axes.size = 0;
  msg.axes.capacity = MAX_AXES;

  msg.buttons.data = buttons_buffer;
  msg.buttons.size = 0;
  msg.buttons.capacity = MAX_BUTTONS;

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),  // Fixed type
    "joy"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  motorTimer.begin(motor_update, 75*1000); // every 75ms
}

void input() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
//    Kp_vy = input.substring(0,3).toFloat();
//    Ki_vy = input.substring(3,9).toFloat();
//    Kd_vy = input.substring(9,15).toFloat();

    input.trim();

    int commaIndex = 0;
    int lastIndex = 0;
    float values[6];
    int valueIndex = 0;

    while ((commaIndex = input.indexOf(',', lastIndex)) != -1 && valueIndex < 6) {
      values[valueIndex] = input.substring(lastIndex, commaIndex).toFloat();
      lastIndex = commaIndex + 1;
      valueIndex++;
    }

    // Capture the last value after the final comma (or if no comma left)
    if (valueIndex < 6) {
      values[valueIndex] = input.substring(lastIndex).toFloat();
    }

    // Assign to your variables
    Kp_vy = values[0];
    Ki_vy = values[1];
    Kd_vy = values[2];
    Kp_vx = values[3];
    Ki_vx = values[4];
    Kd_vx = values[5];

//    Serial.printf("kp:%.2f  ki:%.2f  kd:%.2f",Kp_vy,Ki_vy,Kd_vy);
//    Serial.println();
//    Serial.printf("kp:%.2f  ki:%.2f  kd:%.2f",Kp_vx,Ki_vx,Kd_vx);
//    Serial.println();
  }
}

void loop(){
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
