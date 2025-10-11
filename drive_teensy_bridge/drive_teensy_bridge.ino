#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joy.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <Wire.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
#include <Encoder.h>

#define RADIUS 6.3                // in cm
float ROBOT_RADIUS[3] = {28, 30, 30};

int pwmL_pin[3] = {1, 22, 4};
int pwmR_pin[3] = {0, 23, 5};
int max_rpm = 400;

Encoder m[3] = { Encoder(20, 21), Encoder(40, 41), Encoder(27, 26) };

// === MICRO-ROS VARIABLES ===
rcl_subscription_t subscriber;
rcl_publisher_t params_publisher;
rcl_publisher_t state_publisher;

sensor_msgs__msg__Joy msg;
std_msgs__msg__String params_msg;
std_msgs__msg__Float32MultiArray state_msg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Message buffers
#define MAX_AXES 10
#define MAX_BUTTONS 20
#define STATE_ARRAY_SIZE 6  // x, y, w, vx, vy, timestamp

float axes_buffer[MAX_AXES];
int32_t buttons_buffer[MAX_BUTTONS];
int32_t last_buttons[MAX_BUTTONS] = {0};
float state_data[STATE_ARRAY_SIZE];
char params_buffer[256];

// === CONTROL VARIABLES ===
int x = 0, y = 0, w = 0;

// VELOCITY PID - Initialize with default values
float Kp_vx = 0.0, Ki_vx = 0.0, Kd_vx = 0.0;
float Kp_vy = 0.0, Ki_vy = 0.0, Kd_vy = 0.0;

float dt = 0.075;
float error_vx, eDer_vx, eInt_vx, error_vy, eDer_vy, eInt_vy; 
float pid_vx, pid_vy;
float lastError_vx, lastError_vy;
float vx = 0, vy = 0;

// RPM PID (your existing variables)
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
volatile long count[3] = { 0, 0, 0 };
volatile long newPosition[3] = { 0, 0, 0 };
volatile float rpm_rt[3] = { 0, 0, 0 };
float cpr[]={700.0,1300.0,700.0};

// Publishing timing
unsigned long last_publish_time = 0;
const unsigned long PUBLISH_INTERVAL = 100; // ms

// === ETHERNET CONFIGURATION ===
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

// === MESSAGE INITIALIZATION ===
void setup_messages() {
  // Initialize Joy message buffers
  msg.axes.data = axes_buffer;
  msg.axes.size = 0;
  msg.axes.capacity = MAX_AXES;
  msg.buttons.data = buttons_buffer;
  msg.buttons.size = 0;
  msg.buttons.capacity = MAX_BUTTONS;

  // Initialize parameters message
  params_msg.data.data = params_buffer;
  params_msg.data.capacity = sizeof(params_buffer);
  params_msg.data.size = 0;

  // Initialize state message
  state_msg.data.data = state_data;
  state_msg.data.capacity = STATE_ARRAY_SIZE;
  state_msg.data.size = STATE_ARRAY_SIZE;
}

// === PUBLISHING FUNCTIONS ===
void publish_parameters() {
  // Create parameter string
  snprintf(params_buffer, sizeof(params_buffer), 
           "Kp_vx:%.3f,Ki_vx:%.3f,Kd_vx:%.3f,Kp_vy:%.3f,Ki_vy:%.3f,Kd_vy:%.3f",
           Kp_vx, Ki_vx, Kd_vx, Kp_vy, Ki_vy, Kd_vy);
  
  params_msg.data.size = strlen(params_buffer);
  
  RCSOFTCHECK(rcl_publish(&params_publisher, &params_msg, NULL));
  
  Serial.printf("Published params: %s", params_buffer);
  Serial.println();
}

void publish_state() {
  // Update state data array
  state_data[0] = (float)x;           // x command
  state_data[1] = (float)y;           // y command  
  state_data[2] = (float)w;           // w command
  state_data[3] = vx;                // actual vx
  state_data[4] = vy;                // actual vy
  state_data[5] = millis() / 1000.0; // timestamp in seconds
  
  RCSOFTCHECK(rcl_publish(&state_publisher, &state_msg, NULL));
  
  Serial.printf("Published state - x:%.1f, y:%.1f, w:%.1f, vx:%.2f, vy:%.2f", 
                state_data[0], state_data[1], state_data[2], state_data[3], state_data[4]);
  Serial.println();
}

// === PS4 SUBSCRIPTION CALLBACK ===
void subscription_callback(const void* msgin){
  const sensor_msgs__msg__Joy* joy_msg = (const sensor_msgs__msg__Joy*)msgin;

  // Process buttons
  for(size_t i=0; i<joy_msg->buttons.size; i++){
    int32_t current = joy_msg->buttons.data[i];
    if(current==1 && last_buttons[i]==0) {
      digitalWrite(13,!digitalRead(13));
      Serial.printf("Button %d pressed", i);
      Serial.println();
    }
    last_buttons[i]=current;
  }

  // Update control values from PS4
  // PS4 axes mapping:
  // axes[0] = left stick X
  // axes[1] = left stick Y  
  // axes[5] = R2 trigger
  y = joy_msg->axes.data[1] * 375;   // left stick Y for forward/back
  x = joy_msg->axes.data[0] * 375;   // left stick X for left/right
  w = joy_msg->axes.data[5] * -60;   // R2 for rotation
  
  if(abs(w)==255) w=0;
  
  Serial.printf("PS4 Received - x:%d  y:%d  w:%d", x, y, w);
  Serial.println();
}

// === YOUR EXISTING FUNCTIONS (mostly unchanged) ===
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
}

void runMotor(int pwm_val, int pwmLPin, int pwmRPin){
  analogWrite(pwmRPin, (pwm_val <= 0 ? -pwm_val : 0));
  analogWrite(pwmLPin, (pwm_val >= 0 ? pwm_val : 0));
}

void input() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
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

    if (valueIndex < 6) {
      values[valueIndex] = input.substring(lastIndex).toFloat();
    }

    // Update PID parameters
    Kp_vy = values[0];
    Ki_vy = values[1];
    Kd_vy = values[2];
    Kp_vx = values[3];
    Ki_vx = values[4];
    Kd_vx = values[5];

    Serial.printf("Updated PID - vy: Kp:%.2f Ki:%.2f Kd:%.2f", Kp_vy, Ki_vy, Kd_vy);
    Serial.println();
    Serial.printf("Updated PID - vx: Kp:%.2f Ki:%.2f Kd:%.2f", Kp_vx, Ki_vx, Kd_vx);
    Serial.println();
    
    // Publish updated parameters immediately
    publish_parameters();
  }
}

IntervalTimer motorTimer;

void motor_update(){
  input();
  
  // Your existing motor control code...
  for (int i = 0; i < 3; i++) {
    newPosition[i] = m[i].read();
    ::count[i] = abs(newPosition[i] - oldPosition[i]);
    rpm_rt[i] = ::count[i] / cpr[i] * 600 * 4.0 / 3;
    rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
    ::count[i] = 0;
    oldPosition[i] = newPosition[i];
  }

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float ax = accel.x();
  float ay = accel.y();

//  vx += ax * dt*100;
//  vy += ay * dt*100;

   vx = ax;
   vy = ay;

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

//  float rpm_cmd[3];
//  inverseKinematics(pid_vy, pid_vx, w, rpm_cmd); 

  float rpm_cmd[3];
  inverseKinematics(x, y, w, rpm_cmd); 

  for (int i = 0; i < 3; i++) {
    error[i] = rpm_cmd[i] - rpm_rt[i];
    eDer[i] = (error[i] - lastError[i]) / dt;
    eInt[i] = eInt[i] + error[i] * dt;

    pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);
    pwm_pid[i] = pwm_pid[i] % 16383;
    
    analogWrite(pwmR_pin[i], pwm_pid[i]>=0?pwm_pid[i]:0);
    analogWrite(pwmL_pin[i], pwm_pid[i]<=0?pwm_pid[i]*-1:0);

    lastError[i] = error[i];
  }
}

// === SETUP ===
void setup(){
  pinMode(13,OUTPUT); digitalWrite(13,HIGH);
  Serial.begin(115200); delay(2000);
  analogWriteResolution(14);

  // Ethernet setup
  byte arduino_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };
  #if defined(ARDUINO_TEENSY41)
  get_teensy_mac(arduino_mac);
  #endif

  IPAddress arduino_ip(192, 168, 1, 177);
  IPAddress agent_ip(192, 168, 1, 100);
  set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999);

  while (!Serial) delay(10);

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);

  // Micro-ROS initialization
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Change node name to avoid conflicts
  RCCHECK(rclc_node_init_default(&node, "teensy_control_node", "", &support));

  // Setup messages
  setup_messages();

  // Create subscriber for PS4 data - CHANGE TOPIC NAME
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "ps4_commands"));  // Changed from "joy" to "ps4_commands"

  // Create publishers for parameters and state
  RCCHECK(rclc_publisher_init_default(
    &params_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "teensy_parameters"));

  RCCHECK(rclc_publisher_init_default(
    &state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "teensy_state"));

  // Executor setup - increased to handle multiple subscribers/publishers
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  motorTimer.begin(motor_update, 75*1000); // every 75ms
  
  Serial.println("Teensy Micro-ROS Node Ready with Bidirectional Communication!");
}

// === MAIN LOOP ===
void loop(){
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
  // Periodic publishing
  unsigned long current_time = millis();
  if (current_time - last_publish_time >= PUBLISH_INTERVAL) {
    publish_parameters();
    publish_state();
    last_publish_time = current_time;
  }
}
