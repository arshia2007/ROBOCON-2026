#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

//#include <Wire.h>
//#include <Adafruit_BNO055.h>
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
#include <Encoder.h>

//#define RADIUS 0.063      //in m  
//float ROBOT_RADIUS[3] = {0.28, 0.3, 0.3};

#define RADIUS 6.3                // in cm
float ROBOT_RADIUS[3] = {28, 30, 30};

int pwmL_pin[3] = {1, 22, 4};
int pwmR_pin[3] = {0, 23, 5};
//int max_rpm = 400;

Encoder m[3] = { Encoder(20, 21), Encoder(40, 41), Encoder(27, 26) };

// === MICRO-ROS VARIABLES ===
rcl_subscription_t tuning_subscriber;
rcl_publisher_t state_publisher;

std_msgs__msg__Float32MultiArray tuning_msg;  // From Python node - [ps4 + accl + pid constants]
std_msgs__msg__Float32MultiArray state_msg;   // To Python node - [actual and desired velocity]

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Message buffers
#define TUNING_ARRAY_SIZE 12   // [ps4_lx, ps4_ly, ps4_rx, ps4_ry, phone_ax, phone_ay, Kp_vx, Ki_vx, Kd_vx, Kp_vy, Ki_vy, Kd_vy]  
#define STATE_ARRAY_SIZE 4     // [vx, vy, pid_vx, pid_vy] 

float tuning_data[TUNING_ARRAY_SIZE];
float state_data[STATE_ARRAY_SIZE];

// === CONTROL VARIABLES ===
float ps4_lx = 0, ps4_ly = 0, ps4_rx = 0, ps4_ry = 0;
float phone_ax = 0, phone_ay = 0;
  
// VELOCITY PID - Initialize with default values
float Kp_vx = 0.0, Ki_vx = 0.0, Kd_vx = 0.0;
float Kp_vy = 0.0, Ki_vy = 0.0, Kd_vy = 0.0;

float dt = 0.075;
float error_vx, eDer_vx, eInt_vx, error_vy, eDer_vy, eInt_vy; 
float pid_vx, pid_vy;
float lastError_vx, lastError_vy;
float ax = 0, ay = 0;
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
volatile long count[3] = { 0, 0, 0 };
volatile long newPosition[3] = { 0, 0, 0 };
volatile float rpm_rt[3] = { 0, 0, 0 };
float cpr[] = {700.0, 1300.0, 700.0};

// Publishing timing
unsigned long last_publish_time = 0;
const unsigned long PUBLISH_INTERVAL = 50; // ms (20Hz to match Python)

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
  // Initialize tuning message (from Jetson)
  tuning_msg.data.data = tuning_data;
  tuning_msg.data.capacity = TUNING_ARRAY_SIZE;
  tuning_msg.data.size = 0;  // Will be set when message received
  
  // Initialize state message (to Jetson) 
  state_msg.data.data = state_data;
  state_msg.data.capacity = STATE_ARRAY_SIZE;
  state_msg.data.size = STATE_ARRAY_SIZE;
}

// === TUNING SUBSCRIPTION CALLBACK ===
void tuning_callback(const void* msgin) {
  const std_msgs__msg__Float32MultiArray* tuning_cmd = (const std_msgs__msg__Float32MultiArray*)msgin;
  
  if (tuning_cmd->data.size >= TUNING_ARRAY_SIZE) {
    // Extract data from Python node
    // PS4 and Phone data
    ps4_lx = tuning_cmd->data.data[0] * -375;   // PS4 Left Stick X
    ps4_ly = tuning_cmd->data.data[1] * 375;    // PS4 Left Stick Y  
    ps4_rx = tuning_cmd->data.data[2];          // PS4 Right Stick X
    ps4_ry = tuning_cmd->data.data[3];          // PS4 Right Stick Y
    phone_ax = tuning_cmd->data.data[4];        // Phone Accel X
    phone_ay = tuning_cmd->data.data[5];        // Phone Accel Y
    
    // Extract PID parameters
    Kp_vx = tuning_cmd->data.data[6];           // Kp for VX
    Ki_vx = tuning_cmd->data.data[7];           // Ki for VX
    Kd_vx = tuning_cmd->data.data[8];           // Kd for VX
    Kp_vy = tuning_cmd->data.data[9];           // Kp for VY
    Ki_vy = tuning_cmd->data.data[10];          // Ki for VY
    Kd_vy = tuning_cmd->data.data[11];          // Kd for VY
    
//    Serial.printf("📱 Received - PS4(LX:%.1f, LY:%.1f) Phone(AX:%.3f, AY:%.3f) ", 
//                  ps4_lx, ps4_ly, phone_ax, phone_ay);
//    Serial.printf("PID(VX: Kp=%.2f, Ki=%.2f, Kd=%.2f | VY: Kp=%.2f, Ki=%.2f, Kd=%.2f)", 
//                  Kp_vx, Ki_vx, Kd_vx, Kp_vy, Ki_vy, Kd_vy);
//    Serial.println();
  }
}

// === YOUR EXISTING FUNCTIONS ===
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


IntervalTimer motorTimer;

void motor_update(){
  for (int i = 0; i < 3; i++) {
    newPosition[i] = m[i].read();
    ::count[i] = abs(newPosition[i] - oldPosition[i]);
    rpm_rt[i] = ::count[i] / cpr[i] * 600 * 4.0 / 3;
    rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
    ::count[i] = 0;
    oldPosition[i] = newPosition[i];
  }

  // Get acceleration from IMU
//  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//  float ax = accel.x();
//  float ay = accel.y();
  ax = phone_ax;
  ay = phone_ay;

  vx += ax * dt*100;  //in cm/s
  vy += ay * dt*100;

  // Apply some damping to prevent infinite integration
  vx *= 0.95;
  vy *= 0.95;

  // === CHOOSE CONTROL INPUT SOURCE ===
  float target_vx, target_vy, target_w;
  
  target_vx = ps4_lx;        
  target_vy = ps4_ly;        
  target_w = ps4_rx;  
  

  // VELOCITY PID
  error_vx = target_vx - vx;
  eDer_vx = (error_vx - lastError_vx)/dt;
  eInt_vx = eInt_vx + error_vx * dt;
  pid_vx = Kp_vx * error_vx + Ki_vx * eInt_vx + Kd_vx * eDer_vx;
  lastError_vx = error_vx;

  error_vy = target_vy - vy;
  eDer_vy = (error_vy - lastError_vy)/dt;
  eInt_vy = eInt_vy + error_vy * dt;
  pid_vy = Kp_vy * error_vy + Ki_vy * eInt_vy + Kd_vy * eDer_vy;
  lastError_vy = error_vy;
  

  float rpm_cmd[3];
  inverseKinematics(pid_vy, pid_vx, target_w, rpm_cmd); 

  Serial.printf("setpoint_vx:%.2f, setpoint_vy:%.2f, actual_vx:%.2f, actual_vy:%.2f", 
                  target_vx, target_vy, vx, vy);
  Serial.println();

//   float rpm_cmd[3];
//   inverseKinematics(target_vx, target_vy, target_w, rpm_cmd);

  for (int i = 0; i < 3; i++) {
    error[i] = rpm_cmd[i] - rpm_rt[i];
    eDer[i] = (error[i] - lastError[i]) / dt;
    eInt[i] = eInt[i] + error[i] * dt;

    pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);
    pwm_pid[i] = constrain(pwm_pid[i], -16383, 16383);
    
    analogWrite(pwmR_pin[i], pwm_pid[i]>=0?pwm_pid[i]:0);
    analogWrite(pwmL_pin[i], pwm_pid[i]<=0?pwm_pid[i]*-1:0);

    lastError[i] = error[i];
//     Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
//     Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_cmd[i]);
  }

  // Update state data for publishing (4 elements as expected by Python)
  state_data[0] = vx;        // Actual velocity X
  state_data[1] = vy;        // Actual velocity Y  
  state_data[2] = pid_vx;    // PID output for VX
  state_data[3] = pid_vy;    // PID output for VY
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

  Serial.println("🤖 Teensy Tuning Control Node Starting...");

  /* Initialize the sensor */
  // Commented out for now as per your code
  // if (!bno.begin()) {
  //   Serial.print("❌ Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //   while (1);
  // }
  // delay(1000);

  // Micro-ROS initialization
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Change node name to match Python
  RCCHECK(rclc_node_init_default(&node, "tuning", "", &support));

  // Setup messages
  setup_messages();

  // Create subscriber for tuning commands from Python
  RCCHECK(rclc_subscription_init_default(
    &tuning_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "tuning_commands"));  // Match Python publisher topic

  // Create publisher for state (to Python)
  RCCHECK(rclc_publisher_init_default(
    &state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "teensy_state"));  // Match Python subscriber topic

  // Executor setup
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &tuning_subscriber, &tuning_msg, &tuning_callback, ON_NEW_DATA));

  motorTimer.begin(motor_update, 75*1000); // every 75ms
  
  Serial.println("✅ Teensy Tuning Node Ready!");
  Serial.println("📥 Listening: tuning_commands (12 floats)");
  Serial.println("📤 Publishing: teensy_state (4 floats)");
  Serial.println("🎮 Control Sources: PS4 sticks + Phone accelerometer");
  Serial.println("🔧 PID Parameters: Received via ROS from Python");
}

// === MAIN LOOP ===
void loop(){
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
  // Periodic publishing at ~20Hz to match Python
  unsigned long current_time = millis();
  if (current_time - last_publish_time >= PUBLISH_INTERVAL) {
    RCSOFTCHECK(rcl_publish(&state_publisher, &state_msg, NULL));
    
//    Serial.printf("vx:%.2f, vy:%.2f, pid_vx:%.2f, pid_vy:%.2f", 
//                  state_data[0], state_data[1], state_data[2], state_data[3]);
//    Serial.println();
    
    last_publish_time = current_time;
  }
}
