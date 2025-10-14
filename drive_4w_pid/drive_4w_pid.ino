#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joy.h>

#define RADIUS 6.3
#define ROBOT_LENGTH 52.5
#define ROBOT_WIDTH 48.5

#define MAX_AXES 10
#define MAX_BUTTONS 20
#define LED_PIN 13

// Motor pins (4 wheels)
int pwmL_pin[4] = {36,4,15,0};
int pwmR_pin[4] = {37,5,14,1};
int max_rpm = 400;

Encoder m[3] = { Encoder(20, 21), Encoder(40, 41), Encoder(27, 26) };

// RPM PID
volatile int pwm_pid[] = { 0, 0, 0, 0 };
volatile float rpm_sp[] = { 0, 0, 0, 0 };

volatile float kp[] = { 09.0, 09.0, 09.0, 09.0 };
volatile float ki[] = { 165.0, 165.0, 165.0, 165.0 };
volatile float kd[] = { 00.50, 00.50, 00.50, 00.50 };

float error[] = { 0, 0, 0, 0 };
float eInt[] = { 0, 0, 0, 0 };
float eDer[] = { 0, 0, 0, 0 };
float lastError[] = { 0, 0, 0, 0 };

volatile long oldPosition[3] = { 0, 0, 0, 0 };
volatile long count[3] = { 0, 0, 0, 0 };  // use volatile for shared variables
volatile long newPosition[3] = { 0, 0, 0, 0 };

volatile float rpm_rt[3] = { 0, 0, 0, 0 };
float cpr[]={700.0,700.0,1300.0, 700.0};

// Buffers for Joy
float axes_buffer[MAX_AXES];
int32_t buttons_buffer[MAX_BUTTONS];
int32_t last_buttons[MAX_BUTTONS] = {0};

// micro-ROS
rcl_subscription_t subscriber;
sensor_msgs_msg_Joy joy_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

IntervalTimer motorTimer;

// Joystick control values
float vx = 0, vy = 0, omega = 0;

// Safety macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){}}

// ----------------- Inverse Kinematics -----------------
void inverseKinematics(float vx, float vy, float omega, float* rpms) {
    const float R = (ROBOT_LENGTH + ROBOT_WIDTH)/2.0;

    float wFL = ( vx - vy - R*omega ) / RADIUS;
    float wFR = ( vx + vy + R*omega ) / RADIUS;
    float wRL = ( vx + vy - R*omega ) / RADIUS;
    float wRR = ( vx - vy + R*omega ) / RADIUS;

    rpms[0] = wFL * 60.0 / (2 * PI);
    rpms[1] = wFR * 60.0 / (2 * PI);
    rpms[2] = wRL * 60.0 / (2 * PI);
    rpms[3] = wRR * 60.0 / (2 * PI);
}

// ----------------- Motor Control -----------------
void runMotor(int pwm_val, int pwmLPin, int pwmRPin){
    analogWrite(pwmRPin, (pwm_val <= 0 ? -pwm_val : 0));
    analogWrite(pwmLPin, (pwm_val >= 0 ? pwm_val : 0));
}

void motor_update(){

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

  float rpm_cmd[4];
  inverseKinematics(vx, vy, omega, rpm_cmd);
    // for(int i=0;i<4;i++){
    //     rpm_cmd[i] = constrain(rpm_cmd[i], -max_rpm, max_rpm);
    //     runMotor((int)rpm_cmd[i], pwmL_pin[i], pwmR_pin[i]);
    // }

    for (int i = 0; i < 4; i++) {
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
    Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
    Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_cmd[i]);
   }
}

// ----------------- Joy Callback -----------------
void subscription_callback(const void* msgin){
    const sensor_msgs_msgJoy* joy = (const sensor_msgsmsg_Joy*)msgin;

    for(size_t i=0; i<joy->buttons.size; i++){
        int32_t current = joy->buttons.data[i];
        if(current==1 && last_buttons[i]==0) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        last_buttons[i] = current;
    }

    // Map joystick to velocities
    vy = joy->axes.data[1] * 100;   // forward/backward
    vx = joy->axes.data[0] * 100;   // left/right
    omega = joy->axes.data[5] * -100; // rotation

    // Debug
    Serial.printf("vx: %.2f vy: %.2f omega: %.2f\n", vx, vy, omega);
}

// ----------------- Setup -----------------
#if defined(ARDUINO_TEENSY41)
void get_teensy_mac(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
}
#endif

void setup() {
    Serial.begin(115200);
    delay(2000);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    analogWriteResolution(14);

    // ---------- Ethernet setup ----------
    byte arduino_mac[] = {0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF};
    #if defined(ARDUINO_TEENSY41)
    get_teensy_mac(arduino_mac);
    #endif
    IPAddress arduino_ip(192,168,1,177);
    IPAddress agent_ip(192,168,1,100);
    set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999);

    // ---------- micro-ROS init ----------
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "teensy_ps4_node", "", &support));

    // Joy buffers
    joy_msg.axes.data = axes_buffer; joy_msg.axes.size = 0; joy_msg.axes.capacity = MAX_AXES;
    joy_msg.buttons.data = buttons_buffer; joy_msg.buttons.size = 0; joy_msg.buttons.capacity = MAX_BUTTONS;

    // Subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        "joy"));

    // Executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &joy_msg, &subscription_callback, ON_NEW_DATA));

    // Motor timer (50ms)
    motorTimer.begin(motor_update, 50000);
}

// ----------------- Loop -----------------
void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}