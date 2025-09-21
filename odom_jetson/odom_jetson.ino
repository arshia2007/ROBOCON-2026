// Move ClientServerEthernet.h to a subfolder of your Arduino/libraries/ directory.
#include <ClientServerEthernet.h>
#include <Encoder.h>
#include <math.h>

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
const float WHEEL_DIAMETER = 5.82;               //5.7 cm
const float TICKS_PER_REV_encx = 2330.0;
const float TICKS_PER_REV_ency = 2330.0;

const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

const float CX = WHEEL_CIRCUMFERENCE / TICKS_PER_REV_encx; // cm per tick
const float CY = WHEEL_CIRCUMFERENCE / TICKS_PER_REV_ency;

// === Computed Geometry ===
float DIST_LR = 0.0;
float FRONT_OFFSET = 0.0;

// === Odometry State ===
volatile float globalX = 0.0;
volatile float globalY = 0.0;
volatile float globalTheta = 0;

// === Tick Deltas (global for performance) ===
volatile long deltaLeft = 0;
volatile long deltaRight = 0;
volatile long deltaFront = 0;

bool data_update=true;

IntervalTimer pidTimer;

int pwmL_pin[3] = { 2, 7, 12 };
int pwmR_pin[3] = { 3, 6, 13 };

Encoder m[3] = { Encoder(21,20), Encoder(26,27), Encoder(40,41) };

volatile float rpm_rt[3] = { 0, 0, 0 };


int max_rpm = 300;

// int ii=0;
// <<<< IMPORTANT ----

// Ensure the struct is packed with no padding between members.
// This is important for consistent memory layout, especially when sending data over serial or network.
// #pragma pack(1) → No padding (tightest packing).

// int a = 1000; -> size may vary (typically 4 bytes) across systems and compilers
// int16_t b = 1000; -> guaranteed to be 2 bytes across all platforms

// ---- IMPORTANT >>>>
#pragma pack(push, 1)
struct OdometryData {
    float x;
    float y;
    float theta;
};
#pragma pack(pop)

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


// ControllerData jetdata; // Struct instance to hold incoming controller data
ControllerData jetdata;
OdometryData odomData;  // Add this
IntervalTimer odomSendTimer;  // Add this

// ClientServerEthernet<ControllerData> con; // Instance of the ClientServerEthernet class templated with ControllerData

ClientServerEthernet<ControllerData, OdometryData> con;  // Modified

void sendOdomData() {  // Add this function
    odomData.x = globalX;
    odomData.y = globalY;
    odomData.theta = globalTheta;
 
    con.sendData(&odomData, false);
}

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
  float midTheta = globalTheta + deltaTheta ;

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

  // Normalize theta to [-PI, PI]
  if (globalTheta > PI) globalTheta -= 2 * PI;
  if (globalTheta < -PI) globalTheta += 2 * PI;

  Serial.print("X: "); Serial.print(globalX, 2);
  Serial.print(" | Y: "); Serial.print(globalY, 2);
  Serial.print(" | θ (deg): "); Serial.println(globalTheta * 180.0 / PI, 2);

  // Serial.print(globalX);
  // Serial.print(",");
  // Serial.print(globalY);
  // Serial.print(",");
  // Serial.println(globalTheta * 180.0 / PI, 2);
}


void setup() {
  Serial.begin(9600);
  // pinMode(13,OUTPUT);
  // digitalWrite(13,HIGH);


  //  for (int i = 0; i < 3; i++) 
  // {
  //   // analogWriteFrequency(pwmL_pin[i], 9000);
  //   // pinMode(pwmR_pin[i], OUTPUT);
  // }
  analogWriteResolution(14);
  
  vector<int> client_ip = {192, 168, 1, 101}; // IP address of this device (client)
  vector<int> server_ip = {192, 168, 1, 100}; // IP address of the server to communicate with
  vector<int> subnet_mask = {255, 255, 255, 0}; // Subnet mask for the network

  // Initialize the Ethernet client-server connection with IPs, subnet, and a pointer to the data structure
  con = ClientServerEthernet<ControllerData, OdometryData>(client_ip, subnet_mask, server_ip, &jetdata);
  DIST_LR = sqrt(pow(enc_left.x - enc_right.x, 2) + pow(enc_left.y - enc_right.y, 2));
  FRONT_OFFSET = enc_front.x;  // X offset of front encoder from center

  // Serial.println("Odometry system initialized.");
  pidTimer.begin(pid, 75000);
  odomSendTimer.begin(sendOdomData, 100000); 
}

volatile long oldPosition[3] = { 0, 0, 0 };
int ledState = LOW;
volatile long count[3] = { 0, 0, 0 };  // use volatile for shared variables
volatile long newPosition[3] = { 0, 0, 0 };

volatile int pwm_pid[] = { 0, 0, 0 };
volatile float rpm_sp[] = { 0, 0, 0 };


volatile float kp[] = { 9, 9, 9 };
volatile float ki[] = { 165.0, 165.0, 165.0 };
volatile float kd[] = { 00.50, 00.50, 00.50 };

// volatile float kp[] = { 40, 40, 40 };
// volatile float ki[] = { 165.0, 165.0, 165.0 };
// volatile float kd[] = { 00.50, 00.50, 00.50 };

float error[] = { 0, 0, 0 };
float eInt[] = { 0, 0, 0 };
float eDer[] = { 0, 0, 0 };
float lastError[] = { 0, 0, 0 };
float cpr[]={1300.0,700.0,700.0};
int y=0;
int x=0;
int w = 0;



void pid() {
  // ii++;
  // con.getData(true);
  // Odom();
  for (int i = 0; i < 3; i++) {
    newPosition[i] = m[i].read();
    ::count[i] = abs(newPosition[i] - oldPosition[i]);
    // count=newPosition<oldPosition?-count:count;
    rpm_rt[i] = ::count[i] / cpr[i]* 600 * 4.0 / 3;
    rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
      // Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
    ::count[i] = 0;
    oldPosition[i] = newPosition[i];
  }


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
      psAxisY = map(jetdata.axis[1], 135, 255, 0, 255);

    else if (jetdata.axis[1] < 120)
      psAxisY = map(jetdata.axis[1], 125, 0, 0, -255);
    else
      psAxisY = 0;
      // if (jetdata.axis[2] > 135)
      //   w = map(jetdata.axis[2], 135, 255, 0, 255);

      // else if (jetdata.axis[2] < 120)
      //   w = map(jetdata.axis[2], 125, 0, 0, -255);
      // else
    if (jetdata.axis[2] > 135)
      w = map(jetdata.axis[2], 135, 255, 0, 255);

    else if (jetdata.axis[2] < 120)
      w = map(jetdata.axis[2], 125, 0, 0, -255);
    else
      w=0;

    w=w*0.83;

    y = psAxisY;
    x = psAxisX;

      // Serial.print(x);
      // Serial.print("   ok ");
      // Serial.print(y);
      // Serial.println();
      // x=0;
  }

  rpm_sp[0] = map(x + 0.3*w, -175, 175, max_rpm, -max_rpm);
  rpm_sp[1] = map(-0.5 * x - 0.852 * y + 0.3*w, -175, 175, max_rpm, -max_rpm);
  rpm_sp[2] = map(-0.5 * x + 0.860 * y + 0.3*w, -175, 175, max_rpm, -max_rpm);

  for (int i = 0; i < 3; i++) {
    error[i] = rpm_sp[i] - rpm_rt[i];
    eDer[i] = (error[i] - lastError[i]) / 0.075;
    eInt[i] = eInt[i] + error[i] * 0.075;

    pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);
    // Serial.printf("pwm_pid:%d ",pwm_pid[i]);
    // pwm_pid[i]=map(pwm_pid[i],-16383,16383,-pwm_18,pwm_18);
    //Serial.printf("pwm_pid:%d \n",pwm_pid[i]);
    pwm_pid[i]=pwm_pid[i]%16383;
    analogWrite(pwmR_pin[i], pwm_pid[i]>=0?pwm_pid[i]:0);
    analogWrite(pwmL_pin[i], pwm_pid[i]<=0?pwm_pid[i]*-1:0);

    lastError[i] = error[i];
    // Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_sp[i]);
  }


}



void loop() {
  Serial.println("ok");
    
  con.MaintainConnection(false);
  con.getData();
  Odom();

}