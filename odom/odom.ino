// Move ClientServerEthernet.h to a subfolder of your Arduino/libraries/ directory.
#include <ClientServerEthernet.h>
#include <Encoder.h>
#include <VescUart.h>
bool data_update=true;
VescUart UART;
IntervalTimer pidTimer;

int store_rpm=0;

int pwmL_pin[3] = { 2, 7, 12 };
int pwmR_pin[3] = { 3, 6, 13 };

Encoder m[3] = { Encoder(21,20), Encoder(26,27), Encoder(40,41) };

// deadwheels
// a,b: {31,30; 32,33; 29,28}
volatile float rpm_rt[3] = { 0, 0, 0 };


int duty_cycle = 100;                           //in percentage
// int max_pwm = (int)(duty_cycle / 100.0 * res);  //6v--250rpm
int max_rpm = 300;

// int ii=0;
// <<<< IMPORTANT ----

// Ensure the struct is packed with no padding between members.
// This is important for consistent memory layout, especially when sending data over serial or network.
// #pragma pack(1) → No padding (tightest packing).

// int a = 1000; -> size may vary (typically 4 bytes) across systems and compilers
// int16_t b = 1000; -> guaranteed to be 2 bytes across all platforms

// ---- IMPORTANT >>>>

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
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);


  //  for (int i = 0; i < 3; i++) 
  // {
  //   // analogWriteFrequency(pwmL_pin[i], 9000);
  //   // pinMode(pwmR_pin[i], OUTPUT);
  // }
  analogWriteResolution(14);
  
  vector<int> client_ip = {192, 168, 1, 101}; // IP address of this device (client)
  vector<int> server_ip = {192, 168, 1, 102}; // IP address of the server to communicate with
  vector<int> subnet_mask = {255, 255, 255, 0}; // Subnet mask for the network

  // Initialize the Ethernet client-server connection with IPs, subnet, and a pointer to the data structure
  con = ClientServerEthernet<ControllerData>(client_ip, subnet_mask, server_ip, &jetdata);
  pidTimer.begin(pid, 75000);


  Serial8.begin(115200);

  while (!Serial8) { ; }
  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial8);



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
      psAxisY = map(jetdata.axis[1], 135, 255, 0, -255);

    else if (jetdata.axis[1] < 120)
      psAxisY = map(jetdata.axis[1], 125, 0, 0, 255);
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
    
  con.MaintainConnection(false);
  con.getData();

}