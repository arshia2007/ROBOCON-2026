// Move ClientServerEthernet.h to a subfolder of your Arduino/libraries/ directory.
#include <ClientServerEthernet.h>
#include <Encoder.h>
#include <VescUart.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <queue>

double easeInOutExpo(double x) {
  if (x == 0.0) {
    return 0.0;
  } else if (x == 1.0) {
    return 1.0;
  } else if (x < 0.5) {
    return pow(2.0, 20.0 * x - 10.0) / 2.0;
  } else {
    return (2.0 - pow(2.0, -20.0 * x + 10.0)) / 2.0;
  }
}


IntervalTimer targetLocking;

bool flag_bldc_dribble=false;
bool flag_bldc=false;
bool data_update=true;
bool is_flag_bldc=false;
bool is_r1_pressed=false;


VescUart UART;
IntervalTimer pidTimer;

int pwmL_pin[3] = { 0, 5, 12};
int pwmR_pin[3] = { 1, 4, 13};

int drib_pwmL=2;
int drib_pwmR=3;

int feed_pwmL=15;
int feed_pwmR=14;

int drib_in=3; // pca
int drib_out=2; // pca

int target_pwmL=22;
int target_pwmR=23;
int limit_switch=9;
int switchState = 1;

int blue = 28;
int red = 29;
int green = 8;

int bldc_RPM=0;
Encoder m[3] = { Encoder(21,20), Encoder(27,26), Encoder(41,40) };

volatile float rpm_rt[3] = { 0, 0, 0 };
// float cpr[]={1300.0,1300.0,1300.0};
float cpr[]={1300.0,700.0,1300.0};


int duty_cycle = 100;                           //in percentage
// int max_pwm = (int)(duty_cycle / 100.0 * res);  //6v--250rpm
int max_rpm = 500;

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
    int16_t bldc_rpm;
};
#pragma pack(pop) // restore previous alignment


ControllerData jetdata; // Struct instance to hold incoming controller data

ClientServerEthernet<ControllerData> con; // Instance of the ClientServerEthernet class templated with ControllerData
uint32_t checkTimer = millis();

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void runMotor(int pwm_val, int pwmLPin, int pwmRPin)
{
  analogWrite(pwmLPin, (pwm_val <= 0 ? pwm_val*-1 : 0));
  analogWrite(pwmRPin, (pwm_val >= 0 ? pwm_val : 0));
}
bool shoot_flag=false;

void target_locking()
{ 
  
}



void setup() {
  Serial.begin(115200);

  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);


  analogWriteResolution(14);
  
  pinMode(limit_switch,INPUT_PULLUP);
  
  // vector<int> client_ip = {192, 168, 1, 101}; // IP address of this device (client)
  // vector<int> server_ip = {192, 168, 1, 100}; // IP address of the server to communicate with
  // vector<int> subnet_mask = {255, 255, 255, 0}; // Subnet mask for the network

  vector<int> client_ip = {192, 168, 1, 101}; // IP address of this device (client)
  vector<int> server_ip = {192, 168, 1, 102}; // IP address of the server to communicate with
  vector<int> subnet_mask = {255, 255, 255, 0}; // Subnet mask for the network

  // Initialize the Ethernet client-server connection with IPs, subnet, and a pointer to the data structure
  con = ClientServerEthernet<ControllerData>(client_ip, subnet_mask, server_ip, &jetdata);
    pidTimer.begin(pid, 75000);
    targetLocking.begin(target_locking,50000);

      Serial8.begin(115200);

  // while (!Serial8) { ; }
  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial8);



}

volatile long oldPosition[3] = { 0, 0, 0 };
int ledState = LOW;
volatile long count[3] = { 0, 0, 0 };  // use volatile for shared variables
volatile long newPosition[3] = { 0, 0, 0 };

volatile int pwm_pid[] = { 0, 0, 0 };
volatile float rpm_sp[] = { 0, 0, 0 };


volatile float kp[] = { 09.0, 09.0, 09.0 };
volatile float ki[] = { 165.0, 165.0, 165.0 };
volatile float kd[] = { 00.50, 00.50, 00.50 };

float error[] = { 0, 0, 0 };
float eInt[] = { 0, 0, 0 };
float eDer[] = { 0, 0, 0 };
float lastError[] = { 0, 0, 0 };
    int y=0;
    int x=0;
    int w = 0;

queue<int> rpm;
queue<int> pwm;
int sum_rpm;
int sum_pwm;

void pid() {


  if(jetdata.touch_button){
    if(abs(jetdata.turn_pwm)<200){
      runMotor(-jetdata.turn_pwm*64,target_pwmL,target_pwmR);}

    if(jetdata.bldc_rpm<0 && !is_r1_pressed){
      bldc_RPM=1500;}
  }
  else if(jetdata.bldc_rpm<0)
  {
    is_r1_pressed=false;
    bldc_RPM=0;
    is_flag_bldc=false;
  }
  else{
    is_r1_pressed=false;
    is_flag_bldc=false;
      runMotor(0*64,target_pwmL,target_pwmR);
  }

    if(jetdata.touch_button && !is_flag_bldc)
  {
    flag_bldc=true;
    is_flag_bldc=true;
  }
  



  // ii++;
    // con.getData(true);
// runMotor(-1*255*64,feed_pwmL,feed_pwmR);
for (int i = 0; i < 3; i++) {
    newPosition[i] = m[i].read();
    ::count[i] = abs(newPosition[i] - oldPosition[i]);
    // count=newPosition<oldPosition?-count:count;
    rpm_rt[i] = ::count[i] / cpr[i] * 600 * 4.0 / 3;
    rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
      // Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
    ::count[i] = 0;
    oldPosition[i] = newPosition[i];
  }
  // if(ii%10==0)
    // Serial.printf("\n");
switchState=digitalRead(limit_switch);
if(switchState==LOW){
  runMotor(0, target_pwmL, target_pwmR);
} 

if(data_update){


  int psAxisX = 0;
    int psAxisY = 0;
    if (jetdata.axis[0] < 120)
      psAxisX = map(jetdata.axis[0], 120, 0, 0, -255);

    else if (jetdata.axis[0] > 135)
      psAxisX = map(jetdata.axis[0], 135, 255, 0, 255);
    else
      psAxisX = 0;

    if (jetdata.axis[1] > 135)
      psAxisY = map(jetdata.axis[1], 135, 255, 0, -255);

    else if (jetdata.axis[1] < 120)
      psAxisY = map(jetdata.axis[1], 120, 0, 0, 255);
    else
      psAxisY = 0;
    if (jetdata.axis[2] > 135)
      w = map(jetdata.axis[2], 135, 255, 0, 255);

    else if (jetdata.axis[2] < 120)
      w = map(jetdata.axis[2], 120, 0, 0, -255);
    else
      w=0;
    
    // else
    // if(jetdata.r2)
    //   w = jetdata.r2;
    // else
    //   w = -1*jetdata.l2;

    y = psAxisY;
    x = psAxisX;

    // Serial.print(x);
    // Serial.print("   ok ");
    // Serial.print(y);
    // Serial.println();
    // x=0;
    // Serial.print("Rotation:");
    // Serial.println(w);
}
    // Serial.println(bldc_RPM);
    // Serial.println(flag_bldc);
    rpm_sp[0] = map(x + 0.22*w, -175, 175, max_rpm, -max_rpm);
    rpm_sp[1] = map(-0.5 * x - 0.852 * y + 0.22*w, -175, 175, max_rpm, -max_rpm);
    rpm_sp[2] = map(-0.5 * x + 0.866 * y + 0.22*w, -175, 175, max_rpm, -max_rpm);

    for (int i = 0; i < 3; i++) {
      Serial.printf("RPM_%d_input:%0.2f  ", i + 1, rpm_sp[i]);
    }
    //~~this block of code is to take the input from the ps4 controller




    for (int i = 0; i < 3; i++) {
    error[i] = rpm_sp[i] - rpm_rt[i];
    eDer[i] = (error[i] - lastError[i]) / 0.075;
    eInt[i] = eInt[i] + error[i] * 0.075;

    pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);
    //Serial.printf("pwm_pid:%d ",pwm_pid[i]);
    // pwm_pid[i]=map(pwm_pid[i],-16383,16383,-pwm_18,pwm_18);
    //Serial.printf("pwm_pid:%d \n",pwm_pid[i]);
    pwm_pid[i]=pwm_pid[i]%16383;
    analogWrite(pwmR_pin[i], pwm_pid[i]>=0?pwm_pid[i]:0);
    analogWrite(pwmL_pin[i], pwm_pid[i]<=0?pwm_pid[i]*-1:0);

    lastError[i] = error[i];
    Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_sp[i]);
  }
  if(flag_bldc_dribble){
        UART.setRPM(700*7);}
  else if(flag_bldc)
    {
        UART.setRPM(bldc_RPM*7);
    }
}



void loop() {
  // Serial.println("ok");
  // if (rpm.size() >= 10){
  //   sum_rpm -= rpm.front();
  //   sum_pwm -= pwm.front();
  //   rpm.pop();
  //   pwm.pop();
  // }

  // if (sum_rpm == 0 && sum_pwm == 0){      // not detected
  //     digitalWrite(red, LOW);
  //     digitalWrite(blue, HIGH);
  //     digitalWrite(green, HIGH);
  //   // Serial.println("red");
  // }else if (sum_rpm != 0 && sum_pwm != 0){  // detected NOT aligned
  //     digitalWrite(red, HIGH);
  //     digitalWrite(blue, LOW);
  //     digitalWrite(green, HIGH);
  //   // Serial.println("blue");
  // }else if (sum_rpm != 0 && sum_pwm == 0){    // aligned
  //     digitalWrite(red, HIGH);
  //     digitalWrite(blue, HIGH);
  //     digitalWrite(green, LOW);
  //   // Serial.println("green");
  // }

  con.MaintainConnection(false);
  con.getData();
  // Serial.printf("%d %d %d %d %d %d %d %d %d %d %d %d %d ",);
  // Serial.println(switchState);

  // switchState=digitalRead(limit_switch);
 
  // if(jetdata.l2>10)
  // {
  //   runMotor(jetdata.l2*64*0.8, target_pwmL, target_pwmR);  
  // }
  // else if(jetdata.r2>10 && switchState!=LOW)
  // {
  //   runMotor(-1*jetdata.r2*64*0.8, target_pwmL, target_pwmR);
  // }
  // else{
  //   runMotor(0, target_pwmL, target_pwmR);
  // }
  


  // if(jetdata.circle==1)
  // {

  //   while(switchState!=LOW){
  //     runMotor(-1*200*64*0.8, target_pwmL, target_pwmR); 
  //     switchState=digitalRead(limit_switch);
  //   }
  //   runMotor(0, target_pwmL, target_pwmR);


  //   int lastTime=0;
  //   flag_bldc_dribble=true;
  //   delay(200);
    
  //   // lastTime=millis();
  //   // while(millis()-lastTime<200)
  //   // {
  //   //     con.getData();      
  //   // }

  //   runMotor(-1*255*64,feed_pwmL,feed_pwmR);
  //   delay(900);  

  //   data_update=false;
  //   double yy=0;
  //   for(int i=1;i<=150;i++)
  //   {
  //     // Serial.println("Drive working");
  //     yy=mapDouble(i,0,100.0,0.458,1.0);
  //     yy=easeInOutExpo(yy);
  //     y=1*int(mapDouble(yy,0.3,1.0,0,110));
  //     // x=10;
  //     Serial.println(yy);
  //     delayMicroseconds(12000);
  //   }
  //   // y=-65;
  //   // delay(750);
  //   // y=0;
  //   data_update=true;    
  //   flag_bldc_dribble=false;
  //   runMotor(0*64,feed_pwmL,feed_pwmR);
  // }

  // if(jetdata.triangle==1)
  // {
  //   runMotor(-1*255*64,feed_pwmL,feed_pwmR);
    

  // }
  // if(jetdata.cross)
  // {
  //     runMotor(1*255*64,feed_pwmL,feed_pwmR);
      
  // }
  // if(jetdata.square)
  // {
  //     UART.setRPM(0);
  //     runMotor(0*64,feed_pwmL,feed_pwmR);
  // }

  //   if(jetdata.r1 && jetdata.bldc_rpm!=0)
  // {
  //   is_r1_pressed=true;
  //   if(jetdata.bldc_rpm>0){
  //   flag_bldc=true;
  //   runMotor(0,drib_pwmL,drib_pwmR);
  //   bldc_RPM=jetdata.bldc_rpm;

  //   delay(3000);
  //   runMotor(-255*64,feed_pwmL,feed_pwmR);
  //   delay(3000);
  //   flag_bldc=false;
  //   // runMotor(0*64,drib_pwmL,drib_pwmR);
  //   runMotor(0*64,feed_pwmL,feed_pwmR);
  //   // flag_bldc=0;
  //   bldc_RPM=0;
  //   }
  //   else

  //   {
  //     bldc_RPM=-1*jetdata.bldc_rpm;
  //   runMotor(-1*255*64,feed_pwmL,feed_pwmR);
  //   delay(2000);
  //   flag_bldc=false;
  //   is_flag_bldc=true;
  //   runMotor(0*255*64,feed_pwmL,feed_pwmR);
  //   bldc_RPM=0;
  

  //   }


  // }
  // if(jetdata.l1)
  // {
  //   UART.setRPM(1500*7);
  // }
  




  delay(10);
}