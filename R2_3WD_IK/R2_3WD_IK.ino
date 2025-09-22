// Move ClientServerEthernet.h to a subfolder of your Arduino/libraries/ directory.
#include <ClientServerEthernet.h>
#include <Encoder.h>
#include <VescUart.h>
#include <queue>
// #include <cmath>  

bool flag_bldc_dribble=false;
bool flag_bldc=false;
int bldc_RPM=0;
bool data_update=true;
bool is_flag_bldc=false;
bool is_r1_pressed=false;

VescUart UART;
IntervalTimer pidTimer;

int store_rpm=0;

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
// int pwmL_pin[3] = { 6, 4, 12 };
// int pwmR_pin[3] = { 7, 5, 13 };

int pwmL_pin[3] = { 0, 4, 12 };
int pwmR_pin[3] = { 1, 5, 13 };

int drib_pwm=10;
int drib_dir=11;

int feed_pwmL=22;
int feed_pwmR=23;

int rot_pwm=36;
int rot_dir=37;


int blue = 28;
int red = 29;
int green = 8;


int bldc_rpm=0;
Encoder m[3] = { Encoder(21,20), Encoder(26,27), Encoder(41,40) };

volatile float rpm_rt[3] = { 0, 0, 0 };


int duty_cycle = 100;                           //in percentage
// int max_pwm = (int)(duty_cycle / 100.0 * res);  //6v--250rpm
int max_rpm = 12000;

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

  pinMode(drib_dir,OUTPUT);
  // pinMode(feed_pwmR,OUTPUT);
  pinMode(rot_dir,OUTPUT);

  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);

  digitalWrite(red, 255);
  digitalWrite(blue, 255);
  digitalWrite(green, 255);

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
float cpr[]={1300.0,1300.0,1300.0};
int y=0;
int x=0;
int w = 0;


queue<int> rpm;
queue<int> pwm;
int sum_rpm;
int sum_pwm;
int bldc_rpm_;
int pwm_;




void pid() {





  // ii++;
  // con.getData(true);
//   for (int i = 0; i < 3; i++) {
//   newPosition[i] = m[i].read();
//   ::count[i] = abs(newPosition[i] - oldPosition[i]);
//   // count=newPosition<oldPosition?-count:count;
//   rpm_rt[i] = ::count[i] / cpr[i]* 600 * 4.0 / 3;
//   rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
//     // Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
//   ::count[i] = 0;
//   oldPosition[i] = newPosition[i];
// }
  // if(ii%10==0)
    // Serial.printf("\n");

Serial.println(bldc_RPM);
Serial.println(flag_bldc);

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
     


    // auto align part no
    if(jetdata.touch_button){

      if(jetdata.bldc_pwm<0 && !is_r1_pressed)
         bldc_RPM=800;
    w = jetdata.turn_pwm;
    psAxisY = 0;
    psAxisX = 0;
    Serial.printf("x:%d  y:%d  w:%d",x,y,w);
  Serial.println();
  }
  else if(jetdata.bldc_pwm<0)
  {
    is_r1_pressed=false;
    bldc_RPM=0;
    is_flag_bldc=false;

  }
  else
  {
    is_r1_pressed=false;
    is_flag_bldc=false;
  }
// Serial.println(bldc_RPM);
  if(jetdata.touch_button && !is_flag_bldc)
  {
    flag_bldc=true;
    is_flag_bldc=true;
  }




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
  Serial.printf("RPM_%d_input:%0.2f  ", i + 1, rpm_sp[i]);
}
    //~~this block of code is to take the input from the ps4 controller




    for (int i = 0; i < 3; i++) {
      // error[i] = rpm_sp[i] - rpm_rt[i];
      // eDer[i] = (error[i] - lastError[i]) / 0.075;
      // eInt[i] = eInt[i] + error[i] * 0.075;

      // pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);
      // Serial.printf("pwm_pid:%d ",pwm_pid[i]);
      // pwm_pid[i]=map(pwm_pid[i],-16383,16383,-pwm_18,pwm_18);
      //Serial.printf("pwm_pid:%d \n",pwm_pid[i]);
      // pwm_pid[i]=pwm_pid[i]%16383;
      rpm_sp[i] = constrain(rpm_sp[i], -16383, 16383);
      analogWrite(pwmR_pin[i], rpm_sp[i]>=0?rpm_sp[i]:0);
      analogWrite(pwmL_pin[i], rpm_sp[i]<=0?rpm_sp[i]*-1:0);

      // lastError[i] = error[i];
      // Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_sp[i]);
    }
  if(flag_bldc_dribble){
        UART.setRPM(850*7);
  }
  else if(flag_bldc)
  {
    UART.setRPM(bldc_RPM*7);
  }
  // if(flag_bldc_pass){
  //   // UART.setRPM(jetdata.bldc_pwm*7);
  //   int orpm=0;
  //   int nrpm=jetdata.bldc_pwm*7;
   
  // else{

  //         UART.setRPM(0);

  // }

// Serial.println();
  }

void runMotor(int pwm_val, int pwmLPin, int pwmRPin)
{
  analogWrite(pwmLPin, (pwm_val <= 0 ? pwm_val*-1 : 0));
  analogWrite(pwmRPin, (pwm_val >= 0 ? pwm_val : 0));
}
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void loop() {
    
  con.MaintainConnection(false);
  con.getData(true);

        if (rpm.size() >= 10){
      sum_rpm -= rpm.front();
      sum_pwm -= pwm.front();
      rpm.pop();
      pwm.pop();
    }
  
    bldc_rpm_ = jetdata.bldc_pwm; 
    if(bldc_rpm_ !=0){
      store_rpm=bldc_rpm_;
    }
    rpm.push(bldc_rpm_);
    sum_rpm += bldc_rpm_;

    pwm_ = abs(jetdata.turn_pwm); 
    pwm.push(pwm_);
    sum_pwm += pwm_;



    // Serial.println(sum_rpm);
    // Serial.println(sum_pwm);
    if (sum_rpm == 0 && sum_pwm == 0){      // not detected
      digitalWrite(red, LOW);
      digitalWrite(blue, HIGH);
      digitalWrite(green, HIGH);
      // Serial.println("red");
    }else if (sum_rpm != 0 && sum_pwm != 0){  // detected NOT aligned
      digitalWrite(red, HIGH);
      digitalWrite(blue, LOW);
      digitalWrite(green, HIGH);
      // Serial.println("blue");
    }else if (sum_rpm != 0 && sum_pwm == 0){    // aligned
      digitalWrite(red, HIGH);
      digitalWrite(blue, HIGH);
      digitalWrite(green, LOW);
      // Serial.println("green");
    }
//   if(Serial.available())
//   {
// bldc_rpm=Serial.readString().toInt();
//   }
  // Serial.println("Main loop");

  // Serial.printf("%d %d %d %d %d %d %d %d %d %d %d %d %d ",);

  if(jetdata.r1 && jetdata.bldc_pwm != 0){
    is_r1_pressed=true;
    // flag_bldc_pass = true;
  if(jetdata.bldc_pwm>0){
    // bldc_acc((jetdata.bldc_pwm*7)*4/5);
    // int lastTime=millis();
  // bldc_acc(jetdata.bldc_pwm*7);
        // UART.setRPM((jetdata.bldc_pwm)*7*10/15);/
    // bldc_RPM=jetdata.bldc_pwm;
    bldc_RPM=jetdata.bldc_pwm;
    // for(int i=0;i<3;i++)
    //   kp[i]=40;
    flag_bldc=true;
        delay(3000);
    runMotor(-1*255*64,feed_pwmL,feed_pwmR);
    delay(3000);
    runMotor(0,feed_pwmL,feed_pwmR); 
    flag_bldc=false;
    bldc_RPM=0;
    // while(millis()-lastTime<3000)
    // {
    //   // UART.setRPM(jetdata.bldc_pwm*7);
    //   if (UART.getVescValues()){
    }

    else
        //     Serial.println("Values: ");
    //     // Serial.println(rpm);
    //     Serial.println(UART.data.rpm / 7);}
    // }
    // delay(2000);
    //   flag_bldc=false;
    //   bldc_RPM=0;
      // for(int i=0;i<3;i++)
      //   kp[i]=9;
    
    {
      if(jetdata.bldc_pwm != 0){
        bldc_RPM=-1*jetdata.bldc_pwm;
      
      }
      else{
        bldc_RPM = -1 * store_rpm;
      }

    runMotor(-1*255*64,feed_pwmL,feed_pwmR);
    delay(2000);
    flag_bldc=false;
    is_flag_bldc=true;
    runMotor(0,feed_pwmL,feed_pwmR);
      bldc_RPM=0;

    }

    // bl

  } 

  if(jetdata.circle==1)
  {
    int lastTime=0;
    flag_bldc_dribble=true;
    delay(200);
    
    // lastTime=millis();
    // while(millis()-lastTime<200)
    // {
    //     con.getData();      
    // }

    runMotor(-12000,feed_pwmL,feed_pwmR);
    delay(1375);  

    data_update=false;
    double yy=0;
    for(int i=1;i<=100;i++)
    {
      // Serial.println("Drive working");
      yy=mapDouble(i,0,100.0,0.458,1.0);
      yy=easeInOutExpo(yy);
      y=-1*int(mapDouble(yy,0.3,1.0,0,85));
      // Serial.println(yy);
      delayMicroseconds(12000);
    }
    // y=-65;
    // delay(750);
    // y=0;

    runMotor(0*255*64,feed_pwmL,feed_pwmR);
    data_update=true;    
    flag_bldc_dribble=false;


    // analogWrite(feed_pwmR,0);
    // analogWrite(feed_pwmL,0*64);    
  } 
  if(jetdata.cross){
    runMotor(1*255*64,feed_pwmL,feed_pwmR);
  }
  if(jetdata.square)
  {
    runMotor(0*255*64,feed_pwmL,feed_pwmR);
    UART.setRPM(0);

    // flag_bldc_pass = false;
  }

  if(jetdata.triangle)
  {
    runMotor(-1*255*64,feed_pwmL,feed_pwmR);
    // flag_bldc_pass = false;
  }
  if(jetdata.l1)
  {
      UART.setRPM(1500*7);
    // flag_bldc_pass = false;
  }
  // if(jetdata.l2)
  // {
  //   UART.setRPM(0);
  // }
  delay(10);

}