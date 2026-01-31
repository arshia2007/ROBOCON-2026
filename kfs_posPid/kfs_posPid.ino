#include <Wire.h>
#include <AS5600.h>
#include <IntervalTimer.h> 

// ============= kfs picking ================= 
int dc_lpwm = 2;    
int dc_rpwm = 3;

// Using standard Wire (Pin 18 SDA, Pin 19 SCL)
AS5600 as5600(&Wire); 

//pos pid constants
float kfs_kp = 0.0;   
float kfs_ki = 0.0;   
float kfs_kd = 0.0;   

double prevError = 0;         
int kfs_sp = 0;
float kfs_integ=0.0;
float kfs_der=0.0;
float kfs_pid = 0.0;

long rotations = 0;
int prevRawAngle = 0;

IntervalTimer timer;


void setup() {
  Serial.begin(9600);

  // led on
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // 1. Initialize Standard Wire (Pins 18/19)
  // Wire.setSDA(16);  // Set SDA to pin 16
  // Wire.setSCL(17);  // Set SCL to pin 17
  Wire.begin();
  
  // 2. Initialize AS5600
  as5600.begin(); 

  // Check connection
  if (!as5600.isConnected()) {
    Serial.println("WARNING: AS5600 not found on Pins 18/19!");
  } else {
    prevRawAngle = as5600.readAngle();
  }

  // Motor control pins setup
  pinMode(dc_lpwm, OUTPUT);
  pinMode(dc_rpwm, OUTPUT);

  // Initialize motors to stop
  analogWrite(dc_lpwm, 0);
  analogWrite(dc_rpwm, 0);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);


  timer.begin(calcPID, 75000);
}

void loop() {

}

// void input() {
//   if (Serial.available() > 0) {       
//     String input = Serial.readString();
//     kfs_sp = input.toInt() / 360.0 * 4096;   // deg -> ticks
//   }
//   // Serial.printf(" kfs_sp:%f", kfs_sp);
// }

void input() {
  if (Serial.available() > 0) {       // 0350.50.30
  
  String input = Serial.readString();

  kfs_kp = input.substring(0,3).toFloat();
  kfs_ki = input.substring(3,6).toFloat();
  kfs_kd = input.substring(6,9).toFloat();
  kfs_sp = input.substring(9).toFloat() / 360.0 * 4096;
  }
  Serial.printf(" kp:%f\n", kfs_kp);
  Serial.printf(" ki:%f", kfs_ki);
  Serial.printf(" kd:%f", kfs_kd);
  Serial.printf(" sp:%f", kfs_sp);
}

void parseSerialInput() {
  if (Serial.available() > 0) {
    // Read the entire line
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove \r, \n, spaces
    
    if (input.length() == 0) return;
    
    Serial.print("Received: ");
    Serial.println(input);
    
    // Find comma positions
    int comma1 = input.indexOf(',');
    int comma2 = input.indexOf(',', comma1 + 1);
    int comma3 = input.indexOf(',', comma2 + 1);
    
    // Check if we have all 3 commas
    if (comma1 == -1 || comma2 == -1 || comma3 == -1) {
      Serial.println("Error: Invalid format. Use: kp,ki,kd,angle");
      Serial.println("Example: 0.5,0.0,0.0,30.0");
      return;
    }
    
    // Extract substrings
    String kp_str = input.substring(0, comma1);
    String ki_str = input.substring(comma1 + 1, comma2);
    String kd_str = input.substring(comma2 + 1, comma3);
    String angle_str = input.substring(comma3 + 1);
    
    // Convert to values
    kfs_kp = kp_str.toFloat();
    kfs_ki = ki_str.toFloat();
    kfs_kd = kd_str.toFloat();
    float angle_degrees = angle_str.toFloat();
    
    // Convert angle to encoder counts (AS5600 has 4096 counts/rev)
    kfs_sp = (int)(angle_degrees * (4096.0 / 360.0));
    
    // Reset integral term when setpoint changes
    kfs_integ = 0;
    prevError = 0;
    
    // Print confirmation
    Serial.print("Updated - ");
    Serial.print("KP: "); Serial.print(kfs_kp, 3);
    Serial.print(", KI: "); Serial.print(kfs_ki, 3);
    Serial.print(", KD: "); Serial.print(kfs_kd, 3);
    Serial.print(", Angle: "); Serial.print(angle_degrees, 1);
    Serial.print("°, SP: "); Serial.println(kfs_sp);
  }
}

void calcPID() {

  // input();
  parseSerialInput();

  // // Read encoder with wrap-around handling
  // int raw = as5600.readAngle();
  
  // // Handle wrap-around
  // if ((raw - prevRawAngle) < -2048) rotations++;
  // else if ((raw - prevRawAngle) > 2048) rotations--;
  
  // long currentCounts = (rotations * 4096L) + raw;
  // prevRawAngle = raw;
  // Serial.println(currentCounts);
 
  long currentCounts = as5600.getCumulativePosition();

  // PID Control 
  float err = kfs_sp - currentCounts;
  kfs_integ = kfs_integ + (err*0.075);
  kfs_der = (err - prevError)/0.075;

  kfs_pid = (kfs_kp*err) + (kfs_ki*kfs_integ) + (kfs_kd*kfs_der);
  prevError = err;

  runMotor(dc_rpwm, dc_lpwm, kfs_pid);

}

void runMotor(int RPWM, int LPWM, float speed) {
  int pwm = abs(speed);
  pwm = constrain(pwm, 0, 12000);

  if (speed > 0) {      // to check direction: if +ve - HIGH, else LOW
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  } else if (speed < 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}