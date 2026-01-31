IntervalTimer Timer;

int climber[2][2] = {
  {7,6},      // front lift motor
  {3, 2}     // rear lift motor
};

int ultra[2][2] = {
  {14, 15},      // front ultrasonic    -    trig, echo
  {13, 12}       // rear ultrasonic
};

const int PWM_MAX = 14000;      
const int near = 70;  

int holding_pwm = 
int targetHeight[2] = {230, 230};     // mm
float Kp=0;


void driveMotor(int motor[], int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  analogWrite(motor[0], pwm>0?pwm:0);
  analogWrite(motor[1], pwm<0?-pwm:0);

}

int readHeight(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long t = pulseIn(echo, HIGH, 25000); 
  if (t == 0) return -1;

  return t * 0.343 / 2;
}

// p control
void computePWM(int i,int Kp, float height) {
  int error = targetHeight[i] - height;
  int pwm = Kp * error;

  if(abs(error) < near){
    pwm+= holding_pwm;
  }

  Serial.print("error: ");
  Serial.print(error);
  Serial.print("  curr: ");
  Serial.println(height);

  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  driveMotor(climber[i], -pwm);
}

void controlLoop() {
  int height1 = readHeight(ultra[0][0], ultra[0][1]);
  int height2 = readHeight(ultra[1][0], ultra[1][1]);
  computePWM(0, Kp, height1);   // front lift
  computePWM(1, Kp, height2);   // rear lift
}

void setup() {
  Serial.begin(115200);
  analogWriteResolution(14);

  for (int i = 0; i < 2; i++) {
    pinMode(climber[i][0], OUTPUT);
    pinMode(climber[i][1], OUTPUT);
    pinMode(ultra[i][0], OUTPUT);
    pinMode(ultra[i][1], INPUT);
  }

  // 25 Hz control loop → 40 ms
  Timer.begin(controlLoop, 40000);

  Serial.println("IntervalTimer lift control started");
}

void loop() {
  // Serial command: set same height for both lifts
  if (Serial.available()) {
    Kp = Serial.parseInt();
    
  }
}