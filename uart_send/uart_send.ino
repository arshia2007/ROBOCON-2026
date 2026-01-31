const int NUM_SENSORS = 1;
int distances[NUM_SENSORS];

// bot 
// int trigPins[NUM_SENSORS] = {20, 14, 13, 40, 37, 33, 27, 30, 4, 28, 6, 23};
// int echoPins[NUM_SENSORS] = {21, 15, 12, 41, 36, 32, 26, 31, 5, 29, 7, 22};

// int trigPins[NUM_SENSORS] = {34, 8, 31, 10}; //front, left, back, right
// int echoPins[NUM_SENSORS] = {33, 7, 30, 11};

// int trigPins[NUM_SENSORS] = {25, 37, 8};// front, back, right
// int echoPins[NUM_SENSORS] = {26, 38,  7};

int trigPins[NUM_SENSORS] = {29};// back, right
int echoPins[NUM_SENSORS] = {28};

float alpha = 0.2;

static inline void enableCycleCounter() {
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}


void setup() {
  Serial.begin(115200);      // USB Serial for monitoring
  Serial1.begin(115200);   // UART to other Teensy
  
  // delay(100);
  Serial.println("Sender Readyyyy");
    for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);

    
  }

  enableCycleCounter();
}

uint32_t pulseInFast(uint8_t pin, bool state, uint32_t timeout_us)
{
  const uint32_t timeout_cycles = timeout_us * 600; // 600 MHz
  uint32_t start = ARM_DWT_CYCCNT;

  // Wait for any previous pulse to end
  while (digitalReadFast(pin) == state) {
    if ((ARM_DWT_CYCCNT - start) > timeout_cycles)
      return 0;
  }

  // Wait for pulse to start
  while (digitalReadFast(pin) != state) {
    if ((ARM_DWT_CYCCNT - start) > timeout_cycles)
      return 0;
  }

  // Pulse started
  uint32_t rise = ARM_DWT_CYCCNT;

  // Wait for pulse to end
  while (digitalReadFast(pin) == state) {
    if ((ARM_DWT_CYCCNT - rise) > timeout_cycles)
      return 0;
  }

  uint32_t fall = ARM_DWT_CYCCNT;

  // Convert cycles → microseconds
  return (fall - rise) / 600;
}

void loop() {
     for (int i = 0; i < NUM_SENSORS; i++) {
    distances[i] = readUltrasonic(trigPins[i], echoPins[i]);
    // delayMicroseconds(150); 
  }
  
  // Create comma-separated string and send
  String dataString = "";
  for (int i = 0; i < NUM_SENSORS; i++) {
    dataString += String(distances[i]);
    if (i < NUM_SENSORS - 1) {
      dataString += ",";
    }
    
  }
  // dataString += "hi";
  
  Serial1.println(dataString);  // Send string with newline
  
  // Optional: Debug print
  Serial.println(dataString);
  
  // delay(1000);  // Send every 1 second
}
int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(500);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseInFast(echoPin, HIGH,200000);
  // long duration = pulseIn(echoPin, HIGH);
  // Serial.printf("duration: %d\t", duration);
  // Serial.printf("duration2: %d\n", duration);
  // return 1;
  if (duration == 0) {
    return -1; // no echo
  }
  
  int distance = duration * 0.0343 / 2; // cm
  
  return distance;
}