const int NUM_SENSORS = 12;
int distances[NUM_SENSORS];

const int trigPins[NUM_SENSORS] = {
  2, 3, 4, 5, 6, 7,
  8, 9, 10, 11, 12, 13
};

const int echoPins[NUM_SENSORS] = {
  22, 23, 24, 25, 26, 27,
  28, 29, 30, 31, 32, 33
};


void setup() {
  Serial.begin(9600);
  Serial1.begin(115200); 
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

}

void loop() {

  for (int i = 0; i < NUM_SENSORS; i++) {
    distances[i] = readUltrasonic(trigPins[i], echoPins[i]);
    delayMicroseconds(150); 
  }

  Serial1.write('<');  // start marker
  Serial1.write((uint8_t*)distances, sizeof(distances));
  Serial1.write('>');  // end marker

  delay(40);  
}

int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH); 

  if (duration == 0) {
    return -1; // no echo
  }

  int distance = duration * 0.0343 / 2; // cm
  return distance;
  
}

