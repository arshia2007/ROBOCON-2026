const int NUM_SENSORS = 12;
int distances[NUM_SENSORS];

uint8_t buffer[sizeof(distances)];
int index_ = 0;
bool receiving = false;

void setup() {
  Serial.begin(9600);     
  Serial1.begin(115200);   
}

void loop() {

  while (Serial1.available()) {
    uint8_t incoming = Serial1.read();

    // Start marker
    if (incoming == '<') {
      receiving = true;
      index_ = 0;
    }
    // End marker
    else if (incoming == '>' && receiving) {
      memcpy(distances, buffer, sizeof(distances));
      receiving = false;

      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(distances[i]);
        Serial.print(" ");
      }
      Serial.println();
    }

    else if (receiving && index_ < sizeof(distances)) {
      buffer[index_++] = incoming;
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(distances[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
