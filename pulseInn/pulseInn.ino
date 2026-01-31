unsigned long pulseInn(uint8_t pin, bool state, unsigned long timeout) {
  unsigned long startTime = micros();

  while (digitalReadFast(pin) == state) {
    if (micros() - startTime > timeout)
      return 0;
  }

  startTime = micros();
  while (digitalReadFast(pin) != state) {
    if (micros() - startTime > timeout)
      return 0;
  }

  unsigned long pulseStart = micros();
  while (digitalReadFast(pin) == state) {
    if (micros() - pulseStart > timeout)
      return 0;
  }

  return micros() - pulseStart;
}
