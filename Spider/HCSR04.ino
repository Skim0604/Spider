void initHC_SR04(void) {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

int readHC_SR04() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long  duration = pulseIn(ECHO, HIGH);

  return duration * 0.034 / 2;
}
