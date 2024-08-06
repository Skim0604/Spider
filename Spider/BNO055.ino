#define BNO055_SAMPLERATE_DELAY_MS (1)

void initBNO055(){
  bno.begin();
  bno.setExtCrystalUse(true);
  BNO_CALL();
}

void displaySensorDetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  delay(100);
}

void displaySensorStatus(void) {
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  delay(100);
}

void BNO_CALL(void) {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
}

int readBNO(byte mode) {
  double Orientation;
  sensors_event_t event;
  bno.getEvent(&event);

  switch (mode) {
    case 0 : Orientation = (event.orientation.x); break;
    case 1 : Orientation = (event.orientation.y); break;
    case 2 : Orientation = (event.orientation.z); break;
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
  return Orientation;
}
