#define pinDirection 2
int periode = 50;

void initServo() {
  serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2);
  Dynamixel.setSerial(&serial2);
  Dynamixel.begin(1000000, pinDirection);
  ESP32PWM::allocateTimer(0);
  servo_POW.setPeriodHertz(periode);
  servo_POW.attach(POW, 1000, 2000);
  servo_POW.write(0);
  arm(1);
}

void servo(byte ID, int degree) {
  int position  = (degree * 1023) / 300;
  // Dynamixel.move(ID, position);
  Dynamixel.moveSpeed(ID, position, 350);
}

void arm(byte mode) {
  switch (mode) {
    case 0: //Atas
      for (byte i = 0; i < 20; i++) {
        servo(SERVO_ATAS, 240 - i);
        servo(SERVO_BAWAH, 234 - (i + 5));
        delay(100);
      }

      servo_POW.write(100);
      delay(200);
      servo(SERVO_ATAS, 60);
      delay(400);
      servo(SERVO_BAWAH, 210);
      delay(400);
      break;

    case 1: //Bawah
      servo_POW.write(0);
      delay(200);
      servo(SERVO_BAWAH, 240);
      delay(400);
      servo(SERVO_ATAS, 246);
      delay(400);
      break;
  }
}

void arm_PICK() {
  servo(SERVO_BAWAH, 240);
  delay(400);
  servo(SERVO_ATAS, 120);
  delay(400);
}
