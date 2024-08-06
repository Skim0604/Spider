#include <Wire.h>
#include <DynamixelSerial.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "nuke.h"

#define BNO055_ADDRESS 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS, &Wire);

#define SH110X_BLACK  BLACK
#define SH110X_WHITE  WHITE
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT    64
#define OLED_RESET       -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define RXD2 16
#define TXD2 17
HardwareSerial serial2(1);

//IDservo
#define Pcoxa_FL     2
#define Pcoxa_RL     1
#define Pcoxa_FR     4
#define Pcoxa_RR     3

#define Pfemur_FL    6
#define Pfemur_RL    5
#define Pfemur_FR    8
#define Pfemur_RR    7

#define Ptibia_FL    10
#define Ptibia_RL    9
#define Ptibia_FR    12
#define Ptibia_RR    11

//Servo ARM
#define SERVO_ATAS   20
#define SERVO_BAWAH  21
#define POW          23
Servo servo_POW;

//pin
#define TRIG             25
#define ECHO             26

const byte servoID[4][3] = {
  {Pcoxa_FR, Pfemur_FR, Ptibia_FR},
  {Pcoxa_RR, Pfemur_RR, Ptibia_RR},
  {Pcoxa_FL, Pfemur_FL, Ptibia_FL},
  {Pcoxa_RL, Pfemur_RL, Ptibia_RL}
};

//---------------Variabel PID--------------------------
int imu_ref, computedImu;
float read_gyroy;


int koordinat_OBJ[2];
char buff[33];

void readDataJetson() {

}

void setup() {
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.setRotation(0);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(16, 0);
  display.print("POLINEMA");
  display.setCursor(8, 20);
  display.print("ROBOSAREMA");
  display.setTextSize(1);
  display.display();

  initServo();
  initBNO055();
  initHC_SR04();
  delay(2000);

  setupIK();
  gaitSelect(RIPPLE);
  doIK();

  display.clearDisplay();


  /* Set IMU 0 */
  imu_ref = readBNO(0);

  Xspeed = 0;
}

void loop() {
  arm(1);
  while (1) {
    if (Serial.available()) {
      String response = Serial.readStringUntil('\n');
      Serial.println(response);
      if (response == "") {
        response = "-1";
      }
      // Parsing DATA response
      int firstComma = response.indexOf(',');
      int secondComma = response.indexOf(',', firstComma + 1);
      koordinat_OBJ[0] = response.substring(0, firstComma).toInt(); //koordinat x
      koordinat_OBJ[1] = response.substring(firstComma + 1, secondComma).toInt(); //koordinat y}
    }

    if (koordinat_OBJ[0] != -1 && koordinat_OBJ[0] != 0) {
      if (koordinat_OBJ[0] > (300 - 70) && koordinat_OBJ[0] < (300 + 50)) {
        Serial.println("STOP");
        Rspeed = 0;
        if (koordinat_OBJ[0] > (300 - 70)) {
          imu_ref = readBNO(0) + 2;
          delay(1000);
        } else if (koordinat_OBJ[0] > (300 + 70)) {
          imu_ref = readBNO(0) - 2;
          delay(1000);
        } else {
          imu_ref = readBNO(0);
          delay(1000);
        }
        break;
      }
      else if (koordinat_OBJ[0] > (300 - 70)) {
        Rspeed = -0.1;
      } else if (koordinat_OBJ[0] < (300 + 50)) {
        Rspeed = 0.1;
      }
    } else {
      Rspeed = 0;
    }

    gaitSelect(RIPPLE);
    doIK();

    display.setTextSize(2);
    display.setCursor(4, 0);
    display.print("ROBOSAREMA");

    display.setTextSize(1);
    sprintf(buff, "KOORDINAT X:%d Y:%d", koordinat_OBJ[0], koordinat_OBJ[1]);
    display.setCursor(0, 20);
    display.print(buff);

    sprintf(buff, "Xspeed:%d", Xspeed);
    display.setCursor(0, 30);
    display.print(buff);

    sprintf(buff, "Yspeed:%d", Yspeed);
    display.setCursor(0, 40);
    display.print(buff);

    sprintf(buff, "Rspeed:%s", String(Rspeed));
    display.setCursor(0, 50);
    display.print(buff);

    display.display();
    display.clearDisplay();
  }

  arm(1);
  Xspeed = 20;

  while (1) {
    int dist = readHC_SR04();
    Serial.println(dist);
    if (dist < 5) {
      Xspeed = 0;
      arm(0);
      while (1) {
        Serial.println("STOP");
        display.setTextSize(2);
        display.setCursor(4, 0);
        display.print("ROBOSAREMA");

        display.setTextSize(2);
        display.setCursor(20, 30);
        display.print("SELESAI");

        display.display();
        display.clearDisplay();

        setupIK();
        gaitSelect(RIPPLE);
        doIK();
      }
    }
    else {
      scanImu();
      Rspeed = cal_pid(0, -10, 10, -0.2, 0.2, 0.1, 0.1,
                       0.2, 0.01, 0.1, computedImu);
    }
    gaitSelect(RIPPLE);
    doIK();

    display.setTextSize(2);
    display.setCursor(4, 0);
    display.print("ROBOSAREMA");

    display.setTextSize(1);
    sprintf(buff, "IMU:%d", computedImu);
    display.setCursor(0, 20);
    display.print(buff);

    sprintf(buff, "Xspeed:%d", Xspeed);
    display.setCursor(0, 30);
    display.print(buff);

    sprintf(buff, "Yspeed:%d", Yspeed);
    display.setCursor(0, 40);
    display.print(buff);

    sprintf(buff, "Rspeed:%s", String(Rspeed));
    display.setCursor(0, 50);
    display.print(buff);

    display.display();
    display.clearDisplay();
  }

}
