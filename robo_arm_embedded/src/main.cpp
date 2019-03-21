#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define FW 1
#define BW 0

#define SHOULDER_ENABLE 7
#define SHOULDER_FW 11
#define SHOULDER_BW 10

#define ELBOW_ENABLE 8
#define ELBOW_FW 9
#define ELBOW_BW 6

// base motor controler data sheet
// http://www.warburtech.co.uk/products/motion.control/discontinued/solutions.cubed.simple.bridge/downloads/SBDS_1.pdf

#define BASE_L1 3
#define BASE_L2 5
#define BASE_H1 12
#define BASE_H2 13

// <---    shoulder motor controler   --->
void disable_shoulder() {
  digitalWrite(SHOULDER_ENABLE, LOW);
}

void enable_shoulder() {
  digitalWrite(SHOULDER_ENABLE, HIGH);
}

void pwm_shoulder(uint8_t duty, uint8_t dir) {
  if (dir == FW) {
    // move shoulder forward
    analogWrite(SHOULDER_BW, 0);
    analogWrite(SHOULDER_FW, duty);
  } else {
    // move shoulder backwards
    analogWrite(SHOULDER_FW, 0);
    analogWrite(SHOULDER_BW, duty);
  }
}

//<---   elbow motor controler  --->
void disable_elbow() {
  digitalWrite(ELBOW_ENABLE, LOW);
}

void enable_elbow() {
  digitalWrite(ELBOW_ENABLE, HIGH);
}

void pwm_elbow(uint8_t duty, uint8_t dir) {
  if (dir == FW) {
    // move elbow forward
    analogWrite(ELBOW_BW, 0);
    analogWrite(ELBOW_FW, duty);
  } else {
    // move elbow backwards
    analogWrite(ELBOW_FW, 0);
    analogWrite(ELBOW_BW, duty);
  }
}

// <---    base motor controler   --->
void lock_base() {
  digitalWrite(BASE_H2, LOW);
  digitalWrite(BASE_L1, HIGH);
  digitalWrite(BASE_H1, LOW);
  digitalWrite(BASE_L2, HIGH);
}

void pwm_base(uint8_t duty, uint8_t dir) {
 if (dir == FW) {
    // move base FW
    digitalWrite(BASE_H2, HIGH);
    analogWrite(BASE_L1, duty);
    digitalWrite(BASE_H1, LOW);
    digitalWrite(BASE_L2, LOW);
  } else {
    // move base BW
    digitalWrite(BASE_H2, LOW);
    digitalWrite(BASE_L1, LOW);
    digitalWrite(BASE_H1, HIGH);
    analogWrite(BASE_L2, duty);
  }
}

void run_test() {
  // test elbow
  enable_elbow();
  pwm_elbow(90, BW);
  delay(500);
  pwm_elbow(130, FW);
  delay(500);
  disable_elbow();

  // test shoulder
  enable_shoulder();
  pwm_shoulder(90, BW);
  delay(500);
  pwm_shoulder(130, FW);
  delay(500);
  disable_shoulder();

  // test base
  pwm_base(100, BW);
  delay(500);
  pwm_base(100, FW);
  delay(500);
  lock_base();
  Serial.println("motor test done.");
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);

  // init pins
  // shoulder
  pinMode(SHOULDER_ENABLE, OUTPUT);
  digitalWrite(SHOULDER_ENABLE, LOW);
  pinMode(SHOULDER_FW, OUTPUT);
  pinMode(SHOULDER_BW, OUTPUT);

  // elbow
  pinMode(ELBOW_ENABLE, OUTPUT);
  digitalWrite(ELBOW_ENABLE, LOW);
  pinMode(ELBOW_FW, OUTPUT);
  pinMode(ELBOW_BW, OUTPUT);

  // base
  pinMode(BASE_L1, OUTPUT);
  pinMode(BASE_L2, OUTPUT);
  pinMode(BASE_H1, OUTPUT);
  pinMode(BASE_H2, OUTPUT);
  lock_base();
  run_test();
}

void loop() {
  char buff[10];
  if (Serial.available() > 0) {
    memset(buff, 0, 10);
    Serial.readBytesUntil('\n', buff, 40);
    if (strncmp((const char *) &buff, "right", 5) == 0) {
      pwm_base(120, FW);
      //delay(200);
      //Serial.println("got right\n\n");
    } else if (strncmp((const char *) &buff, "left", 4) == 0) {
      pwm_base(120, BW);
      //delay(200);
      //Serial.println("got left\n\n");
    } else if (strncmp((const char *) &buff, "up", 2) == 0) {
      enable_elbow();
      pwm_elbow(200, FW);
      //delay(200);
      //disable_elbow();
      //Serial.println("got up\n\n");
    } else if (strncmp((const char *) &buff, "down", 4) == 0) {
      enable_elbow();
      pwm_elbow(200, BW);
      //delay(200);
      //disable_elbow();
      //Serial.println("got down");
    } else if (strncmp((const char *) &buff, "s_v", 3) == 0) {
      disable_elbow();
    } else if (strncmp((const char *) &buff, "s_h", 3) == 0) {
      lock_base();
    }
    //lock_base();
  }
}