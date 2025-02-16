#include <Arduino.h>

#define MOTOR1_UP 15
#define MOTOR1_DOWN 2
#define MOTOR2_UP 4
#define MOTOR2_DOWN RX2
#define MOTOR3_UP TX2
#define MOTOR3_DOWN 5
#define MOTOR4_UP 18
#define MOTOR4_DOWN 19

#define MOTOR1_DIR 32
#define MOTOR2_DIR 26
#define MOTOR3_DIR 27
#define MOTOR4_DIR 13

#define MOTOR1_SPD 33
#define MOTOR2_SPD 25
#define MOTOR3_SPD 14
#define MOTOR4_SPD 12

#define MOTOR1_CHANNEL 0
#define MOTOR2_CHANNEL 1
#define MOTOR3_CHANNEL 2
#define MOTOR4_CHANNEL 3

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(MOTOR1_UP, INPUT_PULLUP);
  pinMode(MOTOR1_DOWN, INPUT_PULLUP);
  pinMode(MOTOR2_UP, INPUT_PULLUP);
  pinMode(MOTOR2_DOWN, INPUT_PULLUP);
  pinMode(MOTOR3_UP, INPUT_PULLUP);
  pinMode(MOTOR3_DOWN, INPUT_PULLUP);
  pinMode(MOTOR4_UP, INPUT_PULLUP);
  pinMode(MOTOR4_DOWN, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(digitalRead(MOTOR1_UP));
  Serial.print(", ");
  Serial.print(analogRead(MOTOR1_DOWN));
  Serial.print(", ");
  Serial.print(digitalRead(MOTOR2_UP));
  Serial.print(", ");
  Serial.print(digitalRead(MOTOR2_DOWN));
  Serial.print(", ");

  Serial.print(digitalRead(MOTOR3_UP));
  Serial.print(", ");
  Serial.print(digitalRead(MOTOR3_DOWN));
  Serial.print(", ");
  Serial.print(digitalRead(MOTOR4_UP));
  Serial.print(", ");
  Serial.println(digitalRead(MOTOR4_DOWN));
}
