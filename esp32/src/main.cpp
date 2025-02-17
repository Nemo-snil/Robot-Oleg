#include <Arduino.h>
#include <ros.h>

ros::NodeHandle nh;

#include <motors.h>
#include <bluetooth.h>
#include <lift.h>

void spin(void *pvParameters) {
  while (1) {
    nh.spinOnce();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  motors::setup();
  lift::setup();
  bluetooth::setup();
  xTaskCreate(spin, "spinOnce", 4096, NULL, 2, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(portMAX_DELAY);
}
