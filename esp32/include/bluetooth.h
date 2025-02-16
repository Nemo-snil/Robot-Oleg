#pragma once

#include <Arduino.h>
#include <std_msgs/String.h>
#include <BluetoothSerial.h>

namespace bluetooth {
    BluetoothSerial ESP_BT;
    std_msgs::String str_msg;

    ros::Publisher bluetooth_pub("/bluetooth", &str_msg);

    void loop(void* pvParameters);

    void setup() {
        ESP_BT.begin("ESP32");

        nh.advertise(bluetooth_pub);

        xTaskCreate(loop, "Bluetooth", 4096, NULL, 1, NULL);
    }

    void loop(void* pvParameters) {
        while (1) {
            if (ESP_BT.available()) {
                String in = ESP_BT.readStringUntil('\n');
                str_msg.data = in.c_str();
                bluetooth_pub.publish(&str_msg);
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}