#pragma once

#include <Arduino.h>
#include <std_srvs/Empty.h>

#define MOTOR1_DIR 13
#define MOTOR2_DIR 27
#define MOTOR3_DIR 26
#define MOTOR4_DIR 32

#define MOTOR1_SPD 12
#define MOTOR2_SPD 14
#define MOTOR3_SPD 25
#define MOTOR4_SPD 33

#define MOTOR1_CHANNEL 0
#define MOTOR2_CHANNEL 1
#define MOTOR3_CHANNEL 2
#define MOTOR4_CHANNEL 3

#define MOTOR1_UP 15
#define MOTOR1_DOWN 2
#define MOTOR2_UP 4
#define MOTOR2_DOWN RX2
#define MOTOR3_UP TX2
#define MOTOR3_DOWN 5
#define MOTOR4_UP 18
#define MOTOR4_DOWN 19

#define DOWN 0
#define UP 1
#define HZ 2

#define SPEED_UP 80
#define SPEED_DOWN 50

namespace lift {
    xSemaphoreHandle xSemaphore;

    uint8_t lift_mode;

    void lift_up_callback(const std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
    void lift_down_callback(const std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

    void lift_up(void* pvParameters);
    void lift_down(void* pvParameters);

    ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> lift_up_ser("lift_up", &lift_up_callback);
    ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> lift_down_ser("lift_down", &lift_down_callback);

    void setup() {
        xSemaphore = xSemaphoreCreateMutex();

        nh.advertiseService(lift_up_ser);
        nh.advertiseService(lift_down_ser);

        pinMode(MOTOR1_UP, INPUT_PULLUP);
        pinMode(MOTOR1_DOWN, INPUT_PULLUP);
        pinMode(MOTOR2_UP, INPUT_PULLUP);
        pinMode(MOTOR2_DOWN, INPUT_PULLUP);
        pinMode(MOTOR3_UP, INPUT_PULLUP);
        pinMode(MOTOR3_DOWN, INPUT_PULLUP);
        pinMode(MOTOR4_UP, INPUT_PULLUP);
        pinMode(MOTOR4_DOWN, INPUT_PULLUP);

        pinMode(MOTOR1_DIR, OUTPUT);
        pinMode(MOTOR2_DIR, OUTPUT);
        pinMode(MOTOR3_DIR, OUTPUT);
        pinMode(MOTOR4_DIR, OUTPUT);

        ledcSetup(MOTOR1_CHANNEL, 100, 8);
        ledcSetup(MOTOR2_CHANNEL, 100, 8);
        ledcSetup(MOTOR3_CHANNEL, 100, 8);
        ledcSetup(MOTOR4_CHANNEL, 100, 8);

        ledcAttachPin(MOTOR1_SPD, MOTOR1_CHANNEL);
        ledcAttachPin(MOTOR2_SPD, MOTOR2_CHANNEL);
        ledcAttachPin(MOTOR3_SPD, MOTOR3_CHANNEL);
        ledcAttachPin(MOTOR4_SPD, MOTOR4_CHANNEL);

        if (!digitalRead(MOTOR2_DOWN) && !digitalRead(MOTOR3_DOWN) && !digitalRead(MOTOR4_DOWN)) {
            lift_mode = DOWN;
        } else if (!digitalRead(MOTOR1_UP) && !digitalRead(MOTOR2_UP) && !digitalRead(MOTOR3_UP) && !digitalRead(MOTOR4_UP)) {
            lift_mode = UP;
        } else {
            lift_mode = HZ;
        }
    }

    void lift_up_callback(const std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
        xTaskCreate(lift_up, "LiftUp", 1024, NULL, 1, NULL);
    }

    void lift_down_callback(const std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
        xTaskCreate(lift_down, "LiftDown", 1024, NULL, 1, NULL);
    }

    void lift_up(void* pvParameters) {
        if (lift_mode == UP) {
            vTaskDelete(NULL);
            return;
        }

        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            digitalWrite(MOTOR1_DIR, HIGH);
            digitalWrite(MOTOR2_DIR, HIGH);
            digitalWrite(MOTOR3_DIR, HIGH);
            digitalWrite(MOTOR4_DIR, HIGH);

            while (true) {
                if (!digitalRead(MOTOR1_UP) && !digitalRead(MOTOR2_UP) && !digitalRead(MOTOR3_UP) && !digitalRead(MOTOR4_UP)) {break;}

                if (!digitalRead(MOTOR1_UP)) {ledcWrite(MOTOR1_CHANNEL, 0);} else {ledcWrite(MOTOR1_CHANNEL, SPEED_UP);}
                if (!digitalRead(MOTOR2_UP)) {ledcWrite(MOTOR2_CHANNEL, 0);} else {ledcWrite(MOTOR2_CHANNEL, SPEED_UP);}
                if (!digitalRead(MOTOR3_UP)) {ledcWrite(MOTOR3_CHANNEL, 0);} else {ledcWrite(MOTOR3_CHANNEL, SPEED_UP);}
                if (!digitalRead(MOTOR4_UP)) {ledcWrite(MOTOR4_CHANNEL, 0);} else {ledcWrite(MOTOR4_CHANNEL, SPEED_UP);}

                vTaskDelay(50 / portTICK_PERIOD_MS);
            }

            ledcWrite(MOTOR1_CHANNEL, 0);
            ledcWrite(MOTOR2_CHANNEL, 0);
            ledcWrite(MOTOR3_CHANNEL, 0);
            ledcWrite(MOTOR4_CHANNEL, 0);

            lift_mode = UP;

            xSemaphoreGive(xSemaphore);
            vTaskDelete(NULL);
        }
    }

    void lift_down(void* pvParameters) {
        if (lift_mode == DOWN) {
            vTaskDelete(NULL);
            return;
        }

        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            digitalWrite(MOTOR1_DIR, LOW);
            digitalWrite(MOTOR2_DIR, LOW);
            digitalWrite(MOTOR3_DIR, LOW);
            digitalWrite(MOTOR4_DIR, LOW);

            while (true) {
                if (!digitalRead(MOTOR1_DOWN) && !digitalRead(MOTOR2_DOWN) && !digitalRead(MOTOR3_DOWN) && !digitalRead(MOTOR4_DOWN)) {break;}

                if (digitalRead(MOTOR1_DOWN)) {ledcWrite(MOTOR1_CHANNEL, 0);} else {ledcWrite(MOTOR1_CHANNEL, SPEED_DOWN);}
                if (!digitalRead(MOTOR2_DOWN)) {ledcWrite(MOTOR2_CHANNEL, 0);} else {ledcWrite(MOTOR2_CHANNEL, SPEED_DOWN);}
                if (!digitalRead(MOTOR3_DOWN)) {ledcWrite(MOTOR3_CHANNEL, 0);} else {ledcWrite(MOTOR3_CHANNEL, SPEED_DOWN);}
                if (!digitalRead(MOTOR4_DOWN)) {ledcWrite(MOTOR4_CHANNEL, 0);} else {ledcWrite(MOTOR4_CHANNEL, SPEED_DOWN);}

                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
            
            ledcWrite(MOTOR1_CHANNEL, 0);
            ledcWrite(MOTOR2_CHANNEL, 0);
            ledcWrite(MOTOR3_CHANNEL, 0);
            ledcWrite(MOTOR4_CHANNEL, 0);

            lift_mode = DOWN;

            xSemaphoreGive(xSemaphore);
            vTaskDelete(NULL);
        }
    }
}