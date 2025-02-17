#pragma once

#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <Arduino.h>
#include <Wire.h>

#define POWER 23

namespace motors {
    QueueHandle_t xQueue;
    SemaphoreHandle_t xSemaphore;

    void loop(void* pvParameters);

    void encoder_reset_callback(const std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
    void encoder_reset_semaphore(void* pvParameters);

    void motors_callback(const geometry_msgs::Twist& msg);
    void motor_queue(void* pvParameters);

    void go(int first, int second, int third, int fourth);
    void set_motor_speeds(int address, int mSpeed1, int mSpeed2);
    void get_encoders(int32_t array[]);
    void resetEncoder();
    void send_command(int id, int command);

    int32_t encoder[4];

    std_msgs::Int32MultiArray arr_msg;

    ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> encoder_reset_ser("encoder_reset", &encoder_reset_callback);
    ros::Subscriber<geometry_msgs::Twist> motors_sub("/cmd_vel", &motors_callback);
    ros::Publisher encoder_pub("/encoder", &arr_msg);

    void setup() {
        Wire.begin();
        for (int16_t i = 1; i < 3; ++i) send_command(i, 0x27);
        for (int16_t i = 1; i < 3; ++i) send_command(i, 0x25);
        for (int16_t i = 1; i < 3; ++i) send_command(i, 0x4E);

        nh.advertiseService(encoder_reset_ser);
        nh.advertise(encoder_pub);
        nh.subscribe(motors_sub);

        pinMode(POWER, OUTPUT);
        digitalWrite(POWER, HIGH);

        xQueue = xQueueCreate(1, sizeof(geometry_msgs::Twist*));
        xSemaphore = xSemaphoreCreateMutex();

        xTaskCreate(loop, "Encoders", 2048, NULL, 1, NULL);
        xTaskCreate(motor_queue, "MotorQueue", 4096, NULL, 1, NULL);

        arr_msg.data_length = 4;
    }

    void loop(void* pvParameters) {
        while (1) {
            if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
                get_encoders(encoder);
                arr_msg.data = encoder;
                encoder_pub.publish(&arr_msg);

                xSemaphoreGive(xSemaphore);
            }

            vTaskDelay(15 / portTICK_PERIOD_MS);
        }
    }

    void encoder_reset_callback(const std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)  {
        xTaskCreate(encoder_reset_semaphore, "EncoderReset", 2048, NULL, 2, NULL);
    }

    void encoder_reset_semaphore(void* pvParameters) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            resetEncoder();
            xSemaphoreGive(xSemaphore);
        }
        vTaskDelete(NULL);
    }

    void motors_callback(const geometry_msgs::Twist& msg) {
        geometry_msgs::Twist* msg_ptr = new geometry_msgs::Twist(msg);
        if (xQueueSend(xQueue, &msg_ptr, 0) != pdPASS) {
            xQueueReceive(xQueue, NULL, 0);
            xQueueSend(xQueue, &msg_ptr, 0);
        }
    }

    void motor_queue(void* pvParameters) {
        geometry_msgs::Twist* msg;
        while (1) {
            if (xQueueReceive(xQueue, &msg, portMAX_DELAY) == pdTRUE) {
                if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
                    float x = msg->linear.x;
                    float y = msg->linear.y;
                    float z = msg->angular.z;

                    go(
                        -(x-y)/M_SQRT2*3600.0/M_PI - z*1135.0/M_PI,
                        (x+y)/M_SQRT2*3600.0/M_PI - z*1135.0/M_PI,
                        -(x+y)/M_SQRT2*3600.0/M_PI - z*1135.0/M_PI,
                        (x-y)/M_SQRT2*3600.0/M_PI - z*1135.0/M_PI
                    );

                    delete msg;

                    xSemaphoreGive(xSemaphore);
                }
            }
        }
    }

    void go(int first, int second, int third, int fourth) {
        set_motor_speeds(1, -constrain(first, -720, 720), -constrain(second, -720, 720));
        set_motor_speeds(2, -constrain(third, -720, 720), -constrain(fourth, -720, 720));
    }

    void set_motor_speeds(int address, int mSpeed1, int mSpeed2) {
        int lobyte1 = lowByte(mSpeed1);
        int hibyte1 = highByte(mSpeed1);

        int lobyte2 = lowByte(mSpeed2);
        int hibyte2 = highByte(mSpeed2);
        
        Wire.beginTransmission(address);
        Wire.write(0x45);
        Wire.write(hibyte1);
        Wire.write(lobyte1);
        Wire.write(hibyte2);
        Wire.write(lobyte2);
        Wire.endTransmission();

        vTaskDelay(15 / portTICK_PERIOD_MS);
    }

    void get_encoders(int32_t array[]) {
        unsigned long eCount;    // return value variable. We have to pass this an unsigned into Arduino.

        byte byte1;
        byte byte2;
        byte byte3;
        byte byte4;

        for (int i = 1; i < 3; i++) {
            Wire.beginTransmission(i);
            Wire.write(0x49);
            Wire.endTransmission();
            vTaskDelay(10 / portTICK_PERIOD_MS);

            Wire.requestFrom(i, 4);
            byte1 = Wire.read();
            byte2 = Wire.read();
            byte3 = Wire.read();
            byte4 = Wire.read();

            eCount = byte1;
            eCount = (eCount*256)+byte2;
            eCount = (eCount*256)+byte3;
            eCount = (eCount*256)+byte4;
            array[(3-i)*2-1] = eCount;
            vTaskDelay(10 / portTICK_PERIOD_MS);

            Wire.beginTransmission(i);
            Wire.write(0x4A);
            Wire.endTransmission();
            vTaskDelay(10 / portTICK_PERIOD_MS);

            Wire.requestFrom(i, 4);
            byte1 = Wire.read();
            byte2 = Wire.read();
            byte3 = Wire.read();
            byte4 = Wire.read();

            eCount = byte1;
            eCount = (eCount*256)+byte2;
            eCount = (eCount*256)+byte3;
            eCount = (eCount*256)+byte4;
            array[(3-i)*2-2] = eCount;
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }  
    }

    void resetEncoder() {
        for (int i = 1; i < 3; i++) {
            Wire.beginTransmission(i);
            Wire.write(0x4E);
            Wire.endTransmission();
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    void send_command(int id, int command) {
        Wire.beginTransmission(id);
        Wire.write(command);
        Wire.endTransmission();
        vTaskDelay(15 / portTICK_PERIOD_MS);
    }
}