#include <Arduino.h>
#include "chassis.hpp"
#include "web.hpp"

Motor    left_motor(M1_IN1, M1_IN2, true);
Motor    right_motor(M2_IN1, M2_IN2, false);
Chassis  chassis(left_motor, right_motor);

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Boot begin");

    left_motor.begin(0);
    right_motor.begin(2);
    Serial.println("Motors OK");

    chassis.begin();
    car = &chassis;
    Serial.println("Chassis OK");

    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    server.on("/", httpHandleRoot);
    server.on("/status", httpHandleStatus);
    server.on("/set", httpHandleSet);
    server.on("/mode", httpHandleMode);
    server.on("/man", httpHandleManual);
    server.on("/cal", httpHandleCalibrate);
    server.onNotFound([]
    {
        server.send(404, "text/plain", "404");
    });
    server.begin();
}

void loop()
{
    server.handleClient();

    chassis.web_mode_ = web_mode;
    chassis.web_cmd_ = web_cmd;
    chassis.debug_angle_ = web_debug_angle;
    chassis.debug_speed_ = web_debug_speed;

    chassis.chassisTask();
}
