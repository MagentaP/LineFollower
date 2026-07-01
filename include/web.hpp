#pragma once
#include <WiFi.h>
#include <WebServer.h>
#include "chassis.hpp"

extern WebServer server;
extern String  web_mode;
extern String  web_cmd;
extern float   web_debug_angle;
extern float   web_debug_speed;
extern unsigned long web_cmd_time;
extern Chassis* car;

extern const char HTML_PAGE[];

void httpHandleRoot();
void httpHandleStatus();
void httpHandleSet();
void httpHandleMode();
void httpHandleManual();
void httpHandleCalibrate();
