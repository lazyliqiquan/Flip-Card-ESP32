#ifndef MY_WIFI_H
#define MY_WIFI_H

#include <WiFi.h>
#include <time.h>
#include<Arduino.h>

// 获取当前时间
void getCurrentTime();
// 开始连接WiFi
void startWiFiConnect();
// WiFi 状态机
void wifiTask();

#endif