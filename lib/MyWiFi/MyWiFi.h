#ifndef MY_WIFI_H
#define MY_WIFI_H

#include <WiFi.h>
#include <time.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include "Motor.h"

// 翻页时钟展示的状态:0 时间；1 日期；2 文本；3 计算
extern uint8_t show_status;

extern unsigned long last_minute_trigger; // 最近一次触发函数调用的时间
// 多久更新一次时间参数:因为更新的时候不一定处于 0 秒的位置，如果更新间隔写死，会导致下一次更新的时间会比实际慢一些
extern int refresh_time_interval; // 更新时间的间隔
extern uint8_t timeArr[8];        // 此前到后分别是month-day-hour-minute

extern PubSubClient client;
extern char mqtt_client_id[50]; // 自动生成唯一 mqtt_client_id

void setup_wifi();                    // WiFi 连接
void init_mqtt();                     // 初始化mqtt配置
void reconnect();                     // MQTT 重连
void sync_ntp();                      // NTP 同步
void send_status(const char *status); // MQTT 上报状态
void parse_time();                    // 解析时间

#endif