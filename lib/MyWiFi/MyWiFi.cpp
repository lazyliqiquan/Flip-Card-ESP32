#include "MyWiFi.h"


// WIFI和时间参数
// 此前到后分别是month-day-hour-minute-是否成功获取时间
uint8_t timeArr[9] = {};
// 上一次同步的时间
int lastSyncDay = -1;   
// 每天凌晨三点同步一次
const int syncHour = 3;
// WiFi的状态
enum WiFiState
{
    WIFI_CLOSE,
    WIFI_CONNECTING,
    WIFI_CONNECTED,
    WIFI_WAIT_RETRY
};
WiFiState wifiState = WIFI_CLOSE;
uint32_t wifiStateTs = 0;
const uint32_t WIFI_CONNECT_TIMEOUT = 10000;
const uint32_t WIFI_RETRY_INTERVAL = 60000;
const char *ssid = "LQQ";
const char *password = "qq123456";
const char *ntpServer = "ntp.aliyun.com";
const long gmtOffset_sec = 8 * 3600; // 中国/台北/新加坡
const int daylightOffset_sec = 0;

void wifiTask()
{
    uint32_t now = millis();
    switch (wifiState)
    {
    case WIFI_CLOSE:
        // 需要网络时，外部切到 CONNECTING，在setup里面直接 wifiState = WIFI_CONNECTING 即可
        break;
    case WIFI_CONNECTING:
        if (WiFi.status() == WL_CONNECTED)
        {
            wifiState = WIFI_CONNECTED;
            Serial.println("WiFi connected");
        }
        else if (now - wifiStateTs > WIFI_CONNECT_TIMEOUT)
        {
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);

            wifiState = WIFI_WAIT_RETRY;
            wifiStateTs = now;
            Serial.println("WiFi connect timeout, wait retry");
        }
        break;
    case WIFI_CONNECTED:
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("WiFi lost");
            wifiState = WIFI_WAIT_RETRY;
            wifiStateTs = now;
        }
        break;
    case WIFI_WAIT_RETRY:
        if (now - wifiStateTs > WIFI_RETRY_INTERVAL)
        {
            startWiFiConnect();
        }
        break;
    }
}

void startWiFiConnect()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    wifiState = WIFI_CONNECTING;
    wifiStateTs = millis();

    Serial.println("WiFi connecting...");
}

void getCurrentTime()
{
    struct tm timeinfo;
    timeArr[8] = 0;
    if (getLocalTime(&timeinfo))
    {
        timeArr[0] = (timeinfo.tm_mon + 1) / 10;
        timeArr[1] = (timeinfo.tm_mon + 1) % 10;
        timeArr[2] = timeinfo.tm_mday / 10;
        timeArr[3] = timeinfo.tm_mday % 10;
        timeArr[4] = timeinfo.tm_hour / 10;
        timeArr[5] = timeinfo.tm_hour % 10;
        timeArr[6] = timeinfo.tm_min / 10;
        timeArr[7] = timeinfo.tm_min % 10;
        timeArr[8] = 1;
        if (timeinfo.tm_hour == syncHour && timeinfo.tm_mday != lastSyncDay)
        {
            Serial.println("sync time");
            lastSyncDay = timeinfo.tm_mday;
            // 连接ntp服务器校准误差
            configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        }
    }
}
