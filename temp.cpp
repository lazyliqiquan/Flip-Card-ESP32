/*代码思路是对的
就是有些没有考虑到，
【1】没有考虑电机不转动的时候，应该把控制线置为低电平，以免过热，记得记录上一步的状态，然后再置零
*/

#include <WiFi.h>
#include <SPI.h>
#include <time.h>

// 获取当前时间
// 此前到后分别是month-day-hour-minute-是否成功获取时间
uint8_t timeArr[9] = {};
// 上一次同步的时间
int lastSyncDay = -1;
// 每天凌晨三点同步一次
const int syncHour = 3;
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
        timeArr[6] = timeinfo.tm_sec / 10;
        timeArr[7] = timeinfo.tm_sec % 10;
        timeArr[8] = 1;
        if (timeinfo.tm_hour == syncHour && timeinfo.tm_mday != lastSyncDay)
        {
            lastSyncDay = timeinfo.tm_mday;
        }
    }
}

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

const char *ssid = "YOUR_WIFI";
const char *password = "YOUR_PASS";

const uint32_t WIFI_CONNECT_TIMEOUT = 10000;
const uint32_t WIFI_RETRY_INTERVAL = 60000;

void startWiFiConnect()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    wifiState = WIFI_CONNECTING;
    wifiStateTs = millis();

    Serial.println("WiFi connecting...");
}

// WiFi 状态机
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

#define MOTOR_COUNT 5
#define CHAR_COUNT 45
// SPI 引脚（4 根）
#define PIN_SCK 18
#define PIN_MOSI 23 // 74HC595
#define PIN_MISO 19 // 74HC165
#define PIN_CS 5    // 共用 CS

const char *ssid = "LQQ";
const char *password = "qq123456";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 8 * 3600; // 中国/台北/新加坡

SPIClass vspi(VSPI);
// 步进电机半步表
const uint8_t stepTable[8] = {
    0b1000, 0b1100, 0b0100, 0b0110,
    0b0010, 0b0011, 0b0001, 0b1001};
// 电机状态
uint8_t motorStep[MOTOR_COUNT] = {0};
int motorChar[MOTOR_COUNT] = {0};
bool motorEnable[MOTOR_COUNT] = {false};
// SPI：输出 595 + 读取 165（同一时钟）
uint8_t spiTransfer595_165(uint32_t out595)
{
    uint8_t hall;

    digitalWrite(PIN_CS, LOW);

    vspi.transfer((out595 >> 16) & 0xFF);
    vspi.transfer((out595 >> 8) & 0xFF);
    hall = vspi.transfer(out595 & 0xFF);

    digitalWrite(PIN_CS, HIGH);
    return hall;
}
// 构造步进电机输出（支持断电）
uint32_t buildMotorBits()
{
    uint32_t out = 0;

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (motorEnable[i])
        {
            out |= (uint32_t)stepTable[motorStep[i]] << (i * 4);
        }
    }
    return out;
}
// 单步推进（同步）
uint8_t stepOnce()
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (motorEnable[i])
        {
            motorStep[i] = (motorStep[i] + 1) % 8;
        }
    }

    return spiTransfer595_165(buildMotorBits());
}
// WiFi + 时间
void initTime()
{
    configTime(gmtOffset_sec, 0, ntpServer);
}

String getTimeStr()
{
    struct tm t;
    if (!getLocalTime(&t))
        return "-----";

    char buf[6];
    snprintf(buf, sizeof(buf), "%02d%02d", t.tm_hour, t.tm_min);
    return String(buf);
}

// 上电归零（霍尔校准）
void homeAllMotors()
{
    memset(motorEnable, true, sizeof(motorEnable));

    while (true)
    {
        uint8_t hall = stepOnce();
        bool done = true;

        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            if (hall & (1 << i))
            {
                motorEnable[i] = false;
                motorChar[i] = 0;
            }
            else
            {
                done = false;
            }
        }

        delay(5);
        if (done)
            break;
    }

    spiTransfer595_165(0); // 全断电
}
// 转到指定字符（含“每圈自动校准”）
void moveToChar(int idx, int targetChar)
{
    int diff = (targetChar - motorChar[idx] + CHAR_COUNT) % CHAR_COUNT;
    motorEnable[idx] = true;

    for (int i = 0; i < diff; i++)
    {
        motorStep[idx] = (motorStep[idx] + 1) % 8;
        spiTransfer595_165(buildMotorBits());
        delay(5);

        motorChar[idx]++;
        if (motorChar[idx] >= CHAR_COUNT)
        {
            motorChar[idx] = 0;

            // ===== 自动校准 =====
            while (true)
            {
                uint8_t hall = spiTransfer595_165(buildMotorBits());
                if (hall & (1 << idx))
                    break;
                motorStep[idx] = (motorStep[idx] + 1) % 8;
                delay(5);
            }
        }
    }

    motorChar[idx] = targetChar;
    motorEnable[idx] = false;
    spiTransfer595_165(buildMotorBits());
}

const int daylightOffset_sec = 0;
{
    Serial.begin(9600);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void setup() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.begin(9600);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  vspi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  homeAllMotors();
}


void loop() {
  static unsigned long last = 0;

  if (millis() - last > 6000) {
    last = millis();
    String t = getTimeStr();
    Serial.println(t);

    // 示例：只动第 0 位
    // moveToChar(0, random(0, 45));
  }
}
