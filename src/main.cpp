/*代码思路是对的
就是有些没有考虑到，
【1】没有考虑电机不转动的时候，应该把控制线置为低电平，以免过热，记得记录上一步的状态，然后再置零
*/
#include <WiFi.h>
#include <SPI.h>
#include <time.h>

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
// 获取当前时间
void getCurrentTime();
// 开始连接WiFi
void startWiFiConnect();
// WiFi 状态机
void wifiTask();

// 驱动参数
// 转完一圈需要的脉冲数
#define PULSE_COUNT 4096
// 每移动一个字符所需要的脉冲数 4096 / 45
#define PULSE_CHAR 91
// 显示位的数量
#define MOTOR_COUNT 5
// 每个模块显示字符的翻牌数
#define CHAR_COUNT 45
// SPI 引脚（4 根）
#define PIN_SCK 18  // 时钟信号线
#define PIN_MOSI 23 // 驱动信号线
#define PIN_MISO 19 // 霍尔传感器信号线
#define PIN_CS 5    // 共用 CS
SPIClass vspi(VSPI);
// 步进电机半步表
// const uint8_t stepTable[8] = {0b1000, 0b1100, 0b0100, 0b0110, 0b0010, 0b0011, 0b0001, 0b1001};
const uint8_t stepTable[8] = {0b1001, 0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000};
// 步进电机在电路板上74HC595的对照表
// const uint8_t motorTable[5][4] = {{7, 6, 5, 4}, {3, 2, 1, 15}, {14, 13, 12, 11}, {10, 9, 22, 21}, {20, 19, 18, 17}};
const uint8_t motorTable[5][4] = {{7, 6, 5, 4}, {3, 2, 1, 15}, {14, 13, 12, 11}, {10, 9, 22, 21}, {20, 19, 18, 17}};
// 每个脉冲之间delay的时长
uint8_t motorSpeed = 1;
// 电机状态
uint8_t motorStep[MOTOR_COUNT] = {0};
// 当前展示的文本
int8_t currentWords[MOTOR_COUNT] = {0};
// 准备展示的文本
int8_t comingWords[MOTOR_COUNT] = {0};
// 各个模块移动到指定字符所需要的脉冲数
int needPulse[MOTOR_COUNT] = {};
bool motorEnable[MOTOR_COUNT] = {false};
// 霍尔传感器状态
bool hallSensor[MOTOR_COUNT] = {false};

// 校准所有模块
bool allCalibration();
// 把线圈全部设为低电平，防止线圈持续通电，导致电机过热
// FIXME: 置零以后，电机的状态需要倒退一位嘛
void stopMotor();
// 展示指定文本
void showWords();
// SPI：输出 595 + 读取 165（同一时钟）
void spiTransfer();
// 单步推进（同步）
void stepOnce();
// // 上电归零（霍尔校准）
// void homeAllMotors()
// {
//     memset(motorEnable, true, sizeof(motorEnable));
//     while (true)
//     {
//         uint8_t hall = stepOnce();
//         bool done = true;
//         for (int i = 0; i < MOTOR_COUNT; i++)
//         {
//             if (hall & (1 << i))
//             {
//                 motorEnable[i] = false;
//                 motorChar[i] = 0;
//             }
//             else
//             {
//                 done = false;
//             }
//         }
//         delay(5);
//         if (done)
//             break;
//     }
//     // spiTransfer595_165(0); // 全断电
// }
// // 转到指定字符（含“每圈自动校准”）
// // void moveToChar(int idx, int targetChar)
// // {
// //     int diff = (targetChar - motorChar[idx] + CHAR_COUNT) % CHAR_COUNT;
// //     motorEnable[idx] = true;
// //     for (int i = 0; i < diff; i++)
// //     {
// //         motorStep[idx] = (motorStep[idx] + 1) % 8;
// //         spiTransfer595_165(buildMotorBits());
// //         delay(5);
// //         motorChar[idx]++;
// //         if (motorChar[idx] >= CHAR_COUNT)
// //         {
// //             motorChar[idx] = 0;
// //             // ===== 自动校准 =====
// //             while (true)
// //             {
// //                 uint8_t hall = spiTransfer595_165(buildMotorBits());
// //                 if (hall & (1 << idx))
// //                     break;
// //                 motorStep[idx] = (motorStep[idx] + 1) % 8;
// //                 delay(5);
// //             }
// //         }
// //     }
// //     motorChar[idx] = targetChar;
// //     motorEnable[idx] = false;
// //     spiTransfer595_165(buildMotorBits());
// // }

void setup()
{
    Serial.begin(9600);
    // WiFi.begin(ssid, password);
    // while (WiFi.status() != WL_CONNECTED)
    // {
    //     delay(500);
    //     Serial.print(".");
    // }
    // wifiState = WIFI_CONNECTED;
    // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    vspi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    delay(500);
    if (allCalibration())
    {
        Serial.println("all motors calibration succeed");
    }
    else
    {
        while (true)
            ;
    }
}

void loop()
{
    // wifiTask();
    // getCurrentTime();
    // Serial.println("");
    // FIXME: 升序不用加一，降序需要，可能是因为零点位置有变
    int temp[4] = {1, 12, 9, 7}; // lhf
    // int temp[4] = {1, 8, 11, 26};//hkz
    // int temp[4] = {1,20,25,21};//tyt
    // int temp[4] = {1, 12, 8, 24}; // lgw
    for (int i = 0; i < 4; i++)
    {
        comingWords[0] = temp[i];
        showWords();
        Serial.println(i);
        delay(5000);
    }
    Serial.println("finaly circle");
    delay(20000);
}

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

void showWords()
{
    // 计算各个模块移动到指定字符所需要的脉冲数
    bool firstHall[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        firstHall[i] = false;
        needPulse[i] = (comingWords[i] - currentWords[i] + CHAR_COUNT) % CHAR_COUNT;
        needPulse[i] *= PULSE_CHAR;
    }
    // FIXME:可能存在无限循环，要想办法规避
    while (needPulse[0] > 0 || needPulse[1] > 0 || needPulse[2] > 0 || needPulse[3] > 0 || needPulse[4] > 0)
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            // FIXME: 如果在旋转的过程中检测到零点，需要重新计算脉冲数，以便校准
            if (needPulse[i] > 0)
            {
                motorEnable[i] = true;
                needPulse[i]--;
            }
            else
            {
                motorEnable[i] = false;
            }
        }
        stepOnce();
        // 检测是否有模块在移动的过程中检测到霍尔传感器，如果检测到，那就重新计算还需要多少次脉冲
        // 但是一个模块只有第一次检测到霍尔传感器的时候才需要校验，下一次就不校验了(一个脉冲不足以让电机脱离霍尔传感器的检测)
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            if (hallSensor[i] && firstHall[i] == false)
            {
                firstHall[i] = true;
                needPulse[i] = (int)comingWords[i] * PULSE_CHAR;
            }
        }
    }
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        currentWords[i] = comingWords[i];
    }
    stopMotor();
}

void spiTransfer()
{
    uint32_t out = 0;
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        hallSensor[i] = false;
        // 不需要移动的步进电机就置零就好
        if (motorEnable[i])
        {
            for (int j = 3; j >= 0; j--)
            {
                if (stepTable[motorStep[i]] & (1 << j))
                {
                    out |= (1 << motorTable[i][3 - j]);
                }
            }
        }
    }

    uint8_t hall;
    digitalWrite(PIN_CS, LOW);
    // 好像应该获取第一个的hall吧？还是最后一个的hall
    hall = vspi.transfer((out >> 16) & 0xFF);
    // 第一个霍尔传感器检测到了
    if (hall == 0)
    {
        hallSensor[0] = true;
    }
    vspi.transfer((out >> 8) & 0xFF);

    vspi.transfer(out & 0xFF);
    delay(motorSpeed);
    digitalWrite(PIN_CS, HIGH);
    hall = vspi.transfer(0);
    for (int i = 0; i < 4; i++)
    {
        if (hall & (1 << (7 - i)))
        {
            continue;
        }
        hallSensor[i + 1] = true;
    }
}

void stepOnce()
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (motorEnable[i])
        {
            motorStep[i] = (motorStep[i] + 1) % 8;
        }
    }

    spiTransfer();
}

bool allCalibration()
{
    int cnt = 0;
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        motorEnable[i] = true;
    }
    while (true)
    {
        cnt++;
        // 转三圈还是解决不了直接退出
        if (cnt > PULSE_COUNT * 3)
        {
            return false;
        }
        stepOnce();
        // FIXME: 调试第一个电机而已
        for (int i = 0; i < 1; i++)
        {
            if (hallSensor[i])
            {
                motorEnable[i] = false;
            }
        }
        // FIXME:
        // if (motorEnable[0] == false && motorEnable[1] == false && motorEnable[2] == false && motorEnable[3] == false && motorEnable[4] == false)
        if (motorEnable[0] == false)
        {
            break;
        }
    }
    stopMotor();
    return true;
}

void stopMotor()
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        motorEnable[i] = false;
    }
    spiTransfer();
}