/*
安装翻牌的顺序：原点就是0位点，往前数15个孔，就是空白翻牌上部分的安装位置
注意：安装误差最好保证在一个字符之内
代码思路是对的
就是有些没有考虑到，
【1】没有考虑电机不转动的时候，应该把控制线置为低电平，以免过热，记得记录上一步的状态，然后再置零
【2】展示字符时：离原点比较近的字符容易被检测成原点，从而造成误差，因为每一圈都会检测是否经过原点来校准误差，下一步想要转动的字符，
离远点太近，反而被误以为是原点，从而不再转动，解决方法，离远点较近的字符这一圈不进行校验，使用相对距离作为转动距离；
或者可以多转一圈的距离，再次接触到原点的时候，磁铁就是在霍尔传感器的最右侧了
【3】校准时：多转一圈，确保当前校准位是在霍尔传感器的最右侧，因为后续每一圈的校验都是以最右侧也就是第一次检测到磁铁的位置来作为原点的

*/
#include <Arduino.h>
#include "Motor.h"
#include "MyWiFi.h"

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
    // int temp[3][5] = {{0, 0, 0, 0, 0}, {8, 5, 12, 12, 15}, {0, 2, 18, 15, 0}};
    int temp[3][5] = {{0, 0, 0, 0, 0}, {8, 5, 12, 12, 15}, {13, 15, 21, 19, 5}};
    // 等下我发给你几个英文字母，你告诉我它们的位置，如a对应1，z对应26
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            comingWords[j] = temp[i][5 - 1 - j];
        }
        Serial.printf("%d   %d  %d  %d  %d\n", currentWords[0], currentWords[1], currentWords[2], currentWords[3], currentWords[4]);
        showWords();
        Serial.println(i);
        delay(5000);
    }
    Serial.println("finaly circle");
    delay(120000);
}
