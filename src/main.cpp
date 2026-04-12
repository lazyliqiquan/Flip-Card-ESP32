/*
安装翻牌的顺序：原点就是0位点，往前数15个孔，就是空白翻牌上部分的安装位置
注意：安装误差最好保证在一个字符之内
代码思路是对的
就是有些没有考虑到，
【2】展示字符时：离原点比较近的字符容易被检测成原点，从而造成误差，因为每一圈都会检测是否经过原点来校准误差，下一步想要转动的字符，
离远点太近，反而被误以为是原点，从而不再转动，解决方法，离原点较近的字符这一圈不进行校验，使用相对距离作为转动距离；
或者可以多转一圈的距离，再次接触到原点的时候，磁铁就是在霍尔传感器的最右侧了
【3】校准时：多转一圈，确保当前校准位是在霍尔传感器的最右侧，因为后续每一圈的校验都是以最右侧也就是第一次检测到磁铁的位置来作为原点的

*/
#include <Arduino.h>
#include "Motor.h"
#include "MyWiFi.h"

void setup()
{
    Serial.begin(9600);
    // 生成唯一 clientID（多设备必备）
    sprintf(mqtt_client_id, "esp32-clock-%08X", (uint32_t)ESP.getEfuseMac());
    // 连接wifi
    setup_wifi();
    // 连接ntp服务器，同步当前时间
    sync_ntp();
    // 初始化mqtt服务器配置
    init_mqtt();
    // 初始化spi配置
    init_spi();
    delay(500);
    // 校准每一个翻牌模型
    if (allCalibration())
    {
        Serial.println("All motors calibration succeed");
    }
    else
    {
        Serial.println("Calibration failed");
        while (true)
            ;
    }
}

void loop()
{
    // 查看mqtt服务是否还保持连接
    reconnect();
    // 如果连接wifi失败，这里会直接跳过
    client.loop();

    // 每分钟自动触发
    if (millis() - last_minute_trigger >= refresh_time_interval)
    {
        parse_time();
        switch (show_status)
        {
        case 1:
            // 展示日期
            comingWords[5] = getCharPosition(char(timeArr[1] + '0'));
            comingWords[4] = getCharPosition(char(timeArr[2] + '0'));
            comingWords[3] = getCharPosition('-');
            comingWords[2] = getCharPosition(char(timeArr[3] + '0'));
            comingWords[1] = getCharPosition(char(timeArr[4] + '0'));
            break;
        case 0:
            // 展示时间
            comingWords[5] = getCharPosition(char(timeArr[4] + '0'));
            comingWords[4] = getCharPosition(char(timeArr[5] + '0'));
            comingWords[3] = getCharPosition(':');
            comingWords[2] = getCharPosition(char(timeArr[6] + '0'));
            comingWords[1] = getCharPosition(char(timeArr[7] + '0'));
            break;
        }
    }

    int temp[3][5] = {{0, 0, 0, 0, 0}, {8, 5, 12, 12, 15}, {13, 15, 21, 19, 5}};
    // 等下我发给你几个英文字母，你告诉我它们的位置，如a对应1，z对应26
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            comingWords[j] = temp[i][5 - 1 - j];
        }
        showWords();
        Serial.println(i);
        delay(5000);
    }
}
