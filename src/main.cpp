/*
安装翻牌的顺序：原点就是0位点，往前数15个孔，就是空白翻牌上部分的安装位置
注意：安装误差最好保证在一个字符之内
代码思路是对的，就是有些没有考虑到，
【1】展示字符时：离原点比较近的字符容易被检测成原点，从而造成误差，因为每一圈都会检测是否经过原点来校准误差，下一步想要转动的字符，
离原点太近，反而被误以为是原点，从而不再转动。解决方法，离原点较近的字符不进行原点校验，使用相对距离作为转动距离；
或者可以多转一圈的距离，再次接触到原点的时候，磁铁就是在霍尔传感器的最右侧了
【2】初步校准时：多转一圈，确保当前校准位是在霍尔传感器的最右侧，因为后续每一圈的校验都是以最右侧也就是第一次检测到磁铁的位置来作为原点的

*/
#include <Arduino.h>
#include "Motor.h"
#include "MyWiFi.h"

void setup()
{
    Serial.begin(9600);
    // 生成唯一 clientID（多设备必备）
    sprintf(mqtt_client_id, "esp32-clock-%08X", (uint32_t)ESP.getEfuseMac());
    Serial.printf("mqtt client id : %s\n", mqtt_client_id);
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
    client.loop(); // 会不会下面的showWords太耗时了，导致这里断开（FIXME: 需要在showWords里面间歇调用一下嘛）

    // 每分钟自动触发
    if (millis() - last_minute_trigger >= refresh_time_interval && (show_status == 0 || show_status == 1))
    {
        parse_time();
        showWords();
    }
}
