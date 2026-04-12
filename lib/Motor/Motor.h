#ifndef MOTOR_H
#define MOTOR_H

#include <SPI.h>
#include <Arduino.h>
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

extern SPIClass vspi;
extern int8_t comingWords[MOTOR_COUNT]; // 准备展示的文本

void init_spi();              // 初始化spi配置
bool allCalibration();        // 校准所有模块
void showWords();             // 展示指定文本
int8_t getCharPosition(char c); // 获取字符在列表中的位置
#endif