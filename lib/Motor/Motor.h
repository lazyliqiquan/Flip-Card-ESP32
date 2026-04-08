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
// 当前展示的文本
extern int8_t currentWords[MOTOR_COUNT];
// 准备展示的文本
extern int8_t comingWords[MOTOR_COUNT];

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

#endif