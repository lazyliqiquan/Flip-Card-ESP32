#include "Motor.h"

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
// 由于安装霍尔传感器的时候需要手动掰弯元器件，所以难免会有误差，但是误差通常在一个字符之前，所以需要通过软件校准
// 如果不知道怎么手动调整安装造成的误差，原点是0，负数表示前面的字符，正数表示后面即将显示的字符
const int mistake[5] = {0, PULSE_CHAR - 30, 0, 0, -PULSE_CHAR + 30};
// 霍尔传感器的检测范围，比如距离原点较近的字符，也有可能会被检测为原点
const int hallScope = 5;


void showWords()
{
    // 计算各个模块移动到指定字符所需要的脉冲数
    bool firstHall[MOTOR_COUNT], initialHall[MOTOR_COUNT];
    // 如果当前位置的磁铁能够被检测到，那么就不能将当前位置默认为原点，起码要转转动一定的距离后才得
    const int minPulses = hallScope * PULSE_CHAR;
    // 记录电机的转动脉冲数
    int rotationalPulse[5];
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        // 记录还没有开始转动的时候是不是就已经处于霍尔传感器的检测范围了，如果是，那么就不能将当前位置默认为原点，起码要转转动一定的距离后才得
        firstHall[i] = false;
        initialHall[i] = hallSensor[i];
        needPulse[i] = (comingWords[i] - currentWords[i] + CHAR_COUNT) % CHAR_COUNT * PULSE_CHAR;
        rotationalPulse[i] = 0;
    }
    while (needPulse[0] > 0 || needPulse[1] > 0 || needPulse[2] > 0 || needPulse[3] > 0 || needPulse[4] > 0)
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            // FIXME: 如果在旋转的过程中检测到零点，需要重新计算脉冲数，以便校准
            if (needPulse[i] > 0)
            {
                motorEnable[i] = true;
                needPulse[i]--;
                rotationalPulse[i]++;
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
            // 检测到磁铁
            if (hallSensor[i])
            {
                // 需要校准条件一：初始位置也检测到磁铁，但是已经转动了一定的距离
                // 需要校准条件二：初始位置检测不到磁铁
                if (((initialHall[i] && rotationalPulse[i] > minPulses) || initialHall[i] == false) && firstHall[i] == false)
                {
                    firstHall[i] = true;
                    // 消除误差
                    needPulse[i] = ((int)comingWords[i] * PULSE_CHAR - mistake[i] + PULSE_COUNT) % PULSE_COUNT;
                }
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

/*
校准的时候有一个目标，就是霍尔传感器检测到磁铁的时候，磁铁最好在霍尔传感器的最右侧，就会减少造成的误差了，
所以如果一开始就检测到磁铁，最好还是多转一圈，保证检测到磁铁的时候，是在霍尔传感器的最右侧
*/
bool allCalibration()
{
    int rotationalPulse[5];
    int cnt = 0;
    // 至少要转动一定的距离以后检测到磁铁才能保证此时磁铁在霍尔传感器的最右侧
    const int minPulses = hallScope * PULSE_CHAR;
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        rotationalPulse[i] = 0;
        motorEnable[i] = true;
    }
    while (motorEnable[0] || motorEnable[1] || motorEnable[2] || motorEnable[3] || motorEnable[4])
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            // 有一个问题，就是初始化校准的时候，某个模块的磁铁在霍尔传感器的左侧，会造成误差
            //  最好还是转一段距离，从头开始校准
            if (hallSensor[i] && rotationalPulse[i] > minPulses)
            {
                motorEnable[i] = false;
            }
            rotationalPulse[i]++;
        }
        stepOnce();
        cnt++;
        // 转三圈还是校准不了直接退出
        if (cnt > PULSE_COUNT * 3)
        {
            return false;
        }
    }
    int temp[5] = {};
    // 消除原点误差，安装过程中无法避免的误差（掰霍尔传感器的时候手抖难免的）
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        temp[i] = (PULSE_COUNT - mistake[i]) % PULSE_COUNT;
    }
    Serial.printf("%d   %d  %d  %d  %d\n", temp[0], temp[1], temp[2], temp[3], temp[4]);
    while (true)
    {
        bool flag = false;
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            if (temp[i] > 0)
            {
                temp[i]--;
                motorEnable[i] = true;
                flag = true;
            }
            else
            {
                motorEnable[i] = false;
            }
        }
        if (flag)
        {
            stepOnce();
        }
        else
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