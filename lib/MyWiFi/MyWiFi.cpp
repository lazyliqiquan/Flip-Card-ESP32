#include "MyWiFi.h"

// ===================== 请修改这里的配置 =====================
// WIFI用户名密码
const char *ssid = "LQQ";
const char *password = "qq123456";

uint8_t show_status = 0; // 默认展示时间
bool isBusy = false;     // 忙锁：true = 正在执行指令

// MQTT 配置
const char *mqtt_server = "112.124.52.31";
const int mqtt_port = 1883;
const char *mqtt_user = "lazyman";
const char *mqtt_pass = "lazyman";
char mqtt_client_id[50];

// MQTT 主题
const char *sub_topic = "clock/control"; // 接收指令
const char *pub_topic = "clock/status";  // 上报状态

// mqtt客户端实例化
WiFiClient espClient;
PubSubClient client(espClient);

// NTP 时间服务器（无需修改）
const char *ntp_server = "ntp.aliyun.com";
const long gmt_offset_sec = 8 * 3600; // 东八区 秒
const int daylight_offset_sec = 0;

// 更新时间参数
const int ONE_MINUTE = 60 * 1000;       // 1分钟的ms单位表现形式
int refresh_time_interval = ONE_MINUTE; // 默认一分钟
const uint8_t cost_time = 3;            // 从得到最新的时间，到展示出来所消耗的时间

// 时间状态
unsigned long last_minute_trigger = 0;

uint8_t timeArr[8] = {};                                        // 此前到后分别是month-day-hour-minute
void callback(char *topic, byte *payload, unsigned int length); // 接收指令
void getFutureTime(struct tm *timeinfo, int seconds);           // 获取 N 秒后的时间

// ===================== WiFi 连接 =====================
void setup_wifi()
{
    delay(10);
    Serial.print("连接WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi 连接成功");
    Serial.println("IP: " + WiFi.localIP().toString());
}
// ===================== 初始化MQTT配置 =====================
void init_mqtt()
{
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
}

// ===================== MQTT 重连 =====================
void reconnect()
{
    // 如果已经连接 MQTT，直接退出
    if (client.connected())
        return;

    // 🔥 第一步：先检查 WiFi！如果 WiFi 断了，直接返回，不卡在这里！
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi 已断开，等待重连...");
        return; // 必须 return！不能死循环等 WiFi
    }

    // 🔥 第二步：WiFi 正常，才尝试连接 MQTT
    Serial.println("尝试连接 MQTT 服务器...");

    // 连接 MQTT，关键：第6个参数 = false（清除会话，不收历史指令）
    if (client.connect(mqtt_client_id, mqtt_user, mqtt_pass, NULL, 0, false, NULL))
    {
        Serial.println("MQTT 连接成功！");
        client.subscribe(sub_topic); // 连接成功再订阅
    }
    else
    {
        Serial.print("MQTT 连接失败，原因：");
        Serial.println(client.state());
        delay(2000); // 失败等2秒
    }
}

// ===================== NTP 同步（仅使用 <time.h>） =====================
void sync_ntp()
{
    Serial.println("正在同步 NTP 时间...");

    // 官方原生 configTime，来自 <time.h> 系统库
    configTime(gmt_offset_sec, daylight_offset_sec, ntp_server);
    struct tm timeinfo;
    while (!getLocalTime(&timeinfo))
    {

        delay(1000);
        Serial.print(".");
    }
    // 解析获取到的时间
    parse_time();
}

// ===================== MQTT 上报状态（带时间） =====================
void send_status(const char *status)
{
    if (!client.connected())
        return;

    char buf[128];

    // 使用 # 键间隔，第一个是当前展示状态,
    snprintf(buf, sizeof(buf), "%d#%d#%s", status);

    client.publish(pub_topic, buf);
    Serial.printf("上报: %s\n", buf);
}

// ===================== 接收 MQTT 指令 =====================
// 0        设置为展示时间      return      00      前面的数字表示展示状态，后面的数字表示这条命令执行是否成功,0表示成功
// 1        设置为展示日期      return      10
// 2HELLO   设置为展示文本      return      20      [算式的答案也是交给服务器来发吧]
// 3        获取当前设备信息    return      0       当前的展示状态
void callback(char *topic, byte *payload, unsigned int length)
{
    // 必须先调用，维持 MQTT 连接！
    client.loop();
    // 核心：如果正在忙，直接丢弃新消息
    if (isBusy)
    {
        Serial.println("忙碌中，丢弃新指令");
        send_status return; // 直接返回，不处理
    }

    // 不忙 → 上锁，开始处理
    isBusy = true;
    Serial.println("开始处理指令...");

    // -------------------
    // 你的指令处理代码
    // -------------------

    // 执行完 → 解锁
    isBusy = false;
    Serial.println("指令处理完成，解锁");

    char msg[length + 1];
    msg[length] = '\0';
    for (int i = 0; i < length; i++)
    {
        msg[i] = (char)payload[i];
    }
    Serial.print("收到指令: ");
    Serial.println(msg);

    if (msg[0] == '1')
    {
        show_status = 1;
    }
    else if (msg[0] == '2')
    {
        show_status = 2;
    }
    else
    {
        show_status = 0;
    }
}

// ===================== 获取 N 秒后的时间 =====================
void getFutureTime(struct tm *timeinfo, int seconds)
{
    time_t now = time(nullptr);
    time_t future_time = now + seconds; // 时间 + N 秒
    // 转成结构体
    localtime_r(&future_time, timeinfo);
}

// ===================== 解析时间 =====================
void parse_time()
{
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    last_minute_trigger = millis();
    // 距离下一次 分钟位 变换的时间间隔
    uint8_t refresh_interval = 60 - timeinfo.tm_sec;
    // 设置下一次更新的时间间隔
    refresh_time_interval = refresh_interval;
    // 如果翻页时钟刚准备转动到准确时间，但是 分钟位 时间马上就要变换了，那么我直接转动到下一分钟
    if (refresh_interval < cost_time)
    {
        // 获取下一分钟的时间
        getFutureTime(&timeinfo, cost_time);
        // 现在我提前转动到下一个 分钟位，那么更新的时间间隔也需要加上一分钟
        refresh_time_interval += ONE_MINUTE;
    }
    timeArr[0] = (timeinfo.tm_mon + 1) / 10;
    timeArr[1] = (timeinfo.tm_mon + 1) % 10;
    timeArr[2] = timeinfo.tm_mday / 10;
    timeArr[3] = timeinfo.tm_mday % 10;
    timeArr[4] = timeinfo.tm_hour / 10;
    timeArr[5] = timeinfo.tm_hour % 10;
    timeArr[6] = timeinfo.tm_min / 10;
    timeArr[7] = timeinfo.tm_min % 10;
}