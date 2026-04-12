#include <WiFi.h>
#include <PubSubClient.h>
#include <TimeLib.h>

// ===================== 请修改这里的配置 =====================
const char* ssid = "你的WiFi名称";
const char* password = "你的WiFi密码";

// MQTT 配置
const char* mqtt_server = "服务器IP或域名";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_pass = "";
const char* mqtt_client_id = "esp32-clock-001";

// MQTT 主题
const char* sub_topic = "clock/control";   // 接收指令
const char* pub_topic = "clock/status";    // 上报状态

// NTP 时间服务器（无需修改）
const char* ntp_server = "pool.ntp.org";
const long gmt_offset = 8 * 3600; // 东八区=8
const int daylight_offset = 0;

// 控制引脚
#define PIN_UP 18
#define PIN_DOWN 19
// ============================================================

WiFiClient espClient;
PubSubClient client(espClient);

// 时间相关
unsigned long last_sync = 0;
unsigned long last_minute_trigger = 0;
bool synced = false;

// 函数声明
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void sync_ntp();
void send_status(const char* status);
void trigger_minute_action();

void setup() {
  Serial.begin(115200);

  // 初始化引脚
  pinMode(PIN_UP, OUTPUT);
  pinMode(PIN_DOWN, OUTPUT);
  digitalWrite(PIN_UP, LOW);
  digitalWrite(PIN_DOWN, LOW);

  setup_wifi();
  sync_ntp(); // 上电同步时间

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  // MQTT 重连
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // 每小时自动同步一次 NTP（防止误差）
  if (millis() - last_sync > 3600000 && WiFi.status() == WL_CONNECTED) {
    sync_ntp();
    last_sync = millis();
  }

  // ===================== 每分钟自动驱动一次 =====================
  if (synced && millis() - last_minute_trigger >= 60000) {
    trigger_minute_action();
    last_minute_trigger = millis();
  }
}

// ===================== WiFi 连接 =====================
void setup_wifi() {
  delay(10);
  Serial.print("连接WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi 连接成功");
  Serial.println("IP: " + WiFi.localIP().toString());
}

// ===================== MQTT 重连 =====================
void reconnect() {
  while (!client.connected()) {
    Serial.print("连接MQTT...");
    if (client.connect(mqtt_client_id, mqtt_user, mqtt_pass)) {
      Serial.println("成功");
      client.subscribe(sub_topic);
      send_status("online"); // 上线后上报状态
    } else {
      Serial.print("失败，原因：");
      Serial.print(client.state());
      Serial.println(" 5秒后重试");
      delay(5000);
    }
  }
}

// ===================== 接收指令 =====================
void callback(char* topic, byte* payload, unsigned int length) {
  char msg[length + 1];
  for (int i = 0; i < length; i++) msg[i] = (char)payload[i];
  msg[length] = '\0';

  Serial.print("收到指令: ");
  Serial.println(msg);

  if (strcmp(msg, "UP") == 0 || strcmp(msg, "up") == 0) {
    digitalWrite(PIN_UP, HIGH);
    delay(100);
    digitalWrite(PIN_UP, LOW);
    send_status("action: up");
  }
  else if (strcmp(msg, "DOWN") == 0 || strcmp(msg, "down") == 0) {
    digitalWrite(PIN_DOWN, HIGH);
    delay(100);
    digitalWrite(PIN_DOWN, LOW);
    send_status("action: down");
  }
}

// ===================== NTP 时间同步 =====================
void sync_ntp() {
  Serial.println("同步 NTP 时间...");
  configTime(gmt_offset, daylight_offset, ntp_server);

  time_t now = time(nullptr);
  if (now < 86400) {
    Serial.println("NTP 同步失败");
    synced = false;
    return;
  }

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  Serial.println("NTP 同步成功");
  Serial.print("当前时间: ");
  Serial.print(timeinfo.tm_hour);
  Serial.print(":");
  Serial.println(timeinfo.tm_min);
  synced = true;
}

// ===================== 每分钟自动触发 =====================
void trigger_minute_action() {
  Serial.println("【每分钟自动动作】驱动翻页");
  digitalWrite(PIN_UP, HIGH);
  delay(100);
  digitalWrite(PIN_UP, LOW);
  send_status("auto_minute_triggered");
}

// ===================== MQTT 上报状态 =====================
void send_status(const char* status) {
  if (!client.connected()) return;

  char msg[128];
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  // 状态格式：时间 + 信息
  snprintf(msg, sizeof(msg), "[%02d:%02d:%02d] %s",
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, status);

  client.publish(pub_topic, msg);
  Serial.print("已上报: ");
  Serial.println(msg);
}