#include <WiFiS3.h>
#include <WiFiClient.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <math.h>

// --- LDR & SERVO PINS ---
#define LDR_TL A0  // 光敏电阻：左上
#define LDR_TR A1  // 光敏电阻：右上
#define LDR_BL A2  // 光敏电阻：左下
#define LDR_BR A3  // 光敏电阻：右下

#define PAN_SERVO_PIN 5   // 水平舵机引脚
#define TILT_SERVO_PIN 6  // 俯仰舵机引脚

// --- WIFI CONFIGURATION ---
const char* WIFI_SSID = "Free_WiFi";
const char* WIFI_PASS = "Bob@1357924680";

// --- LOCATION ---
// 纬度 (LATITUDE) = 52.378753
// 经度 (LONGITUDE) = -1.570225 (负数表示西经)
// 此固定坐标用于绕过浏览器 Geolocation 错误
const double LATITUDE = 52.378753;
const double LONGITUDE = -1.570225;

// --- API HOSTS ---
const char* SOLUNAR_HOST = "api.sunrise-sunset.org";
const char* TIME_HOST = "worldtimeapi.org";

// --- TRACKING CONSTANTS ---
const float AZ_GAIN = 0.015;
const float EL_GAIN = 0.015;
const float LDR_DEAD_BAND = 15.0;
const float SOLAR_CORRECTION_RATE = 0.005;

const float NIGHT_MODE_THRESHOLD = -2.0;  // 太阳高度角低于此值进入夜间模式
const float TILT_MAX = 90.0;
const float TILT_MIN = 0.0;
const float PAN_MAX = 180.0;
const float PAN_MIN = 0.0;
const float NIGHT_TILT_ANGLE = 5.0;  // 夜间俯仰到此角度

// --- INTERVALS (milliseconds) ---
const unsigned long SOLAR_CALC_INTERVAL = 5000;       // 太阳位置/时间同步间隔
const unsigned long SERVO_UPDATE_INTERVAL = 50;       // 舵机更新间隔
const unsigned long SOLUNAR_FETCH_INTERVAL_DAYS = 1;  // 每日数据获取间隔 (实际通过日期检查控制)

// --- GLOBAL VARIABLES ---
WiFiServer server(80);
Servo panServo, tiltServo;

float currentPan = 90.0;
float currentTilt = 45.0;

struct SolarAngles {
  double azimuth;
  double elevation;
};
SolarAngles lastSolar = { 0, 0 };

// UTC 时间结构体
struct TimeData {
  int Y = 0, M = 0, D = 0;
  int h = 0, min = 0, s = 0;
  String iso_sync = "";  // 用于发送给网页客户端进行本地实时时钟同步
  String toString() const {
    char buf[20];
    sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d", Y, M, D, h, min, s);
    return String(buf);
  }
};

TimeData g_currentTime = { 0, 0, 0, 0, 0, 0, "" };

// Solunar 数据结构体，包含日出日落时间
struct SolunarData {
  String sunrise = "--:--";
  String sunset = "--:--";
  String solarnoon = "--:--";
  int lastFetchDay = 0;  // 上次获取数据的日期
};
SolunarData dailySolunar;

unsigned long lastSolarCalc = 0;
unsigned long lastServoUpdate = 0;
bool isNightMode = true;

// --- HELPER FUNCTIONS ---

// 读取 LDR 模拟值 (0-1023)
int readLDR(int pin) {
  return analogRead(pin);
}

// 将方位角转换为舵机水平角 (0-180)
float azimuthToServo(double az_deg) {
  return constrain(az_deg / 2.0, PAN_MIN, PAN_MAX);
}

// 将高度角转换为舵机俯仰角 (0-90)
float elevationToServo(double el_deg) {
  return constrain(el_deg, TILT_MIN, TILT_MAX);
}

// 角度转弧度
double deg2rad(double d) {
  return d * M_PI / 180.0;
}
// 弧度转角度
double rad2deg(double r) {
  return r * 180.0 / M_PI;
}

// 将 12 小时制时间字符串 ("7:19:05 AM") 转换为 24 小时制 ("07:19")
String convert12hTo24h(String time12h) {
  if (time12h.length() < 7) return "--:--";
  time12h.trim();

  int hour = 0;
  int minute = 0;

  int firstColon = time12h.indexOf(':');
  int secondColon = time12h.indexOf(':', firstColon + 1);
  int space = time12h.indexOf(' ');

  if (firstColon == -1 || secondColon == -1 || space == -1) return "--:--";

  // 提取小时
  hour = time12h.substring(0, firstColon).toInt();
  // 提取分钟
  minute = time12h.substring(firstColon + 1, secondColon).toInt();

  // 判断是 PM
  bool isPM = time12h.indexOf("PM") != -1;

  if (isPM && hour < 12) {
    hour += 12;
  } else if (!isPM && hour == 12) {
    hour = 0;  // 12:xx AM 是 00:xx
  }

  char buf[6];
  sprintf(buf, "%02d:%02d", hour, minute);
  return String(buf);
}

// --- CORE CALCULATIONS ---

// 计算儒略日
double calcJulianDay(int Y, int M, int D, int h, int m, int s) {
  if (M <= 2) {
    Y -= 1;
    M += 12;
  }
  int A = Y / 100;
  int B = 2 - A + (A / 4);
  double dayFraction = (h + (m + s / 60.0) / 60.0) / 24.0;
  double jd = floor(365.25 * (Y + 4716)) + floor(30.6001 * (M + 1)) + D + dayFraction + B - 1524.5;
  return jd;
}

// 计算太阳在 UTC 时间点的方位角和高度角
SolarAngles solarPositionUTC(const TimeData& t, double lat, double lon) {
  double jd = calcJulianDay(t.Y, t.M, t.D, t.h, t.min, t.s);
  double n = jd - 2451545.0;
  double L = fmod(280.460 + 0.9856474 * n, 360.0);
  if (L < 0) L += 360.0;
  double g = fmod(357.528 + 0.9856003 * n, 360.0);
  if (g < 0) g += 360.0;
  double g_rad = deg2rad(g);
  double lambda = L + 1.915 * sin(g_rad) + 0.020 * sin(2 * g_rad);
  double lambda_rad = deg2rad(lambda);
  double eps = 23.439 - 0.0000004 * n;
  double eps_rad = deg2rad(eps);
  double alpha = atan2(cos(eps_rad) * sin(lambda_rad), cos(lambda_rad));
  double delta = asin(sin(eps_rad) * sin(lambda_rad));
  double JD0 = floor(jd + 0.5) - 0.5;
  double T = (JD0 - 2451545.0) / 36525.0;
  double GMST = fmod(280.46061837 + 360.98564736629 * (jd - 2451545.0) + 0.000387933 * T * T - (T * T * T) / 38710000.0, 360.0);
  if (GMST < 0) GMST += 360.0;
  double LST = fmod(GMST + lon, 360.0);
  if (LST < 0) LST += 360.0;
  double alpha_deg = rad2deg(alpha);
  if (alpha_deg < 0) alpha_deg += 360.0;
  double H = LST - alpha_deg;
  if (H < -180) H += 360;
  if (H > 180) H -= 360;
  double H_rad = deg2rad(H);
  double lat_rad = deg2rad(lat);
  double el_rad = asin(sin(lat_rad) * sin(delta) + cos(lat_rad) * cos(delta) * cos(H_rad));
  double elevation = rad2deg(el_rad);
  double az_rad = atan2(sin(H_rad), cos(H_rad) * sin(lat_rad) - tan(delta) * cos(lat_rad));
  double az_deg = rad2deg(az_rad) + 180.0;
  if (az_deg < 0) az_deg += 360.0;
  if (az_deg >= 360) az_deg -= 360.0;
  SolarAngles out;
  out.azimuth = az_deg;
  out.elevation = elevation;
  return out;
}

// --- API FETCH FUNCTIONS ---

// 获取当前 UTC 时间
bool fetchUTC(TimeData& t) {
  WiFiClient client;
  Serial.print("尝试连接时间API (");
  Serial.print(TIME_HOST);
  Serial.print(")...");

  if (!client.connect(TIME_HOST, 80)) {
    Serial.println("连接失败!");
    return false;
  }
  Serial.println("成功!");

  String req = "GET /api/ip HTTP/1.1\r\nHost: ";
  req += TIME_HOST;
  req += "\r\nConnection: close\r\n\r\n";
  client.print(req);

  unsigned long start = millis();
  while (!client.available() && millis() - start < 5000) delay(10);  // 等待响应

  if (!client.available()) {
    Serial.println("API响应超时.");
    client.stop();
    delay(100);
    return false;
  }

  // 忽略 HTTP Header
  while (client.available()) {
    String line = client.readStringUntil('\n');
    if (line.length() < 3 && line.indexOf('\r') != -1) break;
  }

  // 读取 JSON Body
  String json = client.readString();
  client.flush();  // 强制清除缓冲区
  client.stop();
  delay(100);  // 等待套接字清理

  if (json.length() < 50) {
    Serial.println("API响应太短或无效.");
    return false;
  }

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.print("UTC JSON 解析错误: ");
    Serial.println(error.c_str());
    return false;
  }

  if (!doc.containsKey("utc_datetime")) {
    Serial.println("UTC JSON 缺少 'utc_datetime' 字段.");
    return false;
  }

  String dt = doc["utc_datetime"].as<String>();
  t.iso_sync = dt;  // 存储完整的 ISO 字符串用于 JS 实时时钟

  t.Y = dt.substring(0, 4).toInt();
  t.M = dt.substring(5, 7).toInt();
  t.D = dt.substring(8, 10).toInt();
  t.h = dt.substring(11, 13).toInt();
  t.min = dt.substring(14, 16).toInt();
  t.s = dt.substring(17, 19).toInt();

  Serial.print("成功同步 UTC 时间: ");
  Serial.println(t.toString());
  return true;
}

// 获取日出日落数据
bool fetchSolunar(const TimeData& t) {
  // 仅在时间已同步（年份有效）且今天是新的日期时才获取
  if (t.Y == 0) {
    Serial.println("Solunar Fetch: UTC时间未同步, 跳过获取.");
    return false;
  }
  if (t.D == dailySolunar.lastFetchDay) {
    if (dailySolunar.sunrise != "--:--") {
      Serial.println("Solunar Fetch: 数据已获取，跳过获取.");
      return false;
    }
  }


  WiFiClient client;
  Serial.print("尝试连接 Solunar API (");
  Serial.print(SOLUNAR_HOST);
  Serial.print(")...");

  if (!client.connect(SOLUNAR_HOST, 80)) {
    Serial.println("连接失败!");
    return false;
  }
  Serial.println("成功!");

  char dateStr[11];
  sprintf(dateStr, "%04d-%02d-%02d", t.Y, t.M, t.D);

  // === 使用 sprintf 确保高精度坐标字符串格式稳定 ===
  char latStr[15];
  char lngStr[15];
  // 格式化为 6 位小数，确保 API 接受
  sprintf(latStr, "%.6f", LATITUDE);
  sprintf(lngStr, "%.6f", LONGITUDE);
  // =========================================================

  // 构建 API 路径
  String path = "/json?lat=";
  path += latStr;
  path += "&lng=";
  path += lngStr;
  path += "&date=";
  path += dateStr;

  client.print("GET " + path + " HTTP/1.1\r\n");
  client.print("Host: " + String(SOLUNAR_HOST) + "\r\n");
  client.print("Connection: close\r\n\r\n");

  unsigned long start = millis();
  while (!client.available() && millis() - start < 5000) delay(10);
  if (!client.available()) {
    client.stop();
    delay(100);
    Serial.println("Solunar API超时");
    return false;
  }

  // 忽略 HTTP Header
  while (client.available()) {
    String line = client.readStringUntil('\n');
    if (line.length() < 3 && line.indexOf('\r') != -1) break;
  }

  String json = client.readString();
  client.flush();  // 强制清除缓冲区
  client.stop();
  delay(100);  // 等待套接字清理

  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.print("Solunar JSON错误: ");
    Serial.println(error.c_str());
    return false;
  }

  String status = doc["status"].as<String>();
  if (status != "OK" || !doc.containsKey("results")) {
    Serial.print("Solunar API返回状态非OK或缺少结果字段: ");
    Serial.println(status);
    return false;
  }

  // 提取并转换 12 小时制时间
  String rawSunrise = doc["results"]["sunrise"].as<String>();
  String rawSunset = doc["results"]["sunset"].as<String>();
  String rawSolarNoon = doc["results"]["solar_noon"].as<String>();

  dailySolunar.sunrise = convert12hTo24h(rawSunrise);
  dailySolunar.sunset = convert12hTo24h(rawSunset);
  dailySolunar.solarnoon = convert12hTo24h(rawSolarNoon);

  dailySolunar.lastFetchDay = t.D;
  Serial.println("Solunar数据更新成功 (来自 api.sunrise-sunset.org)");

  return true;
}

// --- STATUS & HTTP SERVER ---

// 构建 JSON 状态字符串
String buildStatusJSON() {
  DynamicJsonDocument doc(1024);
  doc["lat"] = LATITUDE;
  doc["lon"] = LONGITUDE;

  if (g_currentTime.Y != 0) {
    doc["utc"] = g_currentTime.toString();
    doc["timestamp_sync"] = g_currentTime.iso_sync;
  } else {
    doc["utc"] = "Time Sync Failed";
  }

  doc["mode"] = isNightMode ? "夜间模式 (Night Mode)" : "追踪模式 (Tracking)";

  doc["solar"]["azimuth"] = lastSolar.azimuth;
  doc["solar"]["elevation"] = lastSolar.elevation;

  // 将日出日落数据包含到 JSON 状态中
  doc["solunar"]["sunrise"] = dailySolunar.sunrise;
  doc["solunar"]["sunset"] = dailySolunar.sunset;
  doc["solunar"]["solarnoon"] = dailySolunar.solarnoon;

  doc["servos"]["pan"] = currentPan;
  doc["servos"]["tilt"] = currentTilt;

  doc["ldr"]["tl"] = readLDR(LDR_TL);
  doc["ldr"]["tr"] = readLDR(LDR_TR);
  doc["ldr"]["bl"] = readLDR(LDR_BL);
  doc["ldr"]["br"] = readLDR(LDR_BR);

  String out;
  serializeJson(doc, out);
  return out;
}

// 处理 HTTP 请求
void handleClient(WiFiClient client) {
  String req = client.readStringUntil('\r');
  if (req.length() < 2) {
    client.stop();
    return;
  }
  int sp1 = req.indexOf(' '), sp2 = req.indexOf(' ', sp1 + 1);
  String path = req.substring(sp1 + 1, sp2);
  while (client.available()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }

  if (path == "/") {
    // 网页 HTML 内容
    String html = R"rawliteral(
<!doctype html>
<html>
<head>
<meta charset='utf-8'>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>太阳能追踪器控制台</title>
<script src="https://cdn.tailwindcss.com"></script>
<style>
/* 确保字体和 LDR 条样式一致 */
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@100..900&display=swap');
body { 
    font-family: 'Inter', sans-serif; 
    background-color: #f7f9fc;
}
.ldr-bar-container { 
    height: 18px; 
    background: #e0e0e0; 
    border-radius: 4px; 
    overflow: hidden; 
    margin: 4px 0; 
}
.ldr-bar { 
    height: 100%; 
    transition: width 0.3s ease-out; 
    border-radius: 4px; /* 确保条纹也有圆角 */
}
/* 卡片圆角和阴影 */
.bg-white { border-radius: 0.75rem; } 
</style>
</head>
<body class="bg-gray-50 font-sans p-4 sm:p-8">
  <div class="max-w-4xl mx-auto space-y-8">
    <h2 class="text-4xl font-extrabold text-indigo-700 mb-8 text-center border-b-4 border-indigo-200 pb-3">
        ☀️ Arduino R4 Solar Tracker 卫星控制台 🛰️
    </h2>

    <!-- 系统状态和时间同步 -->
    <div class="bg-white p-6 rounded-xl shadow-2xl border-t-4 border-indigo-500">
        <div class="flex flex-col sm:flex-row justify-between items-center mb-4">
            <h3 class="text-xl font-bold text-gray-700">系统状态</h3>
            <p id="system_mode" class="text-sm font-semibold px-3 py-1 rounded-full"></p>
        </div>
        <p class="text-center text-gray-600 text-sm">当前 UTC 时间: <span id="current_utc" class="font-mono text-base text-black">Loading...</span></p>
    </div>

    <!-- 每日太阳事件 (保留并优化样式) -->
    <div class="bg-white p-6 rounded-xl shadow-xl">
        <h3 class="text-xl font-semibold text-gray-700 mb-4 border-b pb-2">每日太阳事件 (本地时间 - Sunrise-Sunset API)</h3>
        <div class="grid grid-cols-3 gap-4 text-center">
            <div class="p-4 bg-blue-50 rounded-lg shadow-inner">
                <p class="text-sm text-gray-500">日出 (Sunrise)</p>
                <p id="sol_sunrise" class="text-2xl font-bold text-green-600">--:--</p>
            </div>
            <div class="p-4 bg-yellow-50 rounded-lg shadow-inner">
                <p class="text-sm text-gray-500">日中 (Solar Noon)</p>
                <p id="sol_noon" class="text-2xl font-bold text-yellow-700">--:--</p>
            </div>
            <div class="p-4 bg-red-50 rounded-lg shadow-inner">
                <p class="text-sm text-gray-500">日落 (Sunset)</p>
                <p id="sol_sunset" class="text-2xl font-bold text-red-600">--:--</p>
            </div>
        </div>
    </div>

    <!-- 太阳位置和舵机角度 -->
    <div class="grid grid-cols-1 md:grid-cols-2 gap-6">
        <div class="bg-white p-6 rounded-xl shadow-xl border-l-4 border-indigo-400">
            <h3 class="text-xl font-semibold text-gray-700 mb-4 border-b pb-2">计算太阳位置 (实时)</h3>
            <p class="text-lg mb-2">方位角 (Azimuth): <span id="az" class="font-mono text-2xl text-indigo-600">0</span>°</p>
            <p class="text-lg">高度角 (Elevation): <span id="el" class="font-mono text-2xl text-indigo-600">0</span>°</p>
        </div>
        <div class="bg-white p-6 rounded-xl shadow-xl border-l-4 border-pink-400">
            <h3 class="text-xl font-semibold text-gray-700 mb-4 border-b pb-2">舵机角度 (Pan/Tilt)</h3>
            <p class="text-lg mb-2">水平角 (Pan): <span id="pan" class="font-mono text-2xl text-pink-600">0</span>°</p>
            <p class="text-lg">俯仰角 (Tilt): <span id="tilt" class="font-mono text-2xl text-pink-600">0</span>°</p>
        </div>
    </div>

    <!-- LDR 传感器读数 -->
    <div class="bg-white p-6 rounded-xl shadow-xl">
        <h3 class="text-xl font-semibold text-gray-700 mb-4 border-b pb-2">LDR 光照传感器读数 (0-1023)</h3>
        <div class="overflow-x-auto">
            <table class="min-w-full divide-y divide-gray-200">
                <thead>
                    <tr class="bg-gray-50">
                        <th class="px-3 py-3 text-left text-sm font-medium text-gray-500 uppercase tracking-wider">传感器</th>
                        <th class="px-3 py-3 text-left text-sm font-medium text-gray-500 uppercase tracking-wider">读数</th>
                        <th class="px-3 py-3 text-left text-sm font-medium text-gray-500 uppercase tracking-wider">光照水平</th>
                    </tr>
                </thead>
                <tbody class="bg-white divide-y divide-gray-200 text-left">
                    <tr class="hover:bg-gray-50"><td>左上 (Top-Left)</td><td id="tl" class="font-mono">0</td><td><div class="ldr-bar-container"><div id="tlbar" class="ldr-bar bg-yellow-400"></div></div></td></tr>
                    <tr class="hover:bg-gray-50"><td>右上 (Top-Right)</td><td id="tr" class="font-mono">0</td><td><div class="ldr-bar-container"><div id="trbar" class="ldr-bar bg-yellow-400"></div></div></td></tr>
                    <tr class="hover:bg-gray-50"><td>左下 (Bottom-Left)</td><td id="bl" class="font-mono">0</td><td><div class="ldr-bar-container"><div id="blbar" class="ldr-bar bg-yellow-400"></div></div></td></tr>
                    <tr class="hover:bg-gray-50"><td>右下 (Bottom-Right)</td><td id="br" class="font-mono">0</td><td><div class="ldr-bar-container"><div id="brbar" class="ldr-bar bg-yellow-400"></div></div></td></tr>
                </tbody>
            </table>
        </div>
    </div>

  </div>
<script>
const LDR_IDS = ['tl', 'tr', 'bl', 'br'];
let lastSyncDate = null; 

// 格式化 UTC 时间字符串
function formatUtcTime(date) {
    const Y = date.getUTCFullYear();
    const M = String(date.getUTCMonth() + 1).padStart(2, '0');
    const D = String(date.getUTCDate()).padStart(2, '0');
    const h = String(date.getUTCHours()).padStart(2, '0');
    const m = String(date.getUTCMinutes()).padStart(2, '0');
    const s = String(date.getUTCSeconds()).padStart(2, '0');
    return `${Y}-${M}-${D} ${h}:${m}:${s}`;
}

// 客户端时钟每秒更新一次，用于平滑显示
function updateClientClock() {
    const utcElement = document.getElementById('current_utc');
    if (lastSyncDate) {
        // 增加 1 秒
        lastSyncDate.setSeconds(lastSyncDate.getSeconds() + 1);
        
        // 显示新时间
        utcElement.innerText = formatUtcTime(lastSyncDate);
        utcElement.classList.remove('text-red-600', 'font-bold');
        utcElement.classList.add('text-black');
    } else {
        // 如果从未同步成功，则显示失败信息
        utcElement.innerText = "Time Sync Failed (Waiting for Arduino Sync...)";
        utcElement.classList.add('text-red-600', 'font-bold');
        utcElement.classList.remove('text-black');
    }
}

// 定期从 Arduino 获取最新状态
async function updateStatus(){
    try {
        let resp = await fetch('/status');
        if (!resp.ok) throw new Error('Network response was not ok');
        let data = await resp.json();

        // --- 1. 处理时间同步和时钟基础 ---
        if (data.timestamp_sync) {
            const newSyncTime = new Date(data.timestamp_sync);
            // 只有当本地时钟为空或新的同步时间明显大于本地时钟时才更新基准，避免网络延迟导致时间倒退
            if (lastSyncDate === null || newSyncTime.getTime() > lastSyncDate.getTime() + 10000) {
                 lastSyncDate = newSyncTime;
            } else if (Math.abs(newSyncTime.getTime() - lastSyncDate.getTime()) > 5000) {
                 // 如果偏差过大，也进行校准
                 lastSyncDate = newSyncTime;
            }

        } else {
            lastSyncDate = null;
        }


        // --- 2. 更新其他状态 ---
        const modeElement = document.getElementById('system_mode');
        modeElement.innerText = data.mode;
        modeElement.className = data.mode.includes('夜间')
            ? 'text-sm font-semibold px-3 py-1 rounded-full bg-gray-600 text-white'
            : 'text-sm font-semibold px-3 py-1 rounded-full bg-green-500 text-white';

        // === 太阳事件数据 (日出日落) ===
        document.getElementById('sol_sunrise').innerText = data.solunar.sunrise || '--:--';
        document.getElementById('sol_noon').innerText = data.solunar.solarnoon || '--:--';
        document.getElementById('sol_sunset').innerText = data.solunar.sunset || '--:--';
        
        document.getElementById('az').innerText = data.solar.azimuth.toFixed(1);
        document.getElementById('el').innerText = data.solar.elevation.toFixed(1);

        document.getElementById('pan').innerText = data.servos.pan.toFixed(1);
        document.getElementById('tilt').innerText = data.servos.tilt.toFixed(1);

        LDR_IDS.forEach(id => {
            let val = data.ldr[id];
            document.getElementById(id).innerText = val;
            let widthPercent = Math.min((val / 1023) * 100, 100) + '%';
            document.getElementById(id + 'bar').style.width = widthPercent;
        });

    } catch (error) {
        console.error("无法获取状态:", error);
    }
}

// 启动客户端时钟
setInterval(updateClientClock, 1000); 

// 定期获取状态数据 (用于更新 LDR, Servo, Solar 计算结果和时间基准)
setInterval(updateStatus, 2000); 
updateStatus();
</script>
</body>
</html>
)rawliteral";

    client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: ");
    client.print(html.length());
    client.print("\r\nConnection: close\r\n\r\n");
    client.print(html);
  } else if (path == "/status") {
    String j = buildStatusJSON();
    client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: ");
    client.print(j.length());
    client.print("\r\nConnection: close\r\n\r\n");
    client.print(j);
  } else {
    client.print("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n");
  }
  delay(1);
  client.stop();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LDR_TL, INPUT);
  pinMode(LDR_TR, INPUT);
  pinMode(LDR_BL, INPUT);
  pinMode(LDR_BR, INPUT);
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
  panServo.write((int)currentPan);
  tiltServo.write((int)currentTilt);

  Serial.print("正在连接 Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(200);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    int attempts = 0;
    while (WiFi.localIP() == IPAddress(0, 0, 0, 0) && attempts++ < 20 && millis() - start < 20000) {
      delay(500);
      Serial.print("#");
    }

    Serial.println("\nWi-Fi 连接成功");
    Serial.print("IP 地址: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWi-Fi 连接失败");
  }

  server.begin();
  lastSolarCalc = 0;
  lastServoUpdate = 0;
  dailySolunar.lastFetchDay = 0;
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    handleClient(client);
  }

  unsigned long now = millis();

  bool shouldRecalculateSolar = false;

  // 定期执行太阳位置计算、时间同步和 Solunar API 调用
  if (now - lastSolarCalc >= SOLAR_CALC_INTERVAL) {

    if (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
      Serial.println("Wi-Fi 或 IP 地址丢失，尝试重新连接...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      unsigned long reconnectStart = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - reconnectStart < 10000) {
        delay(500);
        Serial.print("#");
      }
      if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
        Serial.println("\n重新连接成功. IP: " + WiFi.localIP().toString());
        shouldRecalculateSolar = true;
      } else {
        Serial.println("\n重新连接失败。强制夜间模式。");
        isNightMode = true;
      }
    } else {
      shouldRecalculateSolar = true;
    }

    if (shouldRecalculateSolar) {
      TimeData newTime;
      // 必须先获取时间，因为太阳位置和 Solunar API 都依赖于当前时间
      if (fetchUTC(newTime)) {
        g_currentTime = newTime;
        lastSolar = solarPositionUTC(g_currentTime, LATITUDE, LONGITUDE);

        // 检查是否从失败状态恢复或首次成功
        bool wasNightMode = isNightMode;
        isNightMode = lastSolar.elevation < NIGHT_MODE_THRESHOLD;

        if (wasNightMode && !isNightMode) {
          Serial.println("退出夜间模式，开始追踪。");
        }

        // 获取日出日落时间，它会自动检查是否是新的一天
        fetchSolunar(g_currentTime);
        Serial.println("太阳位置和 Solunar 数据更新完成。");

      } else {
        // 如果时间同步失败，强制进入夜间模式，直到下次同步成功。
        Serial.println("时间同步失败。强制夜间模式以避免误差累积。");
        isNightMode = true;
      }
    }
    lastSolarCalc = now;
  }

  // 定期更新舵机位置
  if (now - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {

    float targetPanFromSolar = azimuthToServo(lastSolar.azimuth);
    float targetTiltFromSolar = elevationToServo(lastSolar.elevation);

    if (isNightMode) {
      if (g_currentTime.Y == 0 || lastSolar.elevation < -15.0) {  // 仅在时间同步失败或夜晚深时将舵机移动到安全位置
        currentPan = PAN_MIN;
        currentTilt = NIGHT_TILT_ANGLE;
      }
    } else {
      // 读取 LDR 传感器
      int tl = readLDR(LDR_TL), tr = readLDR(LDR_TR);
      int bl = readLDR(LDR_BL), br = readLDR(LDR_BR);

      // 计算水平和垂直光强差
      int leftSum = tl + bl, rightSum = tr + br, azDiff = leftSum - rightSum;
      int topSum = tl + tr, botSum = bl + br, elDiff = topSum - botSum;

      // 计算 LDR 修正量
      float deltaPan = (abs(azDiff) > LDR_DEAD_BAND) ? (-azDiff * AZ_GAIN) : 0.0;
      float deltaTilt = (abs(elDiff) > LDR_DEAD_BAND) ? (elDiff * EL_GAIN) : 0.0;

      // 混合 LDR 误差修正和太阳位置校正
      currentPan += deltaPan + SOLAR_CORRECTION_RATE * (targetPanFromSolar - currentPan);
      currentTilt += deltaTilt + SOLAR_CORRECTION_RATE * (targetTiltFromSolar - currentTilt);

      // 限制在有效角度范围内
      currentPan = constrain(currentPan, PAN_MIN, PAN_MAX);
      currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);
    }

    // 写入舵机
    panServo.write((int)currentPan);
    tiltServo.write((int)currentTilt);

    lastServoUpdate = now;
  }
}