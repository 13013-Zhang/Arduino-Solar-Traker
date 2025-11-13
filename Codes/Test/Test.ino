#include <WiFiS3.h>
#include <WiFiClient.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <math.h>

// --- LDR & SERVO PINS ---
#define LDR_TL A0 // 光敏电阻：左上
#define LDR_TR A1 // 光敏电阻：右上
#define LDR_BL A2 // 光敏电阻：左下
#define LDR_BR A3 // 光敏电阻：右下

#define PAN_SERVO_PIN 5 // 水平舵机引脚
#define TILT_SERVO_PIN 6 // 俯仰舵机引脚

// --- WIFI CONFIGURATION (直接定义凭证，不使用 arduino_secrets.h) ---
const char* WIFI_SSID = "Free_WiFi"; 
const char* WIFI_PASS = "Bob@1357924680";

// --- LOCATION (使用 float) ---
const float LATITUDE = 52.378753f;
const float LONGITUDE = -1.570225f;

// --- API HOSTS ---
const char* SOLUNAR_HOST = "api.sunrise-sunset.org";
const char* TIME_HOST = "worldtimeapi.org"; 

// --- TRACKING CONSTANTS (使用 float 后缀 'f') ---
const float AZ_GAIN = 0.015f;
const float EL_GAIN = 0.015f;
const float LDR_DEAD_BAND = 15.0f;
const float SOLAR_CORRECTION_RATE = 0.005f;

const float NIGHT_MODE_THRESHOLD = -2.0f; 
const float TILT_MAX = 90.0f;
const float TILT_MIN = 0.0f;
const float PAN_MAX = 180.0f;
const float PAN_MIN = 0.0f;
const float NIGHT_TILT_ANGLE = 5.0f; // 夜间俯仰到此角度

// --- INTERVALS (milliseconds) ---
const unsigned long SOLAR_CALC_INTERVAL = 10000; // 太阳位置/时间同步间隔
const unsigned long SERVO_UPDATE_INTERVAL = 50; // 舵机更新间隔

// --- GLOBAL VARIABLES ---
WiFiServer server(80);
Servo panServo, tiltServo;

float currentPan = 90.0f;
float currentTilt = 45.0f;

// 使用 float 代替 double
struct SolarAngles {
  float azimuth;
  float elevation;
};
SolarAngles lastSolar = {0.0f, 0.0f};

// UTC 时间结构体
struct TimeData {
  int Y = 0, M = 0, D = 0;
  int h = 0, min = 0, s = 0;
  String iso_sync = ""; 
  String toString() const {
    char buf[20];
    sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d", Y, M, D, h, min, s);
    return String(buf);
  }
};

TimeData g_currentTime = {0, 0, 0, 0, 0, 0, ""};

// Solunar 数据结构体
struct SolunarData {
  String sunrise = "--:--";
  String sunset = "--:--";
  String solarnoon = "--:--";
  int lastFetchDay = 0; 
};
SolunarData dailySolunar;

unsigned long lastSolarCalc = 0;
unsigned long lastServoUpdate = 0;
bool isNightMode = true; 

// --- HELPER FUNCTIONS ---

// 读取 LDR 模拟值 (0-1023)
int readLDR(int pin) { return analogRead(pin); }

// 将方位角转换为舵机水平角 (0-180)
float azimuthToServo(float az_deg) {
  return constrain(az_deg / 2.0f, PAN_MIN, PAN_MAX);
}

// 将高度角转换为舵机俯仰角 (0-90)
float elevationToServo(float el_deg) {
  return constrain(el_deg, TILT_MIN, TILT_MAX);
}

// 角度转弧度 (使用 float)
float deg2rad(float d) { return d * M_PI / 180.0f; }
// 弧度转角度 (使用 float)
float rad2deg(float r) { return r * 180.0f / M_PI; }

// 将 12 小时制时间字符串转换为 24 小时制 (优化：使用 sscanf)
String convert12hTo24h(String time12h) {
  if (time12h.length() < 7) return "--:--";
  
  char buf12h[20];
  time12h.toCharArray(buf12h, 20);

  int hour = 0;
  int minute = 0;
  char ampm[3]; 

  if (sscanf(buf12h, "%d:%d:%*d %2s", &hour, &minute, ampm) != 3) {
      if (sscanf(buf12h, "%d:%d %2s", &hour, &minute, ampm) != 3) {
          return "--:--"; 
      }
  }
  
  bool isPM = (ampm[0] == 'P' || ampm[0] == 'p');
  
  if (isPM && hour < 12) {
    hour += 12;
  } else if (!isPM && hour == 12) {
    hour = 0; 
  }
  
  char buf24h[6];
  sprintf(buf24h, "%02d:%02d", hour, minute);
  return String(buf24h);
}

// --- CORE CALCULATIONS ---

// 计算儒略日 (使用 double 保证精度)
double calcJulianDay(int Y, int M, int D, int h, int m, int s) {
  if (M <= 2) { Y -= 1; M += 12; }
  int A = Y / 100;
  int B = 2 - A + (A / 4);
  double dayFraction = (h + (m + s / 60.0) / 60.0) / 24.0;
  double jd = floor(365.25 * (Y + 4716)) + floor(30.6001 * (M + 1)) + D + dayFraction + B - 1524.5;
  return jd;
}

// 计算太阳在 UTC 时间点的方位角和高度角 (内部使用 double, 结果返回 float)
SolarAngles solarPositionUTC(const TimeData& t, float lat, float lon) {
  double jd = calcJulianDay(t.Y, t.M, t.D, t.h, t.min, t.s);
  double n = jd - 2451545.0;
  double L = fmod(280.460 + 0.9856474 * n, 360.0);
  if (L < 0) L += 360.0;
  double g = fmod(357.528 + 0.9856003 * n, 360.0);
  if (g < 0) g += 360.0;
  double g_rad = deg2rad((float)g);
  double lambda = L + 1.915 * sin(g_rad) + 0.020 * sin(2 * g_rad);
  double lambda_rad = deg2rad((float)lambda);
  double eps = 23.439 - 0.0000004 * n;
  double eps_rad = deg2rad((float)eps);
  double alpha = atan2(cos(eps_rad) * sin(lambda_rad), cos(lambda_rad));
  double delta = asin(sin(eps_rad) * sin(lambda_rad));
  double JD0 = floor(jd + 0.5) - 0.5;
  double T = (JD0 - 2451545.0) / 36525.0;
  double GMST = fmod(280.46061837 + 360.98564736629 * (jd - 2451545.0) + 0.000387933 * T * T - (T * T * T) / 38710000.0, 360.0);
  if (GMST < 0) GMST += 360.0;
  double LST = fmod(GMST + lon, 360.0);
  if (LST < 0) LST += 360.0;
  double alpha_deg = rad2deg((float)alpha); if (alpha_deg < 0) alpha_deg += 360.0;
  double H = LST - alpha_deg; if (H < -180) H += 360; if (H > 180) H -= 360;
  double H_rad = deg2rad((float)H);
  double lat_rad = deg2rad(lat);
  double el_rad = asin(sin(lat_rad) * sin(delta) + cos(lat_rad) * cos(delta) * cos(H_rad));
  double elevation = rad2deg((float)el_rad);
  double az_rad = atan2(sin(H_rad), cos(H_rad) * sin(lat_rad) - tan(delta) * cos(lat_rad));
  double az_deg = rad2deg((float)az_rad) + 180.0;
  if (az_deg < 0) az_deg += 360.0;
  if (az_deg >= 360) az_deg -= 360.0;
  
  SolarAngles out; 
  out.azimuth = (float)az_deg; 
  out.elevation = (float)elevation; 
  return out;
}

// --- NETWORK & API FUNCTIONS ---

// 尝试连接或重连 Wi-Fi 的封装函数
bool attemptReconnect(unsigned long timeout_ms = 15000) {
    if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
        return true;
    }
    
    Serial.println("\n尝试 Wi-Fi 重连...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeout_ms) {
        delay(500);
        Serial.print("#"); 
    }
    
    if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
        Serial.println("\n重连成功. IP: " + WiFi.localIP().toString());
        server.begin(); // 重新开始服务器
        return true;
    }
    
    Serial.println("\nWi-Fi 重连失败。");
    return false;
}

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
  
  // 核心修改：将超时时间从 5 秒增加到 10 秒
  unsigned long start = millis();
  while (!client.available() && millis() - start < 10000) delay(10); // <--- 修正点
  
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
  client.stop();
  delay(100); 

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
  t.iso_sync = dt; 
  
  // 提取时间部分
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
  // 仅在时间已同步且今天是新的日期时才获取
  if (t.Y == 0) {
    Serial.println("Solunar Fetch: UTC时间未同步, 跳过获取.");
    return false;
  }
  if (t.D == dailySolunar.lastFetchDay && dailySolunar.sunrise != "--:--") { 
    Serial.println("Solunar Fetch: 今日数据已获取，跳过获取.");
    return true; 
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

  // 1. 格式化日期字符串
  char dateStr[11];
  sprintf(dateStr, "%04d-%02d-%02d", t.Y, t.M, t.D);

  // 2. 核心修改：使用 String(float, decimalPlaces) 构造函数创建经纬度字符串
  // 确保了负号和浮点数格式的正确性
  String latStr = String(LATITUDE, 7); 
  String lngStr = String(LONGITUDE, 7); 

  // 3. 构建完整的请求路径 (使用 String 拼接)
  String path = "/json?lat=" + latStr + "&lng=" + lngStr + "&date=" + String(dateStr);
  
  // 打印完整的请求 URL 以便调试
  Serial.print("Solunar Request URL: http://");
  Serial.print(SOLUNAR_HOST);
  Serial.println(path); 
  
  // 4. 发送 GET 请求
  client.print("GET ");
  client.print(path); 
  client.print(" HTTP/1.1\r\n");
  client.print("Host: " + String(SOLUNAR_HOST) + "\r\n");
  client.print("Connection: close\r\n\r\n");
  
  String json = "";
  unsigned long startJsonRead = millis();
  
  while (client.connected() && millis() - startJsonRead < 5000) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      
      // 检查行是否包含 JSON 起始字符 {
      int jsonStart = line.indexOf('{');
      
      if (jsonStart != -1) {
        // 从 { 开始截取字符串，并将其作为 JSON 的第一部分
        json += line.substring(jsonStart);
        
        // 继续读取剩余的数据，直到客户端不再可用
        // 确保读取完整的 JSON 内容，避免遗漏
        while (client.available()) {
            json += client.readString();
        }
        break; // 找到并读取完毕，退出循环
      }
    } else {
      delay(10);
    }
  }

  client.stop();
  delay(100); 

  // 打印修正后的 JSON 字符串
  Serial.print("Solunar RAW JSON (CLEANED Length: "); Serial.print(json.length()); Serial.println("):");
  Serial.println(json); // 打印清理后的 JSON 字符串

  if (json.length() < 50) {
      Serial.println("API响应太短或无效.");
      return false;
  }
  
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
      // 捕获 API 自己的错误消息，以便进一步调试
      if (doc.containsKey("results")) {
           Serial.print("API Status: "); 
           Serial.println(status);
      } else {
           Serial.println("API 响应格式错误或缺少结果字段。");
      }
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
  doc["lat"] = LATITUDE; doc["lon"] = LONGITUDE;

  if (g_currentTime.Y != 0) {
      doc["utc"] = g_currentTime.toString();
      doc["timestamp_sync"] = g_currentTime.iso_sync; 
  } else {
      doc["utc"] = "Time Sync Failed";
  }

  doc["mode"] = isNightMode ? "夜间模式 (Night Mode)" : "追踪模式 (Tracking)";

  // 使用 roundf 控制 1 位小数精度
  doc["solar"]["azimuth"] = roundf(lastSolar.azimuth * 10.0f) / 10.0f;
  doc["solar"]["elevation"] = roundf(lastSolar.elevation * 10.0f) / 10.0f;

  doc["solunar"]["sunrise"] = dailySolunar.sunrise;
  doc["solunar"]["sunset"] = dailySolunar.sunset;
  doc["solunar"]["solarnoon"] = dailySolunar.solarnoon;

  doc["servos"]["pan"] = roundf(currentPan * 10.0f) / 10.0f;
  doc["servos"]["tilt"] = roundf(currentTilt * 10.0f) / 10.0f;

  doc["ldr"]["tl"] = readLDR(LDR_TL);
  doc["ldr"]["tr"] = readLDR(LDR_TR);
  doc["ldr"]["bl"] = readLDR(LDR_BL);
  doc["ldr"]["br"] = readLDR(LDR_BR);

  String out;
  serializeJson(doc, out);
  return out;
}

// 处理 HTTP 请求 (HTML 内容未改变，保持原样)
void handleClient(WiFiClient client) {
  String req = client.readStringUntil('\r');
  if (req.length() < 2) { client.stop(); return; }
  int sp1 = req.indexOf(' '), sp2 = req.indexOf(' ', sp1 + 1);
  String path = req.substring(sp1 + 1, sp2);
  while (client.available()) { String line = client.readStringUntil('\n'); if (line == "\r") break; }

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
    border-radius: 4px; 
}
.bg-white { border-radius: 0.75rem; } 
</style>
</head>
<body class="bg-gray-50 font-sans p-4 sm:p-8">
  <div class="max-w-4xl mx-auto space-y-8">
    <h2 class="text-4xl font-extrabold text-indigo-700 mb-8 text-center border-b-4 border-indigo-200 pb-3">
        ☀️ Arduino R4 Solar Tracker 卫星控制台 🛰️
    </h2>

    <div class="bg-white p-6 rounded-xl shadow-2xl border-t-4 border-indigo-500">
        <div class="flex flex-col sm:flex-row justify-between items-center mb-4">
            <h3 class="text-xl font-bold text-gray-700">系统状态</h3>
            <p id="system_mode" class="text-sm font-semibold px-3 py-1 rounded-full"></p>
        </div>
        <p class="text-center text-gray-600 text-sm">当前 UTC 时间: <span id="current_utc" class="font-mono text-base text-black">Loading...</span></p>
    </div>

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
            // 只有当新的同步时间明显大于本地时钟时才更新基准，避免网络延迟导致时间倒退
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
setInterval(updateStatus, 200); 
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

  // 1. 初始化引脚和舵机
  pinMode(LDR_TL, INPUT); pinMode(LDR_TR, INPUT);
  pinMode(LDR_BL, INPUT); pinMode(LDR_BR, INPUT);
  panServo.attach(PAN_SERVO_PIN); tiltServo.attach(TILT_SERVO_PIN);
  
  // 首次写入，使用 roundf 确保是整数
  panServo.write((int)roundf(currentPan)); 
  tiltServo.write((int)roundf(currentTilt));

  // 2. Wi-Fi 连接
  Serial.print("正在连接 Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) { delay(200); Serial.print("."); }

  if (WiFi.status() == WL_CONNECTED) {
    // 等待 IP 地址分配完成
    int attempts = 0;
    while(WiFi.localIP() == IPAddress(0, 0, 0, 0) && attempts++ < 20 && millis() - start < 20000) {
      delay(500);
      Serial.print("#");
    }
    
    Serial.println("\nWi-Fi 连接成功");
    Serial.print("IP 地址: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWi-Fi 连接失败");
  }

  server.begin();
  lastSolarCalc = 0;
  lastServoUpdate = 0;
  dailySolunar.lastFetchDay = 0;
}

void loop() {
  // 1. HTTP 客户端处理 (非阻塞)
  WiFiClient client = server.available();
  if (client) {
    handleClient(client);
  }

  unsigned long now = millis();

  //===============================
  static unsigned long lastLDRDebug = 0;
  if (millis() - lastLDRDebug > 500) {
      lastLDRDebug = millis();
      float avgLDR = (float)(readLDR(LDR_TL) + readLDR(LDR_TR) + readLDR(LDR_BL) + readLDR(LDR_BR)) / 4.0f;
      Serial.print("LDR Avg: ");
      Serial.println(avgLDR);
  }
  //================================
  
  // 2. 太阳位置/时间同步任务
  if (now - lastSolarCalc >= SOLAR_CALC_INTERVAL) {
    lastSolarCalc = now; // 无论成功与否，都重置定时器
    
    // 尝试连接或重连 Wi-Fi
    if (!attemptReconnect()) { 
      isNightMode = true; 
      return; // Wi-Fi 失败，跳过计算
    }

    TimeData newTime;
    // 必须先获取时间
    if (fetchUTC(newTime)) {
      g_currentTime = newTime;
      // 计算太阳位置
      lastSolar = solarPositionUTC(g_currentTime, LATITUDE, LONGITUDE);
      
      // 检查夜间模式
      bool wasNightMode = isNightMode;
      isNightMode = lastSolar.elevation < NIGHT_MODE_THRESHOLD;
      
      if (wasNightMode && !isNightMode) {
          Serial.println("退出夜间模式，开始追踪。");
      }
      
      // 获取日出日落时间，它会自动检查是否是新的一天
      fetchSolunar(g_currentTime);
      Serial.println("太阳位置和 Solunar 数据更新完成。");
    } else {
      // 如果时间同步失败，强制进入夜间模式
      Serial.println("时间同步失败。强制夜间模式以避免误差累积。");
      isNightMode = true;
    }
  }

  // 3. 舵机位置更新和追踪任务
  if (now - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {

    if (isNightMode) {
      // 仅在时间同步失败或夜晚深时将舵机移动到安全位置
      if (g_currentTime.Y == 0 || lastSolar.elevation < -15.0f) { 
        currentPan = PAN_MIN;
        currentTilt = NIGHT_TILT_ANGLE;
      }
    } else {
      // 计算太阳位置目标角度
      float targetPanFromSolar = azimuthToServo(lastSolar.azimuth);
      float targetTiltFromSolar = elevationToServo(lastSolar.elevation);
      
      // LDR 追踪逻辑
      int tl = readLDR(LDR_TL), tr = readLDR(LDR_TR);
      int bl = readLDR(LDR_BL), br = readLDR(LDR_BR);

      int azDiff = (tl + bl) - (tr + br);
      int elDiff = (tl + tr) - (bl + br);

      // 使用 fabsf
      float deltaPan = (fabsf((float)azDiff) > LDR_DEAD_BAND) ? (-(float)azDiff * AZ_GAIN) : 0.0f;
      float deltaTilt = (fabsf((float)elDiff) > LDR_DEAD_BAND) ? ((float)elDiff * EL_GAIN) : 0.0f;

      // 混合 LDR 误差修正和太阳位置校正 (P+I 控制)
      currentPan += deltaPan + SOLAR_CORRECTION_RATE * (targetPanFromSolar - currentPan);
      currentTilt += deltaTilt + SOLAR_CORRECTION_RATE * (targetTiltFromSolar - currentTilt);

      // 限制在有效角度范围内
      currentPan = constrain(currentPan, PAN_MIN, PAN_MAX);
      currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);
    }

    // 写入舵机 (使用 roundf 进行四舍五入)
    panServo.write((int)roundf(currentPan));
    tiltServo.write((int)roundf(currentTilt));

    lastServoUpdate = now;
  }
}