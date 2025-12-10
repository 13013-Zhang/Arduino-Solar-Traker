#include <WiFiS3.h>
#include <WiFiClient.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <math.h>

// --- 传感器引脚定义 (来自代码 1 & 2) ---
#define LDR_TL A0
#define LDR_TR A1
#define LDR_BL A2
#define LDR_BR A3
// 注意：代码1中的 solarPanel (A4) 在代码2中未使用，保留其定义以便于监控，但代码2的逻辑中暂不读取。
const int SOLAR_PANEL_PIN = A4;  // Solar panel voltage reading

// --- 舵机引脚定义 (来自代码 2) ---
#define PAN_SERVO_PIN 5
#define TILT_SERVO_PIN 6

// --- WiFi 配置 (来自代码 2) ---
const char* WIFI_SSID = "Redmi K70 Ultra";
const char* WIFI_PASS = "20050215";
// const char* WIFI_SSID = "Free_WiFi";
// const char* WIFI_PASS = "Bob@1357924680";

// --- 地理位置 (来自代码 2) ---
const float LATITUDE = 52.378753f;
const float LONGITUDE = -1.570225f;

// --- API 主机 (来自代码 2) ---
const char* SOLUNAR_HOST = "api.sunrise-sunset.org";
const char* TIME_HOST = "worldtimeapi.org";


// --- 动态步长 LDR 跟踪参数 (来自代码 1) ---
int tolerance = 25;
// 允许的误差范围 (用于 LDR 跟踪)
int minStepSize = 2;
// 最小步长

// --- 跟踪/控制参数 (来自代码 2) ---
// 由于采用了代码1的LDR逻辑，代码2中的 AZ_GAIN, EL_GAIN, LDR_DEAD_BAND, SOLAR_CORRECTION_RATE 将不再用于LDR调整。
const float NIGHT_MODE_THRESHOLD = -2.0f;
// 日出/日落判断
const float TILT_MAX = 180.0f;  // 调整为代码1的舵机范围
const float TILT_MIN = 0.0f;
const float PAN_MAX = 180.0f;
// 调整为代码1的舵机范围
const float PAN_MIN = 0.0f;
const float NIGHT_TILT_ANGLE = 5.0f;

// --- 时间间隔 (来自代码 2) ---
const unsigned long SOLAR_CALC_INTERVAL = 10000;  // 天文计算/时间同步间隔
const unsigned long SERVO_UPDATE_INTERVAL = 50;
// 舵机更新/LDR跟踪间隔

// --- 全局对象 ---
WiFiServer server(80);
Servo panServo, tiltServo;

// --- 位置变量 (来自代码 1/2) ---
// 统一使用浮点数，但 LDR 逻辑会使用 roundf 转换为整数控制舵机
float currentPan = 90.0f;  // 初始水平位置
float currentTilt = 90.0f;
// 初始垂直位置

// --- 结构体和全局状态 (来自代码 2) ---
struct SolarAngles {
  float azimuth;
  float elevation;
};
SolarAngles lastSolar = { 0.0f, 0.0f };

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

TimeData g_currentTime = { 0, 0, 0, 0, 0, 0, "" };

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



// --- 函数声明 ---
int readLDR(int pin);
float deg2rad(float d);
float rad2deg(float r);
bool fetchUTC(TimeData& t);
bool fetchSolunar(const TimeData& t);
SolarAngles solarPositionUTC(const TimeData& t, float lat, float lon);
String convert12hTo24h(String time12h);
double calcJulianDay(int Y, int M, int D, int h, int m, int s);
bool attemptReconnect(unsigned long timeout_ms = 15000);
String buildStatusJSON();
void handleClient(WiFiClient client);
int calculateDynamicStep(int error);  // 从代码1导入

// --- 函数实现 ---

/**
 * @brief 读取 LDR 传感器值
 * @param pin LDR 连接的模拟引脚
 * @return 模拟值 (0-1023)
 */
int readLDR(int pin) {
  analogRead(pin);  // 确保读两次以提高准确性
  return analogRead(pin);
}

/**
 * @brief 将太阳高度角转换为舵机倾斜角
 * @param el_deg 太阳高度角 (0-90度)
 * @return 舵机倾斜角 (0-180度)
 */
float elevationToServo(float el_deg) {
  // 简单的映射：太阳在地平线上时，舵机在 0-90 度范围内移动 (仰角)，
  // 假设 0 度仰角对应 TILT_MIN (0度舵机)，90 度仰角对应 TILT_MAX/2 (45度舵机)
  // 由于两个代码段的 TILT 舵机控制逻辑不同，我们采用代码1的简单 0-180 范围
  // 这里为了配合代码2的天文模式，我们仍使用天文角度，但 LDR 跟踪会直接控制 0-180
  // 为了兼容 LDR 跟踪，我们直接返回一个限制在 0-180 的值
  return constrain(el_deg, TILT_MIN, TILT_MAX);
}

/**
 * @brief 将太阳方位角转换为舵机平移角
 * @param az_deg 太阳方位角 (0-360度)
 * @return 舵机平移角 (0-180度)
 */
float azimuthToServo(float az_deg) {
  // 简单的映射：方位角 0-360 映射到 0-180
  return constrain(az_deg / 2.0f, PAN_MIN, PAN_MAX);
}

float deg2rad(float d) {
  return d * M_PI / 180.0f;
}
float rad2deg(float r) {
  return r * 180.0f / M_PI;
}

/**
 * @brief 从代码1导入：根据误差大小动态计算舵机步长
 * @param error LDR 传感器计算出的误差
 * @return 动态步长
 */
int calculateDynamicStep(int error) {
  int absError = abs(error);

  // 根据误差大小动态调整步长
  if (absError > 200) {
    return 20;  // 大误差，使用最大步长

  } else if (absError > 100) {
    return 15;
    // 中等误差

  } else if (absError > 50) {
    return 7;
    // 较小误差

  } else if (absError > 20) {
    return 3;
    // 小误差

  } else {
    return minStepSize;  // 最小步长
  }
}


// --- 代码2中的所有 API/时间/HTML/辅助函数 (为节省篇幅，在此省略完整实现，假定它们已按代码2中所示包含) ---
// [fetchUTC, fetchSolunar, solarPositionUTC, calcJulianDay, convert12hTo24h, attemptReconnect, buildStatusJSON, handleClient]

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
  double alpha_deg = rad2deg((float)alpha);
  if (alpha_deg < 0) alpha_deg += 360.0;
  double H = LST - alpha_deg;
  if (H < -180) H += 360;
  if (H > 180) H -= 360;
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

bool attemptReconnect(unsigned long timeout_ms) {
  if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
    return true;
  }

  Serial.println("\nAttempting Wi-Fi reconnect...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeout_ms) {
    delay(500);
    Serial.print("#");
  }

  if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
    Serial.println("\nReconnect successful. IP: " + WiFi.localIP().toString());
    server.begin();
    return true;
  }

  Serial.println("\nWi-Fi reconnect failed.");
  return false;
}

bool fetchUTC(TimeData& t) {
  WiFiClient client;
  Serial.print("Attempting to connect to Time API (");
  Serial.print(TIME_HOST);
  Serial.print(")...");

  if (!client.connect(TIME_HOST, 80)) {
    Serial.println("Connection failed!");
    return false;
  }
  Serial.println("Success!");

  String req = "GET /api/ip HTTP/1.1\r\nHost: ";
  req += TIME_HOST;
  req += "\r\nConnection: close\r\n\r\n";
  client.print(req);

  unsigned long start = millis();
  while (!client.available() && millis() - start < 10000) delay(10);

  if (!client.available()) {

    Serial.println("API response timeout.");
    client.stop();

    delay(100);
    return false;
  }

  while (client.available()) {
    String line = client.readStringUntil('\n');
    if (line.length() < 3 && line.indexOf('\r') != -1) break;
  }

  String json = client.readString();
  client.stop();
  delay(100);


  if (json.length() < 50) {
    Serial.println("API response too short or invalid.");
    return false;
  }

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, json);

  if (error) {

    Serial.print("UTC JSON parsing error: ");

    Serial.println(error.c_str());

    return false;
  }

  if (!doc.containsKey("utc_datetime")) {
    Serial.println("UTC JSON missing 'utc_datetime' field.");
    return false;
  }

  String dt = doc["utc_datetime"].as<String>();
  t.iso_sync = dt;


  t.Y = dt.substring(0, 4).toInt();
  t.M = dt.substring(5, 7).toInt();
  t.D = dt.substring(8, 10).toInt();
  t.h = dt.substring(11, 13).toInt();
  t.min = dt.substring(14, 16).toInt();
  t.s = dt.substring(17, 19).toInt();

  Serial.print("Successfully synced UTC time: ");
  Serial.println(t.toString());
  return true;
}

bool fetchSolunar(const TimeData& t) {
  if (t.Y == 0) {
    Serial.println("Solunar Fetch: UTC time not synced, skipping fetch.");
    return false;
  }
  if (t.D == dailySolunar.lastFetchDay && dailySolunar.sunrise != "--:--") {

    Serial.println("Solunar Fetch: Data for today already fetched, skipping.");
    return true;
  }

  WiFiClient client;
  Serial.print("Attempting to connect to Solunar API (");
  Serial.print(SOLUNAR_HOST);
  Serial.print(")...");

  if (!client.connect(SOLUNAR_HOST, 80)) {
    Serial.println("Connection failed!");
    return false;
  }
  Serial.println("Success!");

  char dateStr[11];
  sprintf(dateStr, "%04d-%02d-%02d", t.Y, t.M, t.D);

  String latStr = String(LATITUDE, 7);

  String lngStr = String(LONGITUDE, 7);


  String path = "/json?lat=" + latStr + "&lng=" + lngStr + "&date=" + String(dateStr);

  Serial.print("Solunar Request URL: http://");
  Serial.print(SOLUNAR_HOST);
  Serial.println(path);


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

      int jsonStart = line.indexOf('{');

      if (jsonStart != -1) {
        json += line.substring(jsonStart);

        while (client.available()) {
          json += client.readString();
        }
        break;
      }

    } else {
      delay(10);
    }
  }

  client.stop();
  delay(100);


  Serial.print("Solunar RAW JSON (CLEANED Length: ");
  Serial.print(json.length());
  Serial.println("):");
  Serial.println(json);

  if (json.length() < 50) {
    Serial.println("API response too short or invalid.");
    return false;
  }

  DynamicJsonDocument doc(2048);

  DeserializationError error = deserializeJson(doc, json);

  if (error) {

    Serial.print("Solunar JSON error: ");

    Serial.println(error.c_str());

    return false;
  }

  String status = doc["status"].as<String>();
  if (status != "OK" || !doc.containsKey("results")) {
    Serial.print("Solunar API returned non-OK status or missing results field: ");

    if (doc.containsKey("results")) {
      Serial.print("API Status: ");

      Serial.println(status);

    } else {
      Serial.println("API response format error or missing results field.");
    }
    return false;
  }

  String rawSunrise = doc["results"]["sunrise"].as<String>();
  String rawSunset = doc["results"]["sunset"].as<String>();
  String rawSolarNoon = doc["results"]["solar_noon"].as<String>();

  dailySolunar.sunrise = convert12hTo24h(rawSunrise);
  dailySolunar.sunset = convert12hTo24h(rawSunset);
  dailySolunar.solarnoon = convert12hTo24h(rawSolarNoon);

  dailySolunar.lastFetchDay = t.D;
  Serial.println("Solunar data updated successfully (from api.sunrise-sunset.org)");

  return true;
}

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

  doc["mode"] = isNightMode ? "Night Mode" : "Tracking Mode";

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

  // 添加太阳能板电压读取 (来自代码1的定义)
  int solarValue = analogRead(SOLAR_PANEL_PIN);
  float solarVoltage = solarValue * (5.0 / 1023.0);
  doc["solar_panel"]["voltage"] = roundf(solarVoltage * 100.0f) / 100.0f;
  doc["solar_panel"]["raw"] = solarValue;

  String out;
  serializeJson(doc, out);
  return out;
}

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
    // 整合后的 HTML：需要更新 JS 以显示新的太阳能板电压信息
    String html = R"rawliteral(
<!doctype html>
<html>
<head>
<meta charset='utf-8'>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Solar Tracker Console</title>
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
        ☀️ Arduino R4 Solar Tracker Console 🛰️
    </h2>

    <div class="bg-white p-6 rounded-xl shadow-2xl border-t-4 border-indigo-500">
        <div class="flex flex-col sm:flex-row justify-between items-center mb-4">
            <h3 class="text-xl font-bold text-gray-700">System Status</h3>
            <p id="system_mode" class="text-sm font-semibold px-3 py-1 rounded-full"></p>
        </div>
        <p class="text-center text-gray-600 text-sm">Current UTC Time: <span id="current_utc" class="font-mono text-base text-black">Loading...</span></p>
    </div>

    <div class="bg-white p-6 rounded-xl shadow-xl">
        <h3 class="text-xl font-semibold text-gray-700 mb-4 border-b pb-2">Daily Solar Events (Local Time - Sunrise-Sunset API)</h3>
        <div class="grid grid-cols-3 gap-4 text-center">
            <div class="p-4 bg-blue-50 rounded-lg shadow-inner">
                <p class="text-sm text-gray-500">Sunrise</p>
                <p id="sol_sunrise" class="text-2xl font-bold text-green-600">--:--</p>
            </div>
            <div class="p-4 bg-yellow-50 rounded-lg shadow-inner">
                <p class="text-sm text-gray-500">Solar Noon</p>
                <p id="sol_noon" class="text-2xl font-bold text-yellow-700">--:--</p>
            </div>
            <div class="p-4 bg-red-50 rounded-lg shadow-inner">
                <p class="text-sm text-gray-500">Sunset</p>
                <p id="sol_sunset" class="text-2xl font-bold text-red-600">--:--</p>
            </div>
        </div>
    </div>

    <div class="grid grid-cols-1 md:grid-cols-3 gap-6">
        <div class="bg-white p-6 rounded-xl shadow-xl border-l-4 border-indigo-400">
            <h3 class="text-xl font-semibold text-gray-700 mb-4 border-b pb-2">Calculated Position</h3>
            <p class="text-lg mb-2">Azimuth: <span id="az" class="font-mono text-2xl text-indigo-600">0</span>°</p>
            <p class="text-lg">Elevation: <span id="el" class="font-mono text-2xl text-indigo-600">0</span>°</p>
        </div>
        <div class="bg-white p-6 rounded-xl shadow-xl border-l-4 border-pink-400">
            <h3 class="text-xl font-semibold text-gray-700 mb-4 border-b pb-2">Servo Angles (Pan/Tilt)</h3>
            <p class="text-lg mb-2">Pan: <span id="pan" class="font-mono text-2xl text-pink-600">0</span>°</p>
            <p class="text-lg">Tilt: <span id="tilt" class="font-mono text-2xl text-pink-600">0</span>°</p>
        </div>
        <div class="bg-white p-6 rounded-xl shadow-xl border-l-4 border-green-400">
            <h3 class="text-xl font-semibold text-gray-700 mb-4 border-b pb-2">Solar Panel Output</h3>
            <p class="text-lg mb-2">Voltage: <span id="voltage" class="font-mono text-2xl text-green-600">0.00</span> V</p>
            <p class="text-sm text-gray-500">Raw ADC: <span id="raw_adc" class="font-mono text-base">0</span></p>
        </div>
    </div>

    <div class="bg-white p-6 rounded-xl shadow-xl">
        <h3 class="text-xl font-semibold text-gray-700 mb-4 border-b pb-2">LDR Light Sensor Readings (0-1023)</h3>
        <div class="overflow-x-auto">
            <table class="min-w-full divide-y divide-gray-200">
                <thead>
                    <tr class="bg-gray-50">
                        <th class="px-3 py-3 text-left text-sm font-medium text-gray-500 uppercase tracking-wider">Sensor</th>
                        <th class="px-3 py-3 text-left text-sm font-medium text-gray-500 uppercase tracking-wider">Reading</th>
                        <th class="px-3 py-3 text-left text-sm font-medium text-gray-500 uppercase tracking-wider">Light Level</th>
                    </tr>
                </thead>
                <tbody class="bg-white divide-y divide-gray-200 text-left">
                    <tr class="hover:bg-gray-50"><td>Top-Left</td><td id="tl" class="font-mono">0</td><td><div class="ldr-bar-container"><div id="tlbar" class="ldr-bar bg-yellow-400"></div></div></td></tr>
                    <tr class="hover:bg-gray-50"><td>Top-Right</td><td id="tr" class="font-mono">0</td><td><div class="ldr-bar-container"><div id="trbar" class="ldr-bar bg-yellow-400"></div></div></td></tr>
                    <tr class="hover:bg-gray-50"><td>Bottom-Left</td><td id="bl" class="font-mono">0</td><td><div class="ldr-bar-container"><div id="blbar" class="ldr-bar bg-yellow-400"></div></div></td></tr>
                    <tr class="hover:bg-gray-50"><td>Bottom-Right</td><td id="br" class="font-mono">0</td><td><div class="ldr-bar-container"><div id="brbar" class="ldr-bar bg-yellow-400"></div></div></td></tr>
                </tbody>
            </table>
        </div>
    </div>

  </div>
<script>
const LDR_IDS = ['tl', 'tr', 'bl', 'br'];
let lastSyncDate = null; 

function formatUtcTime(date) {
    const Y = date.getUTCFullYear();
    const M = String(date.getUTCMonth() + 1).padStart(2, '0');
    const D = String(date.getUTCDate()).padStart(2, '0');
    const h = String(date.getUTCHours()).padStart(2, '0');
    const m = String(date.getUTCMinutes()).padStart(2, '0');
    const s = String(date.getUTCSeconds()).padStart(2, '0');
    return `${Y}-${M}-${D} ${h}:${m}:${s}`;
}

function updateClientClock() {
    const utcElement = document.getElementById('current_utc');
    if (lastSyncDate) {
        lastSyncDate.setSeconds(lastSyncDate.getSeconds() + 1);
        
        utcElement.innerText = formatUtcTime(lastSyncDate);
        utcElement.classList.remove('text-red-600', 'font-bold');
        utcElement.classList.add('text-black');
    } else {
        utcElement.innerText = "Time Sync Failed (Waiting for Arduino Sync...)";
        utcElement.classList.add('text-red-600', 'font-bold');
        utcElement.classList.remove('text-black');
    }
}

async function updateStatus(){
    try {
        let resp = await fetch('/status');
        if (!resp.ok) throw new Error('Network response was not ok');
        let data = await resp.json();

        if (data.timestamp_sync) {
            const newSyncTime = new Date(data.timestamp_sync);
            if (lastSyncDate === null || newSyncTime.getTime() > lastSyncDate.getTime() + 10000) {
                 lastSyncDate = newSyncTime;
            } else if (Math.abs(newSyncTime.getTime() - lastSyncDate.getTime()) > 5000) {
                 lastSyncDate = newSyncTime;
            }

        } else {
            lastSyncDate = null;
        }

        const modeElement = document.getElementById('system_mode');
        modeElement.innerText = data.mode;
        modeElement.className = data.mode.includes('Night')
            ? 'text-sm font-semibold px-3 py-1 rounded-full bg-gray-600 text-white'
            : 'text-sm font-semibold px-3 py-1 rounded-full bg-green-500 text-white';

        document.getElementById('sol_sunrise').innerText = data.solunar.sunrise || '--:--';
        document.getElementById('sol_noon').innerText = data.solunar.solarnoon || '--:--';
        document.getElementById('sol_sunset').innerText = data.solunar.sunset || '--:--';
        
        document.getElementById('az').innerText = data.solar.azimuth.toFixed(1);
        document.getElementById('el').innerText = data.solar.elevation.toFixed(1);

        document.getElementById('pan').innerText = data.servos.pan.toFixed(1);
        document.getElementById('tilt').innerText = data.servos.tilt.toFixed(1);

        document.getElementById('voltage').innerText = data.solar_panel.voltage.toFixed(2);
        document.getElementById('raw_adc').innerText = data.solar_panel.raw;


        LDR_IDS.forEach(id => {
            let val = data.ldr[id];
            document.getElementById(id).innerText = val;
            let widthPercent = Math.min((val / 1023) * 100, 100) + '%';
            document.getElementById(id + 'bar').style.width = widthPercent;
        });

    } catch (error) {
        console.error("Failed to fetch status:", error);
    }
}

setInterval(updateClientClock, 1000); 
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


// --- Setup 函数 ---
void setup() {
  Serial.begin(115200);
  delay(500);

  // LDR 和太阳能板引脚设置
  pinMode(LDR_TL, INPUT);
  pinMode(LDR_TR, INPUT);
  pinMode(LDR_BL, INPUT);
  pinMode(LDR_BR, INPUT);
  pinMode(SOLAR_PANEL_PIN, INPUT);

  // 舵机连接
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);

  // 初始位置
  panServo.write((int)roundf(currentPan));

  tiltServo.write((int)roundf(currentTilt));

  Serial.print("Connecting to Wi-Fi...");
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

    Serial.println("\nWi-Fi connection successful");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

  } else {
    Serial.println("\nWi-Fi connection failed");
  }

  server.begin();
  lastSolarCalc = 0;
  lastServoUpdate = 0;
  dailySolunar.lastFetchDay = 0;
}


// --- Loop 函数 (核心跟踪逻辑融合在此) ---
void loop() {
  WiFiClient client = server.available();
  if (client) {
    handleClient(client);
  }

  unsigned long now = millis();

  // --- 1. 天文和时间同步 (来自代码 2) ---
  if (now - lastSolarCalc >= SOLAR_CALC_INTERVAL) {
    lastSolarCalc = now;

    if (!attemptReconnect()) {

      isNightMode = true;

      return;
    }

    TimeData newTime;
    if (fetchUTC(newTime)) {
      g_currentTime = newTime;
      lastSolar = solarPositionUTC(g_currentTime, LATITUDE, LONGITUDE);

      bool wasNightMode = isNightMode;
      isNightMode = lastSolar.elevation < NIGHT_MODE_THRESHOLD;

      if (wasNightMode && !isNightMode) {
        Serial.println("Exiting night mode, starting tracking.");
      }

      fetchSolunar(g_currentTime);
      Serial.println("Solar position and Solunar data update complete.");

    } else {
      Serial.println("Time sync failed. Forcing night mode to prevent error accumulation.");
      isNightMode = true;
    }
  }

  // --- 2. 舵机更新/LDR 跟踪 (来自代码 1 的逻辑，基于代码 2 的计时) ---
  if (now - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {
    lastServoUpdate = now;

    if (isNightMode) {
      // 夜间模式: 归位到初始位置
      if (g_currentTime.Y == 0 || lastSolar.elevation < -15.0f) {

        currentPan = PAN_MIN;
        currentTilt = NIGHT_TILT_ANGLE;
      }

    } else {
      // 白天跟踪模式: 使用代码 1 的动态步长 LDR 误差调整

      // 1. 读取 LDR 传感器值
      int tl = readLDR(LDR_TL);
      int tr = readLDR(LDR_TR);
      int bl = readLDR(LDR_BL);
      int br = readLDR(LDR_BR);

      // 2. 计算平均值
      int avgTop = (tl + tr) / 2;
      int avgBottom = (bl + br) / 2;
      int avgLeft = (tl + bl) / 2;
      int avgRight = (tr + br) / 2;

      // 3. 计算误差 (Error)
      int errorVertical = avgTop - avgBottom;
      int errorHorizontal = avgLeft - avgRight;

      // 4. 动态计算步长（误差越大，步长越大）
      int verticalStep = calculateDynamicStep(errorVertical);
      int horizontalStep = calculateDynamicStep(errorHorizontal);

      // 5. 垂直方向调整
      if (abs(errorVertical) > tolerance) {
        // 根据误差方向调整垂直位置 (currentTilt)
        if (errorVertical > 0) {
          currentTilt += verticalStep;  // 顶部更亮，向上转动

        } else {
          currentTilt -= verticalStep;  // 底部更亮，向下转动
        }
      }

      // 6. 水平方向调整
      if (abs(errorHorizontal) > tolerance) {
        // 检查垂直位置是否翻转 (使用 currentTilt)
        bool isFlipped = (currentTilt > 90.0f);


        // 根据垂直翻转状态，决定水平方向的调整 (currentPan)
        if (isFlipped) {
          // 垂直翻转 (> 90度) 时，水平控制方向需要反转
          if (errorHorizontal > 0) {
            currentPan += horizontalStep;  // 左侧更亮，向右转动（反转后）

          } else {
            currentPan -= horizontalStep;  // 右侧更亮，向左转动（反转后）
          }

        } else {
          // 垂直未翻转 (<= 90度) 时，使用正常的逻辑
          if (errorHorizontal > 0) {
            currentPan -= horizontalStep;  // 左侧更亮，向左转动

          } else {
            currentPan += horizontalStep;  // 右侧更亮，向右转动
          }
        }
      }

      // 限制舵机角度在 0 到 180 度之间
      currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);
      currentPan = constrain(currentPan, PAN_MIN, PAN_MAX);
    }

    // 写入舵机位置
    panServo.write((int)roundf(currentPan));
    tiltServo.write((int)roundf(currentTilt));
  }
}