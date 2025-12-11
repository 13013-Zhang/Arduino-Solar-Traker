#include <WiFiS3.h> // For Uno R4 WiFi
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <ArduinoJson.h>
#include <Servo.h>

// ----------------------------------------------------------------------------
// 1. CONFIGURATION CONSTANTS
// ----------------------------------------------------------------------------

const char* WIFI_SSID = "Redmi K70 Ultra";
const char* WIFI_PASS = "20050215";


const float LATITUDE = 34.0522; 
const float LONGITUDE = -118.2437; 

// API Hostnames
const char* TIME_HOST = "worldtimeapi.org"; 
const char* SOLUNAR_HOST = "api.sunrise-sunset.org"; 


// LDRs
const int LDR_TL = A0; // Top-Left
const int LDR_TR = A1; // Top-Right
const int LDR_BL = A2; // Bottom-Left
const int LDR_BR = A3; // Bottom-Right
// Solar Panel Voltage Sensor
const int SOLAR_PANEL_PIN = A4; 
// Servos
const int HORIZONTAL_SERVO_PIN = 6; // Pan Servo
const int VERTICAL_SERVO_PIN = 5;   // Tilt Servo
// Manual Override Pin (Low to disable tracking)
const int SERVOS_OFF_PIN = 7; 


// ----------------------------------------------------------------------------
// 2. DATA STRUCTURES
// ----------------------------------------------------------------------------


struct TimeData {
    int Y = 0, M = 0, D = 0, h = 0, min = 0, s = 0;
    String iso_sync = ""; 

    String toString() const {
        char buf[20];
        sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d", Y, M, D, h, min, s);
        return String(buf);
    }
};

struct SolarAngles {
    float azimuth = 0.0;
    float elevation = 0.0;
};

struct SolunarData {
    String sunrise = "--:--";
    String sunset = "--:--";
    String solarnoon = "--:--";
    int lastFetchDay = 0;
};

// ----------------------------------------------------------------------------
// 3. GLOBAL STATE
// ----------------------------------------------------------------------------

// Web Server
WiFiServer server(80);


TimeData g_currentTime;
SolunarData dailySolunar;
SolarAngles lastSolar; 

Servo horizontalServo;
Servo verticalServo;


int sensitivity = 50; 
const int minStepSize = 2; 


int horizontalPos = 90;
int verticalPos = 90;

int tl = 0;
int tr = 0;
int bl = 0;
int br = 0;

// 太阳能板读数
float slr = 0;
float slrTrue = 0;

// 翻转逻辑 (Flip Timer)
int count = 0;
unsigned long stuckTimerStart = 0;
const int waitTime = 1000; // 1秒等待时间
const int cooldown = 1000; // 1秒冷却时间
unsigned long cooldownStarted = 0;

// 日落逻辑 (Sunset Timer)
int sunsetAngle = 20; // 仰角小于此值时开始判断
int LDRavg = 0;
int sunsetDarkness = 850; // 平均 LDR 读数大于此值时认为天黑
int countSunset = 0;
unsigned long sunsetTimer = 0;
const int sunsetWait = 5000; // 5秒等待时间
const int pauseTimer = 250; // 暂停时间
unsigned long currentTime = 0;


// ----------------------------------------------------------------------------
// 4. UTILITY FUNCTIONS (辅助函数)
// ----------------------------------------------------------------------------

float deg2rad(float deg) {
    return deg * (PI / 180.0);
}

float rad2deg(float rad) {
    return rad * (180.0 / PI);
}

// LDR 读取辅助函数
int readLDR(int pin) {
    return analogRead(pin);
}

// 儒略日计算 (简化版，适用于粗略太阳位置计算)
double calcJulianDay(int Y, int M, int D, int h, int min, int s) {
    double D_h = D + (h + min / 60.0 + s / 3600.0) / 24.0;
    if (M <= 2) {
        Y -= 1;
        M += 12;
    }
    double A = floor(Y / 100.0);
    double B = 2 - A + floor(A / 4.0);
    return floor(365.25 * (Y + 4716)) + floor(30.6001 * (M + 1)) + D_h + B - 1524.5;
}

// 12小时制时间转换为24小时制 HH:MM
String convert12hTo24h(String time12h) {
    if (time12h == "") return "--:--";
    time12h.trim();
    int spaceIndex = time12h.indexOf(' ');
    String timePart = time12h.substring(0, spaceIndex);
    String ampmPart = time12h.substring(spaceIndex + 1);

    int colonIndex = timePart.indexOf(':');
    int hour = timePart.substring(0, colonIndex).toInt();
    int minute = timePart.substring(colonIndex + 1, colonIndex + 3).toInt(); // Extract minutes up to the second colon if seconds exist

    if (ampmPart == "PM" && hour != 12) {
        hour += 12;
    } else if (ampmPart == "AM" && hour == 12) {
        hour = 0; // Midnight 12 AM is 00
    }

    char buf[6];
    sprintf(buf, "%02d:%02d", hour, minute);
    return String(buf);
}


// ----------------------------------------------------------------------------
// 5. SOLAR POSITION CALCULATION (太阳位置计算)
// ----------------------------------------------------------------------------

// 计算给定UTC时间的太阳方位角和仰角 (基于 NOAA 算法的简化版)
SolarAngles solarPositionUTC(const TimeData& t, float lat, float lon) {
    // 您的太阳角度计算逻辑
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
    // 修正的方位角计算 atan2(sin(H), cos(H) * sin(lat) - tan(delta) * cos(lat))
    double az_rad = atan2(sin(H_rad), cos(H_rad) * sin(lat_rad) - tan(delta) * cos(lat_rad));
    double az_deg = rad2deg((float)az_rad);
    // 将方位角调整到 0 到 360 度范围，0度为北，180度为南 (NOAA标准: 南为0)
    // 转换为标准方位角 (北为0, 东为90)
    az_deg = az_deg + 180.0; 
    if (az_deg < 0) az_deg += 360.0;
    if (az_deg >= 360) az_deg -= 360.0;
    
    SolarAngles out;
    out.azimuth = (float)az_deg;
    out.elevation = (float)elevation;
    return out;
}


// ----------------------------------------------------------------------------
// 6. NETWORK AND API FUNCTIONS (网络与 API)
// ----------------------------------------------------------------------------

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

    // Skip HTTP headers
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

    // 假设 DynamicJsonDocument size 足够
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
    // 格式: 2023-11-20T10:30:00.123456+00:00 (只取前19位)
    
    // 检查 T 在正确的位置
    if (dt.indexOf('T') != 10) {
        Serial.println("UTC datetime format error.");
        return false;
    }

    t.iso_sync = dt.substring(0, 19); 
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
    // API默认返回本地时间
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
    bool header_passed = false;

    while (client.connected() && millis() - startJsonRead < 8000) { // Timeout 8s
        if (client.available()) {
            String line = client.readStringUntil('\n');
            if (!header_passed) {
                if (line == "\r") { // End of headers
                    header_passed = true;
                }
            } else {
                json += line; // Start reading JSON body
            }
        } else {
            delay(10);
        }
    }
    client.stop();
    delay(100);
    
    // Attempt to find the first '{' to ensure clean JSON parsing
    int jsonStart = json.indexOf('{');
    if (jsonStart != -1) {
        json = json.substring(jsonStart);
    } else {
        Serial.println("API response missing JSON content.");
        return false;
    }


    Serial.print("Solunar RAW JSON (Length: ");
    Serial.print(json.length());
    Serial.println("):");
    // Serial.println(json); // 打印原始JSON可能太长，只打印长度

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
        Serial.print("Solunar API returned non-OK status: ");
        Serial.println(status);
        return false;
    }

    // API 返回 12 小时制带 AM/PM 的时间
    String rawSunrise = doc["results"]["sunrise"].as<String>();
    String rawSunset = doc["results"]["sunset"].as<String>();
    String rawSolarNoon = doc["results"]["solar_noon"].as<String>();

    // 转换为 24 小时制 HH:MM
    dailySolunar.sunrise = convert12hTo24h(rawSunrise);
    dailySolunar.sunset = convert12hTo24h(rawSunset);
    dailySolunar.solarnoon = convert12hTo24h(rawSolarNoon);
    dailySolunar.lastFetchDay = t.D;

    Serial.println("Solunar data updated successfully (from api.sunrise-sunset.org)");
    return true;
}


// ----------------------------------------------------------------------------
// 7. WEB SERVER FUNCTIONS (Web 服务器)
// ----------------------------------------------------------------------------

String buildStatusJSON() {
    DynamicJsonDocument doc(1024);
    doc["lat"] = LATITUDE;
    doc["lon"] = LONGITUDE;

    if (g_currentTime.Y != 0) {
        // 由于 Arduino 没有实时RTC，所以时间只是首次同步的值
        doc["utc"] = g_currentTime.toString(); 
        doc["timestamp_sync"] = g_currentTime.iso_sync; 
    } else {
        doc["utc"] = "Time Sync Failed";
    }

    // 强制设置为 LDR 跟踪模式
    doc["mode"] = "LDR Tracking Mode (Always Active)"; 

    // 天文数据 (初始计算值)
    doc["solar"]["azimuth"] = roundf(lastSolar.azimuth * 10.0f) / 10.0f;
    doc["solar"]["elevation"] = roundf(lastSolar.elevation * 10.0f) / 10.0f;

    // 日出日落数据
    doc["solunar"]["sunrise"] = dailySolunar.sunrise;
    doc["solunar"]["sunset"] = dailySolunar.sunset;
    doc["solunar"]["solarnoon"] = dailySolunar.solarnoon;

    // 伺服角度 (使用控制逻辑中的实时位置)
    doc["servos"]["pan"] = roundf(horizontalPos * 10.0f) / 10.0f;
    doc["servos"]["tilt"] = roundf(verticalPos * 10.0f) / 10.0f;

    // LDR 读数
    doc["ldr"]["tl"] = tl;
    doc["ldr"]["tr"] = tr;
    doc["ldr"]["bl"] = bl;
    doc["ldr"]["br"] = br;

    // 太阳能板输出
    doc["solar_panel"]["voltage"] = roundf(slrTrue * 100.0f) / 100.0f;
    doc["solar_panel"]["raw"] = (int)slr;

    String out;
    serializeJson(doc, out);
    return out;
}

void handleClient(WiFiClient client) {
    // Web Server HTML/JSON 逻辑
    String req = client.readStringUntil('\r');
    if (req.length() < 2) {
        client.stop();
        return;
    }
    int sp1 = req.indexOf(' '), sp2 = req.indexOf(' ', sp1 + 1);
    String path = req.substring(sp1 + 1, sp2);
    
    // Read the rest of the HTTP request headers
    while (client.available()) {
        String line = client.readStringUntil('\n');
        if (line == "\r") break;
    }

    if (path == "/") {
        // HTML Content - 仅保留伺服角度、太阳能板输出和 LDR 读数
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

        <!-- 调整为 2 列布局，仅保留角度和输出 -->
        <div class="grid grid-cols-1 md:grid-cols-2 gap-6">
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

async function updateStatus(){
    try {
        let resp = await fetch('/status');
        if (!resp.ok) throw new Error('Network response was not ok');
        let data = await resp.json();

        // 仅更新保留的字段：伺服角度和太阳能板输出
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

// 首次调用 updateStatus 来获取数据
updateStatus();
// 设置定期更新，频率为 200ms
setInterval(updateStatus, 200); 
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


// ----------------------------------------------------------------------------
// 8. CONTROL LOGIC FUNCTIONS (控制逻辑)
// ----------------------------------------------------------------------------

int calculateDynamicStep(int error) {
    // 您的动态步长计算逻辑
    int absError = abs(error);

    if (absError > 200) {
        return 15; 
    } else if (absError > 100) {
        return 8;      
    } else if (absError > 50) {
        return 4;   
    } else if (absError > 20) {
        return 2;
    } else {
        return minStepSize;
    }
}


// ----------------------------------------------------------------------------
// 9. ARDUINO SETUP AND LOOP (核心代码)
// ----------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    // 等待串行端口连接，仅在调试时需要
    while (!Serial); 
    Serial.println("Solar Tracker Initializing...");

    // 初始化伺服电机
    horizontalServo.attach(HORIZONTAL_SERVO_PIN);
    horizontalServo.write(horizontalPos);

    verticalServo.attach(VERTICAL_SERVO_PIN);
    verticalServo.write(verticalPos);

    // 初始化控制引脚
    pinMode(SERVOS_OFF_PIN, INPUT_PULLUP);

    // 连接 WiFi
    Serial.print("Connecting to Wi-Fi...");
    if (!attemptReconnect(10000)) { // 10秒超时
        Serial.println("Fatal: Wi-Fi connection failed at startup.");
        // 如果连接失败，可以考虑进入低功耗模式或停止后续API调用
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    }

    // 获取 UTC 时间
    if (g_currentTime.Y == 0) {
        fetchUTC(g_currentTime);
    }

    // 获取日出日落时间
    if (g_currentTime.Y != 0) {
        fetchSolunar(g_currentTime);
        // 计算初始太阳位置 (用于网页显示)
        lastSolar = solarPositionUTC(g_currentTime, LATITUDE, LONGITUDE);
        Serial.print("Initial Solar Pos: Az=");
        Serial.print(lastSolar.azimuth);
        Serial.print(", El=");
        Serial.println(lastSolar.elevation);
    }
    
    server.begin();
    Serial.println("HTTP Server started.");
    delay(100);
}

void loop() {
    // 1. Web Server Client Handling
    WiFiClient client = server.available();
    if (client) {
        handleClient(client);
    }

    // 2. Control System Logic (LDRs and Servos)
    
    // LDR reading
    tl = analogRead(LDR_TL);
    tr = analogRead(LDR_TR);
    bl = analogRead(LDR_BL);
    br = analogRead(LDR_BR);

    // Solar reading
    slr = analogRead(SOLAR_PANEL_PIN);
    slrTrue = slr * (5.0 / 1023.0); // 转换为电压 (假设 5V 参考电压)

    // 检查手动关闭开关 (SERVOS_OFF_PIN 低电平有效)
    if (digitalRead(SERVOS_OFF_PIN) == LOW) {
        // 保持在中心位置且不移动
        verticalPos = 90;
        horizontalPos = 90;
        verticalServo.write(verticalPos);
        horizontalServo.write(horizontalPos);
        Serial.println("--- Servos Disabled (Pin 7 LOW) ---");
        delay(500); // 降低循环频率
        return; // 跳过控制和日志输出
    }


    // Horizontal calculations (Pan)
    int errorHorizontal = (tl + bl) / 2 - (tr + br) / 2;
    int horizontalStep = calculateDynamicStep(errorHorizontal);

    // 从 Servo 读取当前位置 (虽然我们用变量跟踪，但这里确保同步)
    horizontalPos = horizontalServo.read();

    // 调整水平步长
    if (abs(errorHorizontal) < sensitivity) horizontalStep = 0;
    if (errorHorizontal < 0) horizontalStep = -horizontalStep; // 负误差表示向右 (TR/BR 大)

    // 垂直翻转校正 (如果垂直舵机超过 90 度，意味着面板倒置，水平方向需反转)
    if (verticalPos > 90) horizontalStep = -horizontalStep;


    // Vertical Servo control (Tilt)
    int errorVertical = (tl + tr) / 2 - (bl + br) / 2;
    int verticalStep = calculateDynamicStep(errorVertical);

    verticalPos = verticalServo.read();

    // 调整垂直步长
    if (abs(errorVertical) < sensitivity) verticalStep = 0;
    // 垂直舵机通常是向上 (大读数在 Top LDRs) 移动，这里是反向控制 (垂直轴与水平轴方向相反)
    if (errorVertical < 0) verticalStep = -verticalStep; // 负误差表示向下 (BL/BR 大)


    // 3. Flip Code (180度翻转逻辑)
    // 检查水平移动是否超出限制 (0-180度)
    if ((horizontalPos + horizontalStep > 180) || (horizontalPos + horizontalStep < 0)) {
        if (count == 0) {
            // 检查冷却时间
            if (millis() < cooldownStarted + cooldown) {
                Serial.println("On cooldown!");
            } else {
                // 启动翻转计时器
                stuckTimerStart = millis();
                count = 1;
                Serial.println("-------------------");
                Serial.println("Flip Timer Started!");
                Serial.println("-------------------");
            }
        } else {
            // 计时器已启动，检查是否超时
            if (millis() >= stuckTimerStart + waitTime) {
                // 执行翻转
                horizontalPos = 180 - horizontalPos; // 水平反转
                horizontalStep = 0;
                verticalPos = 180 - verticalPos;     // 垂直反转
                verticalStep = 0;

                Serial.println("--------------------");
                Serial.println("Flipping the script!");
                Serial.println("--------------------");


                currentTime = millis();
                while (millis() < currentTime + pauseTimer) {} 
                cooldownStarted = millis(); 
                count = 0;  
                stuckTimerStart = 0;
            }
        }
    } else {
        // 如果之前启动了计时器，但现在不再超出限制，则重置计时器
        if (count == 1) {
            count = 0;
            stuckTimerStart = 0;
        }
    }


    // 4. Sunset Code (日落逻辑)
    LDRavg = (tl + tr + bl + br) / 4;

    // 条件: 仰角够低 AND LDR 平均值够高 (天黑)
    if (((verticalServo.read() <= sunsetAngle) || (verticalServo.read() >= 180 - sunsetAngle)) && (LDRavg >= sunsetDarkness)) {
        if (countSunset == 0) {
            // 启动日落计时器
            sunsetTimer = millis();
            countSunset = 1;
            Serial.println("----------------");
            Serial.println("Possible Sunset!");
            Serial.println("----------------");
        } else {
            if (millis() >= sunsetTimer + sunsetWait) {
                // 日落确认，移动到夜间位置 (45度或 135度)
                countSunset = 0;
                sunsetTimer = 0;

                Serial.println("----------------------------");
                Serial.println("Sunset Confirmed! Goodnight!");
                Serial.println("----------------------------");
                
                // 短暂暂停以便串口输出
                currentTime = millis();
                while (millis() < currentTime + pauseTimer) {} 

                if (verticalServo.read() <= sunsetAngle) {
                    verticalPos = 135; // 移动到一侧的 45 度角 (90 + 45)
                } else {
                    verticalPos = 45;  // 移动到另一侧的 45 度角 (90 - 45)
                }
                verticalStep = 0;
                horizontalStep = 0; // 停止水平移动
            }
        }
    } else {
        // 如果条件不再满足，重置日落计时器
        if (countSunset == 1) {
            Serial.println("-------------------------------");
            Serial.println("Sunset Unconfirmed! Staying up!");
            Serial.println("-------------------------------");

            countSunset = 0;
            sunsetTimer = 0;
        }
    }

    // 5. Servo Movement
    horizontalPos = constrain(horizontalPos + horizontalStep, 0, 180);
    horizontalServo.write(horizontalPos);

    verticalPos = constrain(verticalPos - verticalStep, 0, 180); 
    verticalServo.write(verticalPos);

    
    // 6. Serial Debug Output
    
    Serial.print("TL:");
    Serial.print(tl);
    Serial.print(" | TR:");
    Serial.print(tr);
    Serial.print(" | BL:");
    Serial.print(bl);
    Serial.print(" | BR:");
    Serial.print(br);

    Serial.print(" | H_Err:");
    Serial.print(errorHorizontal);
    Serial.print(" | H_Step:");
    Serial.print(horizontalStep);
    Serial.print(" | H_Pos:");
    Serial.print(horizontalPos);

    Serial.print(" | V_Err:");
    Serial.print(errorVertical);
    Serial.print(" | V_Step:");
    Serial.print(verticalStep);
    Serial.print(" | V_Pos:");
    Serial.print(verticalPos);

    Serial.print(" | V_Slr:");
    Serial.print(slrTrue);
    Serial.print("V");

    // 状态提示
    if (count == 1) {
        Serial.print(" | Flip Active!");
    }
    if (millis() < cooldownStarted + cooldown) {
        Serial.print(" | Flip Cooldown!");
    }
    if (countSunset == 1) {
        Serial.print(" | Checking Sunset!");
    }
    Serial.println();
    
    // 7. Obligatory delay
    delay(30);
}