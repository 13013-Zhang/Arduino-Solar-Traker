Read this in other languages: [English](README.md), [中文](README_ZH.md).

# ES2C6-太阳能追踪器

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![HTML5](https://img.shields.io/badge/HTML5-E34F26?style=for-the-badge&logo=html5&logoColor=white)
![TailwindCSS](https://img.shields.io/badge/Tailwind_CSS-38B2AC?style=for-the-badge&logo=tailwind-css&logoColor=white)

这是一个针对 **ES2C6 课程** 的太阳能追踪器项目仓库。该项目包含两种不同的 Arduino 实现，用于演示和比较不同的太阳能追踪策略。

1.  **简单光敏电阻 (LDR)**：一个纯粹的“**主动式**”追踪器，它使用四个光敏电阻（Light Dependent Resistors）来寻找最亮的光源。
2.  **高级混合式 + Web UI 追踪器**：一个“**混合式**”追踪器，它使用 **Arduino R4 WiFi** 获取精确的 **UTC 时间**，计算太阳的**天文位置**（方位角/高度角），然后利用光敏电阻进行微调。它还托管了一个现代化的 **Web UI** 用于状态监测。

### 电路
![Circuit](Demo/Circuit.jpg)

![Schematic](Demo/Schematic.svg)


---

## 1. 简单光敏电阻 LED 追踪器

这是一个经典的“**主动式**”太阳能追踪器。它不关心时间或位置；它只关心光线来自哪里。

### ✨ 特性

* **主动追踪**：纯粹基于四个光敏电阻的读数差异来移动舵机，始终指向最亮的方向。
* **串行数据输出**：通过串行监视器或可选的 OLED 屏幕显示实时数据，包括：
    * 太阳能电池板电压
    * 4 个光敏电阻的原始读数
    * 当前舵机角度（水平/垂直）
    * 平均光照强度
* **启动扫描**：启动时执行 `findMaxLightPosition()` 函数，扫描天空（0-180 度）以找到最初的最亮点。
* **电压监测**：可选地将小型太阳能电池板连接到 `A4` 引脚以监测其输出电压。

### 🛠️ 硬件要求

* Arduino Uno / Nano（或任何兼容的主板）
* 2x 舵机（用于水平/垂直转动）
* 4x 光敏电阻 (LDRs)
* 4x 下拉电阻（例如 10KΩ）
* 1x 小型太阳能电池板（用于 A4 引脚读数）
* 2 轴云台支架
* 面包板和跳线

### 📚 库依赖

* `Servo`
* `Adafruit_SSD1306`
* `Adafruit_GFX`

### 🔧 设置与使用

1.  将代码下载到 `Codes/sdynamic_Step/` 文件夹中。
2.  在 Arduino IDE 中打开 `sdynamic_Step.ino`。
3.  使用库管理器安装 `Adafruit_SSD1306` 和 `Adafruit_GFX` 库。
4.  **检查引脚**：确保您的 OLED 和舵机引脚定义与代码顶部的 `#define` 部分匹配。
    * OLED (SPI)：`OLED_MOSI`、`OLED_CLK`、`OLED_DC`、`OLED_CS`、`OLED_RESET`
    * 舵机：`5`（水平）、`6`（垂直）
    * 光敏电阻：`A0`（左上）、`A1`（右上）、`A2`（左下）、`A3`（右下）
    * 太阳能电池板：`A4`
5.  将代码上传到您的 Arduino。
6.  设备启动时，它将首先执行“寻找最大光照”扫描。然后 OLED 将点亮显示实时数据，追踪器将开始主动追踪。

### 演示
- 未组装
![Solar Tracker](<Demo/Solar Tracker With OLED Demo.gif>)


---

## 2. 高级混合式 + Web UI 追踪器

此版本使用 **Arduino R4 WiFi** 主板，将基于时间的预测追踪与基于光敏电阻的校正追踪相结合。

### ✨ 特性

* **混合追踪**：
    * **预测式**：连接到 `worldtimeapi.org` 获取 UTC 时间，并根据给定的经度和纬度准确计算太阳的**方位角**和**高度角**。
    * **校正式**：使用四象限光敏电阻传感器阵列对计算出的位置进行微调，以补偿任何未对准或大气效应。
* **Web 控制台**：托管一个 Web 服务器，提供一个使用 **Tailwind CSS** 构建的现代化、响应式仪表板。
* **日照 API**：从 `api.sunrise-sunset.org` 获取当天的日出、日落和太阳正午时间。
* **夜间模式**：当太阳的高度角低于阈值（例如日落后）时，自动进入“夜间模式”，将舵机移动到预设的夜间位置（例如面向东方，为日出做准备）。
* **WiFi 管理**：如果连接丢失，自动尝试重新连接到 WiFi。

### 🛠️ 硬件要求

* Arduino R4 WiFi
* 2x 舵机（用于水平/垂直转动）
* 4x 光敏电阻 (LDRs)
* 4x 下拉电阻（例如 10KΩ）
* 2 轴云台支架
* 面包板和跳线

### 📚 库依赖

* `WiFiS3`
* `ArduinoJson`
* `Servo`

### 🔧 设置与使用

1.  将代码下载到 `Codes/Dynamic_Step_wtih_Website/` 文件夹中。
2.  在 Arduino IDE 中打开 `dynamic_Step_wtih_Website.ino`。
3.  使用库管理器安装所需的库。
4.  **关键配置**：修改草图顶部的常量：
    * `WIFI_SSID`：您的 WiFi 网络名称。
    * `WIFI_PASS`：您的 WiFi 密码。
    * `LATITUDE`：您的地理纬度（例如 `52.378753f`）。
    * `LONGITUDE`：您的地理经度（例如 `-1.570225f`）。
5.  将代码上传到您的 Arduino R4 WiFi。
6.  打开**串行监视器**（波特率：115200）以查看连接状态和 IP 地址。
7.  在同一网络上的设备上，打开 Web 浏览器并导航到该 IP 地址（例如 `http://192.168.31.166`）以查看 Web 控制台。

### 演示
![Solar Tracker](<Demo/Arduino R4 Solar Tracker console.jpg>)


---

## 💡 工作原理

### 主动式追踪器（简单）

该模型使用一个简单的**反馈回路**：

1.  比较顶部光敏电阻的平均值与底部光敏电阻的平均值 (`diffVertical`)。
2.  比较左侧光敏电阻的平均值与右侧光敏电阻的平均值 (`diffHorizontal`)。
3.  如果垂直差异大于 `tolerance`，则向上或向下移动垂直舵机。
4.  如果水平差异大于 `tolerance`，则向左或向右移动水平舵机。
5.  这个过程不断重复，持续最小化光敏电阻差异，以保持指向光源。

### 混合式追踪器（高级）

该模型使用一个**预测-校正**循环：

1.  **预测（每 10 秒）**：设备从 API 获取精确时间，并使用 `solarPositionUTC` 函数计算太阳在天空中的*理论*位置（方位角/高度角）。
2.  **校正（每 50 毫秒）**：
    * 光敏电阻读取光差 (`azDiff`、`elDiff`)。
    * 舵机的新目标位置是 **(光敏电阻校正) + (朝向理论位置的温和拉动)** 的组合。
    * `currentPan += deltaPan + SOLAR_CORRECTION_RATE * (targetPanFromSolar - currentPan);`
    * 这确保了追踪器不仅能准确指向太阳（光敏电阻部分），而且即使太阳被云层暂时遮挡，它也知道太阳*应该*在哪里（基于时间的部分）。

---


## 🎉 致谢 / 团队成员

本项目是 ES2C6 模块的一项协作努力，其成功是整个团队共同努力的结果。我们对以下成员表示衷心的感谢：

| 姓名 | 负责角色 |
| :--- | :--- |
| Monteiro, Marcus | 背景研究 |
| Zhao, Haoxiang Owen | 机械设计和 CAD 工作 |
| Jackson, Josh | 电子设计 |
| Stephan, Freddy | 机械部件和电子电路板的制造 |
| Maini, Rohan | 组装 |
| Boyu Zhang Bob (me) | 编程与测试 |
| Charusirisawad, Pun | 总体项目管理和任务负责人 |

---

## 📄 许可证

本项目根据 [MIT 许可证](LICENSE) 授权。您应该在您的仓库中添加一个 `LICENSE` 文件。