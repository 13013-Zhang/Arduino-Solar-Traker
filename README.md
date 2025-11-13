# ES2C6-Solar-Tracker

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![HTML5](https://img.shields.io/badge/HTML5-E34F26?style=for-the-badge&logo=html5&logoColor=white)
![TailwindCSS](https://img.shields.io/badge/Tailwind_CSS-38B2AC?style=for-the-badge&logo=tailwind-css&logoColor=white)

This is a solar tracker repository for the ES2C6 course (or personal project). This project includes two distinct Arduino implementations to demonstrate and compare different solar tracking strategies.

1.  **Simple LDR + OLED Tracker**: A pure "active" tracker that uses four LDRs (Light Dependent Resistors) to find the brightest light source and displays real-time data on an OLED screen.
2.  **Advanced Hybrid + Web UI Tracker**: A "hybrid" tracker that uses an Arduino R4 WiFi to fetch precise UTC time, calculate the sun's astronomical position (azimuth/elevation), and then uses LDRs for fine-tuning. It also hosts a modern Web UI for status monitoring.

---

## 🚀 Project Structure


---

## 1. Simple LDR + OLED Tracker

This is a classic "active" solar tracker. It doesn't care about time or location; it only cares where the light is coming from.

### ✨ Features

* **Active Tracking**: Purely based on the difference in readings from the four LDRs to move the servos, always pointing towards the brightest direction.
* **OLED Display**: Uses an SSD1306 OLED screen to display real-time:
    * Solar panel voltage (if connected)
    * Raw readings of the 4 LDRs
    * Current servo angles (Horizontal/Vertical)
    * Average light intensity
* **Startup Scan**: Executes a `findMaxLightPosition()` function on boot, scanning the sky (0-180 degrees) to find the initial brightest spot.
* **Voltage Monitoring**: Optionally connect a small solar panel to pin `A4` to monitor its output voltage.

### 🛠️ Hardware Requirements

* Arduino Uno / Nano (or any compatible board)
* 2x Servo Motors (for Pan/Tilt)
* 4x LDRs (Light Dependent Resistors)
* 4x Pull-down Resistors (e.g., 10KΩ)
* SSD1306 OLED Display (SPI version used in the code)
* (Optional) 1x Small Solar Panel (for A4 pin reading)
* 2-axis Pan/Tilt bracket
* Breadboard and jumper wires

### 📚 Library Dependencies

* `Servo`
* `Adafruit_SSD1306`
* `Adafruit_GFX`

### 🔧 Setup and Usage

1.  Download the code into the `simple-ldr-oled-tracker/` folder.
2.  Open `simple-ldr-oled-tracker.ino` in the Arduino IDE.
3.  Install the `Adafruit_SSD1306` and `Adafruit_GFX` libraries using the Library Manager.
4.  **Check Pins**: Ensure your OLED and servo pin definitions match the `#define` section at the top of the code.
    * OLED (SPI): `OLED_MOSI`, `OLED_CLK`, `OLED_DC`, `OLED_CS`, `OLED_RESET`
    * Servos: `5` (horizontal), `6` (vertical)
    * LDRs: `A0` (TL), `A1` (TR), `A2` (BL), `A3` (BR)
    * Solar Panel: `A4`
5.  Upload the code to your Arduino.
6.  When the device boots, it will first perform a "find max light" scan. The OLED will then light up with real-time data, and the tracker will begin active tracking.

---

## 2. Advanced Hybrid + Web UI Tracker

This version uses an Arduino R4 WiFi board, combining time-based predictive tracking with LDR-based corrective tracking.

### ✨ Features

* **Hybrid Tracking**:
    * **Predictive**: Connects to `worldtimeapi.org` to get UTC time and accurately calculates the sun's Azimuth and Elevation based on a given Latitude and Longitude.
    * **Corrective**: Uses a 4-quadrant LDR sensor array to fine-tune the calculated position, compensating for any misalignments or atmospheric effects.
* **Web Console**: Hosts a web server that provides a modern, responsive dashboard built with Tailwind CSS.
* **Solunar API**: Fetches the day's sunrise, sunset, and solar noon times from `api.sunrise-sunset.org`.
* **Night Mode**: Automatically enters "Night Mode" when the sun's elevation is below a threshold (e.g., after sunset), moving the servos to a preset night position (e.g., facing east, ready for sunrise).
* **WiFi Management**: Automatically attempts to reconnect to WiFi if the connection is lost.

### 🛠️ Hardware Requirements

* Arduino R4 WiFi
* 2x Servo Motors (for Pan/Tilt)
* 4x LDRs (Light Dependent Resistors)
* 4x Pull-down Resistors (e.g., 10KΩ)
* 2-axis Pan/Tilt bracket
* Breadboard and jumper wires

### 📚 Library Dependencies

* `WiFiS3`
* `ArduinoJson`
* `Servo`

### 🔧 Setup and Usage

1.  Download the code into the `advanced-hybrid-tracker/` folder.
2.  Open `advanced-hybrid-tracker.ino` in the Arduino IDE.
3.  Install the required libraries using the Library Manager.
4.  **Crucial Configuration**: Modify the constants at the top of the sketch:
    * `WIFI_SSID`: Your WiFi network name.
    * `WIFI_PASS`: Your WiFi password.
    * `LATITUDE`: Your geographic latitude (e.g., `52.378753f`).
    * `LONGITUDE`: Your geographic longitude (e.g., `-1.570225f`).
5.  Upload the code to your Arduino R4 WiFi.
6.  Open the **Serial Monitor** (Baud Rate: 115200) to view the connection status and the IP address.
7.  On a device on the same network, open a web browser and navigate to that IP address (e.g., `http://192.168.1.100`) to see the web console.
8.  

### Demo
![Solar Tracker](<Demo/Solar Tracker With OLED Demo.gif>)
---

## 💡 How It Works

### Hybrid Tracker (Advanced)

This model uses a **Predict-Correct** loop:

1.  **Predict (Every 10 seconds)**: The device gets the precise time from an API and uses the `solarPositionUTC` function to calculate the sun's *theoretical* position in the sky (azimuth/elevation).
2.  **Correct (Every 50 milliseconds)**:
    * The LDRs read the light differential (`azDiff`, `elDiff`).
    * The new target position for the servo is a combination of **(LDR correction) + (a gentle pull towards the theoretical position)**.
    * `currentPan += deltaPan + SOLAR_CORRECTION_RATE * (targetPanFromSolar - currentPan);`
    * This ensures the tracker not only points accurately at the sun (LDR part) but also knows where the sun *should* be, even after it's temporarily hidden by clouds (time-based part).

### Active Tracker (Simple)

This model uses a simple **Feedback Loop**:

1.  Compare the average of the top LDRs vs. the average of the bottom LDRs (`diffVertical`).
2.  Compare the average of the left LDRs vs. the average of the right LDRs (`diffHorizontal`).
3.  If the vertical difference is greater than the `tolerance`, move the vertical servo up or down.
4.  If the horizontal difference is greater than the `tolerance`, move the horizontal servo left or right.
5.  This process repeats, constantly minimizing the LDR differences to stay pointed at the light source.

## 📄 License

This project is licensed under the [MIT License](LICENSE). (Recommended). You should add a `LICENSE` file to your repository.