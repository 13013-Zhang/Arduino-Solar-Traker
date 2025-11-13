#include <Servo.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define OLED_MOSI   11
#define OLED_CLK    13
#define OLED_DC     9
#define OLED_CS     10
#define OLED_RESET  8

Adafruit_SSD1306 display(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS);

Servo horizontalServo;
Servo verticalServo;

const int ldrTL = A0;
const int ldrTR = A1;
const int ldrBL = A2;
const int ldrBR = A3;
const int solarPanel = A4;

int horizontalPos = 90;
int verticalPos = 90;
int tolerance = 20;
unsigned long lastDisplayUpdate = 0;

void setup() {
  Serial.begin(9600);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  display.display();
  delay(2000);
  display.clearDisplay();
  
  horizontalServo.attach(5);
  verticalServo.attach(6);
  
  findMaxLightPosition();
  updateDisplay();
}

void loop() {
  int tl = analogRead(ldrTL);
  int tr = analogRead(ldrTR);
  int bl = analogRead(ldrBL);
  int br = analogRead(ldrBR);
  
  int avgTop = (tl + tr) / 2;
  int avgBottom = (bl + br) / 2;
  int avgLeft = (tl + bl) / 2;
  int avgRight = (tr + br) / 2;
  
  int diffVertical = avgTop - avgBottom;
  int diffHorizontal = avgLeft - avgRight;
  
  if (abs(diffVertical) > tolerance) {
    if (diffVertical > 0) {
      verticalPos = min(verticalPos + 2, 180);
    } else {
      verticalPos = max(verticalPos - 2, 0);
    }
    verticalServo.write(verticalPos);
  }
  
  if (abs(diffHorizontal) > tolerance) {
    if (diffHorizontal > 0) {
      horizontalPos = min(horizontalPos + 2, 180);
    } else {
      horizontalPos = max(horizontalPos - 2, 0);
    }
    horizontalServo.write(horizontalPos);
  }
  
  int solarValue = analogRead(solarPanel);
  float solarVoltage = solarValue * (5.0 / 1023.0);
  
  Serial.print("Solar Voltage: ");
  Serial.print(solarVoltage);
  Serial.print("V | LDR: ");
  Serial.print(tl);
  Serial.print(",");
  Serial.print(tr);
  Serial.print(",");
  Serial.print(bl);
  Serial.print(",");
  Serial.print(br);
  Serial.print(" | Servo H:");
  Serial.print(horizontalPos);
  Serial.print(" V:");
  Serial.println(verticalPos);
  
  if (millis() - lastDisplayUpdate > 100) {
    updateDisplayData(solarVoltage, tl, tr, bl, br, horizontalPos, verticalPos);
    lastDisplayUpdate = millis();
  }
  
  delay(10);
}

void findMaxLightPosition() {
  int maxLight = 0;
  int bestH = 90;
  int bestV = 90;
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Finding max light...");
  display.display();
  
  for (int h = 0; h <= 180; h += 30) {
    for (int v = 0; v <= 180; v += 30) {
      horizontalServo.write(h);
      verticalServo.write(v);
      delay(400);
      
      int tl = analogRead(ldrTL);
      int tr = analogRead(ldrTR);
      int bl = analogRead(ldrBL);
      int br = analogRead(ldrBR);
      
      int totalLight = tl + tr + bl + br;
      
      if (totalLight > maxLight) {
        maxLight = totalLight;
        bestH = h;
        bestV = v;
      }
    }
  }
  
  horizontalServo.write(bestH);
  verticalServo.write(bestV);
  horizontalPos = bestH;
  verticalPos = bestV;
  
  Serial.print("Initial position set to H:");
  Serial.print(bestH);
  Serial.print(" V:");
  Serial.println(bestV);
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Solar Tracker");
  display.println("------------");
  
  display.display();
}

void updateDisplayData(float voltage, int tl, int tr, int bl, int br, int hPos, int vPos) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Volt: ");
  display.print(voltage, 2);
  display.println(" V");
  
  display.print("LDR: ");
  display.print(tl);
  display.print(",");
  display.print(tr);
  display.print(",");
  display.print(bl);
  display.print(",");
  display.println(br);
  
  display.print("Servo H: ");
  display.print(hPos);
  display.print(" V: ");
  display.println(vPos);
  
  display.println("------------");
  
  int lightLevel = (tl + tr + bl + br) / 4;
  display.print("Light: ");
  display.println(lightLevel);
  
  display.display();
}