#include <Servo.h>

Servo horizontalServo;
Servo verticalServo;

const int ldrTL = A0;
const int ldrTR = A1;
const int ldrBL = A2;
const int ldrBR = A3;
const int solarPanel = A4;

int horizontalPos = 90;
int verticalPos = 90;
int tolerance = 10;

void setup() {
  Serial.begin(9600);
  horizontalServo.attach(9);
  verticalServo.attach(10);
  
  findMaxLightPosition();
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
  
  delay(10);
}

void findMaxLightPosition() {
  int maxLight = 0;
  int bestH = 90;
  int bestV = 90;
  
  for (int h = 0; h <= 180; h += 30) {
    for (int v = 0; v <= 180; v += 30) {
      horizontalServo.write(h);
      verticalServo.write(v);
      delay(500);
      
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