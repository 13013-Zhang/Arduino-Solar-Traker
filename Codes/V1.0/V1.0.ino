#include <Servo.h>
Servo horizontalServo;
Servo verticalServo;

int horizontalPin = 9;
int verticalPin = 10;

int ldrTop = A0;    // 上方的LDR
int ldrBottom = A1; // 下方的LDR
int ldrLeft = A2;   // 左侧的LDR
int ldrRight = A3;  // 右侧的LDR

int topValue = 0;
int bottomValue = 0;
int leftValue = 0;
int rightValue = 0;

int horizontalPos = 90;
int verticalPos = 90;

int tolerance = 20;
int stepSize = 2;

// solar panel
int panelOutput = A4;
float panelVoltage = 0;
float trueOutput = 0;

void setup() {
  Serial.begin(115200);

  horizontalServo.attach(horizontalPin);
  verticalServo.attach(verticalPin);

  horizontalServo.write(horizontalPos);
  verticalServo.write(verticalPos);
  pinMode(panelOutput, INPUT);

  // waiting for 2 seconds
  delay(2000);
}

void loop() {
  topValue = analogRead(ldrTop);
  bottomValue = analogRead(ldrBottom);
  leftValue = analogRead(ldrLeft);
  rightValue = analogRead(ldrRight);

  panelVoltage = analogRead(panelOutput);

  int verticalDifference = topValue - bottomValue;
  int horizontalDifference = leftValue - rightValue;

  trueOutput = ((panelVoltage/1024)*5);

  Serial.print(topValue);
  Serial.print(" ");
  Serial.print(bottomValue);
  Serial.print(" ");
  Serial.print(leftValue);
  Serial.print(" ");
  Serial.print(rightValue);
  Serial.print("\t");

  Serial.print("S_Out: ");
  Serial.print(trueOutput);
  Serial.print("V | V_Diff: ");
  Serial.print(verticalDifference);
  Serial.print(" | H_Diff: ");
  Serial.println(horizontalDifference);

  if (abs(verticalDifference) > tolerance) {
    if (verticalDifference > 0) {
      verticalPos += stepSize;
    } else if (verticalDifference < 0) {
      verticalPos -= stepSize;
    }
    verticalPos = constrain(verticalPos, 0, 180);
    verticalServo.write(verticalPos);
  }

  if (abs(horizontalDifference) > tolerance) {
    if (horizontalDifference > 0) {
      horizontalPos += stepSize;
    } else if (horizontalDifference < 0) {
      horizontalPos -= stepSize;
    }
    horizontalPos = constrain(horizontalPos, 0, 180);
    horizontalServo.write(horizontalPos);
  }

  delay(10);
}