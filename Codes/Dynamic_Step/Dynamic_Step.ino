#include <Servo.h>

// --- 舵机对象 ---
Servo horizontalServo;
Servo verticalServo;

// --- 传感器引脚定义 ---
const int ldrTL = A0; // Top-Left LDR
const int ldrTR = A1; // Top-Right LDR
const int ldrBL = A2; // Bottom-Left LDR
const int ldrBR = A3; // Bottom-Right LDR
const int solarPanel = A4; // Solar panel voltage reading

// --- 位置变量 ---
int horizontalPos = 90; // 初始水平位置
int verticalPos = 90;   // 初始垂直位置
int tolerance = 25;     // 允许的误差范围
int stepSize = 5;       // 增加步长，加快移动速度

// --- 动态步长控制 ---
int dynamicStepSize = 5;  // 动态步长
int maxStepSize = 10;     // 最大步长
int minStepSize = 2;      // 最小步长

void setup() {
  Serial.begin(9600);
  horizontalServo.attach(5); // 水平舵机连接到引脚 5
  verticalServo.attach(6);   // 垂直舵机连接到引脚 6
  
  // 运行一次，在启动时找到大致的最佳光照位置
  // findMaxLightPosition(); 
  
  horizontalServo.write(horizontalPos);
  verticalServo.write(verticalPos);
  delay(1000); // 初始位置稳定时间
}

void loop() {
  // 1. 读取 LDR 传感器值
  int tl = analogRead(ldrTL);
  int tr = analogRead(ldrTR);
  int bl = analogRead(ldrBL);
  int br = analogRead(ldrBR);
  
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
    // 根据误差方向调整垂直位置
    if (errorVertical > 0) {
      verticalPos += verticalStep; // 顶部更亮，向上转动
    } else {
      verticalPos -= verticalStep; // 底部更亮，向下转动
    }
    
    // 限制舵机角度在 0 到 180 度之间
    verticalPos = constrain(verticalPos, 0, 180);
    verticalServo.write(verticalPos);
  }
  
  // 6. 水平方向调整
  if (abs(errorHorizontal) > tolerance) {
    // 检查垂直位置是否翻转
    bool isFlipped = (verticalPos > 90); 
    
    // 根据垂直翻转状态，决定水平方向的调整
    if (isFlipped) {
      // 垂直翻转 (> 90度) 时，水平控制方向需要反转
      if (errorHorizontal > 0) {
        horizontalPos += horizontalStep; // 左侧更亮，向右转动（反转后）
      } else {
        horizontalPos -= horizontalStep; // 右侧更亮，向左转动（反转后）
      }
    } else {
      // 垂直未翻转 (<= 90度) 时，使用正常的逻辑
      if (errorHorizontal > 0) {
        horizontalPos -= horizontalStep; // 左侧更亮，向左转动
      } else {
        horizontalPos += horizontalStep; // 右侧更亮，向右转动
      }
    }
    
    // 限制舵机角度在 0 到 180 度之间
    horizontalPos = constrain(horizontalPos, 0, 180);
    horizontalServo.write(horizontalPos);
  }
  
  // 7. 串行监视器输出
  int solarValue = analogRead(solarPanel);
  float solarVoltage = solarValue * (5.0 / 1023.0);
  
  Serial.print("Vout: ");
  Serial.print(solarVoltage, 2); 
  Serial.print("V | Error H:");
  Serial.print(errorHorizontal);
  Serial.print(" V:");
  Serial.print(errorVertical);
  Serial.print(" | Step H:");
  Serial.print(horizontalStep);
  Serial.print(" V:");
  Serial.print(verticalStep);
  Serial.print(" | Pos H:");
  Serial.print(horizontalPos);
  Serial.print(" V:");
  Serial.print(verticalPos);
  
  // 输出翻转状态，以便调试
  Serial.print(" | Flipped: ");
  Serial.println(verticalPos > 90 ? "YES" : "NO");
  
  delay(20); // 减少延迟，加快响应速度
}

int calculateDynamicStep(int error) {
  int absError = abs(error);
  
  if (absError > 200) {
    return 20; 
  } else if (absError > 100) {
    return 15;         
  } else if (absError > 50) {
    return 7;   
  } else if (absError > 20) {
    return 3;
  } else {
    return minStepSize;
  }
}

// 快速初始搜索函数（可选，需要时启用）
void findMaxLightPosition() {
  Serial.println("--- Starting Fast Initial Search ---");
  int maxLight = 0;
  int bestH = 90;
  int bestV = 90;
  
  // 每隔 45 度进行快速扫描（减少扫描点）
  for (int h = 0; h <= 180; h += 45) {
    horizontalServo.write(h);
    delay(100); // 减少延迟
    
    for (int v = 0; v <= 180; v += 45) {
      verticalServo.write(v);
      delay(100); // 减少延迟
      
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
  Serial.print(bestV);
  Serial.print(" | Max Light Value: ");
  Serial.println(maxLight);
  Serial.println("--- Initial Search Complete ---");
}

// 快速移动到指定位置函数（可选）
void moveToPosition(int targetH, int targetV, int speed = 10) {
  // 水平移动
  while (abs(horizontalPos - targetH) > speed) {
    if (horizontalPos < targetH) {
      horizontalPos += speed;
    } else {
      horizontalPos -= speed;
    }
    horizontalPos = constrain(horizontalPos, 0, 180);
    horizontalServo.write(horizontalPos);
    delay(30);
  }
  
  // 垂直移动
  while (abs(verticalPos - targetV) > speed) {
    if (verticalPos < targetV) {
      verticalPos += speed;
    } else {
      verticalPos -= speed;
    }
    verticalPos = constrain(verticalPos, 0, 180);
    verticalServo.write(verticalPos);
    delay(30);
  }
  
  // 最终精确位置
  horizontalServo.write(targetH);
  verticalServo.write(targetV);
  horizontalPos = targetH;
  verticalPos = targetV;
}