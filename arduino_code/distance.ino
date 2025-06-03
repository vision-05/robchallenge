const int sensor1 = A0; // 传感器连接到A0
const int sensor2 = A1;
const int sensor3 = A2;

void setup() {
  Serial.begin(9600); // 初始化串口通信
}

void loop() {
  float analogValue1 = analogRead(sensor1); // 读取模拟值（0-4095）
  float voltage1 = analogValue1 * (5 / 1600.0); // 转换为电压值（假设ADC为12位）
  float analogValue2 = analogRead(sensor2); // 读取模拟值（0-4095）
  float voltage2 = analogValue2 * (5 / 1600.0);
  float analogValue3 = analogRead(sensor3); // 读取模拟值（0-4095）
  float voltage3 = analogValue3 * (5 / 1600.0);

  // 根据电压计算距离（示例公式，需根据实际校准调整）
  float distance1 = calculateDistance(voltage1);
  float distance2 = calculateDistance(voltage2);
  float distance3 = calculateDistance(voltage3);

  // Serial.print("analog: ");
  // Serial.print(analogValue);
  // Serial.print(" | Voltage: ");
  // Serial.print(voltage);
  // Serial.print("V | Distance: ");
  Serial.print("Distance1: ");
  Serial.print(distance1);
  Serial.print(" cm|Distance2: ");
  Serial.print(distance2);
  Serial.print(" cm|Distance3: ");
  Serial.print(distance3);
  Serial.println(" cm");



  delay(100); // 适当延迟
}

// 示例转换函数（需根据传感器数据手册校准）
float calculateDistance(float voltage) {
  // 使用反比例公式或查表法，此处为近似公式  
  if (voltage < 0.4) return 15.0; // 超出范围
  else if (voltage > 2.8) return 2.0;
  //else return 13.0 / (voltage - 0.1); // 示例公式，需调整系数
  else return 6.638 / (voltage )-1.307;
}