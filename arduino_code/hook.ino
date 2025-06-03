#include <Servo.h>
#include <string.h>
Servo myServo1;
Servo myServo2;
Servo myServo3;
const int sensor1 = A0;
const int sensor2 = A1;
char state='d';

void setup() {
  myServo1.attach(2);                         // 信号线接 D9
  myServo2.attach(6);   
  myServo3.attach(9);   
  Serial.begin(9600);
  myServo2.write(90);
  //myServo3.write(0);
}

void loop() {
  // 空着就行，或者根据需要重新 attach() 再给脉冲  
  // int sensorValue = analogRead(A0); // 读取模拟输入 A0 的值（范围 0-1023）
  // Serial.println(sensorValue);      // 将值打印到串口监视器
  // int degree=sensorValue*18/(102.3);
  // Serial.println(degree);  
  // myServo1.write(degree);
  // delay(100);
  float analogValue1 = analogRead(sensor1); // 读取模拟值（0-4095）
  float voltage1 = analogValue1 * (5 / 1600.0);
  float distance1 = calculateDistance(voltage1);
  float analogValue2 = analogRead(sensor2); // 读取模拟值（0-4095）
  float voltage2 = analogValue2 * (5 / 1600.0);
  float distance2 = calculateDistance(voltage2);
  Serial.print("Distance1: ");
  Serial.print(distance1);
  Serial.print(" | Distance2: ");
  Serial.println(distance2);
  if (distance1 > 15 && strcmp(state,'d')==0)
  {
    
    Serial.println("rise");
    rise();
    state='r';
    Serial.println("Done");
  }
  else 
  {
    if ((distance1 < 15 && distance2 < 15) && strcmp(state,'d')!=0)
    {
      
      Serial.println("down");
      down();
      state='d';
      Serial.println("Done");
      
    }
  }
  delay(500);
  // myServo1.write(155);
  // delay(500);
  // myServo2.write(90);
  // delay(500);
  // myServo1.write(50);
  // delay(1000);
  // myServo3.write(82);
  // delay(500);
  // myServo2.write(15);
  // delay(500);
  // myServo3.write(0);
  // delay(2000);

}

void rise()
{
  myServo1.write(50);
  delay(1000);
  myServo3.write(155);
  delay(1000);
}

void down()
{
  myServo2.write(15);
  delay(500);
  myServo3.write(0);
  delay(500);
  myServo1.write(155);
  delay(500);
  myServo2.write(90);
  delay(1000);
}

float calculateDistance(float voltage) {
  // 使用反比例公式或查表法，此处为近似公式  
  if (voltage < 0.4) return 15.1; // 超出范围
  else if (voltage > 2.8) return 2.0;
  //else return 13.0 / (voltage - 0.1); // 示例公式，需调整系数
  else return 6.638 / (voltage )-1.307;
}

