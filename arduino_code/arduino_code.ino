#include <Motoron.h> //library for motoron motor driver
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SharpIR.h>
#include <qtr.h>
#include <string.h>

#include "constants.h"

MotoronI2C mdL(15);
MotoronI2C mdR(17);

Servo sensorServo;
Servo unlockServo;
Servo lockServo;
Servo liftServo;

WiFiUDP Udp; //UDP to receive WiFi signal

//infra-red distance sensor classes (library reference: https://github.com/guillaume-rico/SharpIR)
SharpIR frontIR(FRONT_IR_PIN, IR_MODEL);
SharpIR leftIR(LEFT_IR_PIN, IR_MODEL);
SharpIR rightIR(RIGHT_IR_PIN, IR_MODEL);
SharpIR downIR(DOWN_IR_PIN, IR_MODEL);

int left_speed = 0; //percentage for left speed -100% to 100%
int right_speed = 0; //percentage for right speed -100% to 100%

//pwms for speed (need 4 as back wheels use different motors)
int front_left_speed = 0;
int front_right_speed = 0;
int back_left_speed = 0;
int back_right_speed = 0;

//distance sensor distances
double front_distance = 0.0f;
double left_distance = 0.0f;
double right_distance = 0.0f;
double down_distance = 0.0f;

//WiFi variables
char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS; // the WiFi radio's status
unsigned int localPort = 2390; // local port to listen for UDP packets


byte packetBuffer[PACKET_SIZE]; // buffer to hold incoming and outgoing packets


bool button_not_pressed = true;
int msg = 0;
int killStatus = 0;

//section logic
int section_idx = 0;
Section current_section = section_order[section_idx];


//Line following
const int count = 9;
const int halfwidth = 6;
constexpr float invRadius = 1/6.5;
const float invArrayCount = 0.333;
QTR qtrs[3];
uint8_t sensors[3][count] = {{22,24,26,28,30,32,34,36,38},
                             {23,25,27,29,31,33,35,37,39},
                             {42,43,44,45,46,47,48,49,50}};

//allow for changing over serial
float kp = 0.04f; //p225
float kd = 0.0f; //d5
float ki = 0.0f;//i0.1

int blackthresholds[3] = {5,5,5};
const float samplet = 0.3;
constexpr float invSamplet = 1/0.3;
constexpr float invCount = 1/9;

int32_t prevErrors[3][count] = {{0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0}};


float sum(float* arr) {
  int s = 0;
  for(int i = 0; i < count; ++i) {
    s += arr[i];
  }
  return s;
}



// ---------- Sensor Reading Functions ----------

/// @brief measure distance to wall in mm
/// @param pin the digital pin which the sensor is connected to
/// @return distance in mm
float read_distance_sensor(int pin){
  float distance = 0.0f;
  float total = 0.0f;
  
  //read multiple times and take average to reduce noise
  for (int i = 0; i < NUM_SENSOR_AVG; ++i){
    distance = 0.0f; //implement actual logic to read sensor here

    total = total + distance;
  }


  return total / NUM_SENSOR_AVG;
}


// ---------- Util Functions ----------

int read_udp(){
  if (!Udp.parsePacket()){
    return 0;
  } 

  Udp.read(packetBuffer, PACKET_SIZE);
  return 1;
}

int handleCommand(char command, String payload, int kill) {
  float k;
  Serial.print("Command: ");
  Serial.println(command);
  if(command == 'p') {
    k = payload.toFloat()/100;
    kp = k;
    return kill;
  }
  if(command == 'i') {
    k = payload.toFloat()/100;
    ki = k;
    return kill;
  }
  if(command == 'd') {
    k = payload.toFloat()/100;
    kd = k;
    return kill;
  }
  if(command == 'f') {
    return 0;
  }
  if(command == 'k') {
    return 1;
  }
  return kill;
}

String readSerialCommand(char* com) {
  int incomingByte;
  int c = -1;
  int idx = 0;
  int bten[4] = {1000,100,10,1};
  int res = 0;
  while (Serial.available() > 0) {
    incomingByte = Serial.read();
    if(c == -1) {
      if(incomingByte > 58) {
        char command = incomingByte;
        String f = Serial.readString();
        float k = f.toFloat()/100;
        Serial.print("K: ");
        Serial.println(k);
        if(command == 'k') {
          mdL.setSpeed(1,0);
          mdL.setSpeed(3,0);

          mdR.setSpeed(1,0);
          mdR.setSpeed(3,0);
          while(!Serial.available()) {
            Serial.println("killed");
          }
          break;
        }
        if(command == 'p') {
          kp = k;
        }
        if(command == 'i') {
          ki = k;
        }
        if(command == 'd') {
          kd = k;
        }
        break;
      }
      idx = incomingByte-48;
      ++c;
      continue;
    }
    if(c == 4) {
      Serial.println(res);
      blackthresholds[idx] = res;
      Serial.println(qtrs[idx].getThreshold());
      delay(3000);
      break;
    }
    res += bten[c]*(incomingByte-48);
    Serial.println(incomingByte);
    ++c;
  }
}

String readUDPCommand(char* com) {
  int packSize = Udp.parsePacket();
  Serial.println(packSize);
  if(packSize > 1) {
    Udp.read(packetBuffer, 48);
    *com = packetBuffer[0];
    String f = (char*)(packetBuffer + sizeof(char));
    Serial.println(f);
    return f;
    }
  if(packSize == 1) {
    Udp.read(packetBuffer, 48);
    *com = packetBuffer[0];
    return "nil";
  }
  return "none";
}

void setupUDP() {
  while (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    delay(500);
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);

    status = WiFi.begin(ssid, pass);

    // 5 second delay waiting for connection
    delay(5000);
  }

  Serial.println("You're connected to the network");
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);

  Udp.begin(localPort);
}

void setupServos() {
  sensorServo.attach(ARM_PIN);
  unlockServo.attach(LATCH_PIN);
  lockServo.attach(HOOK_PIN);
  liftServo.attach(WINCH_PIN);
}

void setupMotors() {
  Serial.println("Initialising Left Motoron Motor Driver...");
  Wire.begin();
  mdL.reinitialize();
  mdL.disableCrc();
  mdL.clearResetFlag();
  Serial.println("Left Motoron Motor Driver Connected!");

  Serial.println("Initialising Right Motoron Motor Driver...");
  mdR.reinitialize();
  mdR.disableCrc();
  mdR.clearResetFlag();
  Serial.println("Right Motoron Motor Driver Connected!");

  mdR.setMaxAcceleration(1,80);
  mdL.setMaxAcceleration(1,80);
  mdR.setMaxDeceleration(1,300);
  mdL.setMaxDeceleration(1,300);
  mdR.setMaxAcceleration(3,80);
  mdL.setMaxAcceleration(3,80);
  mdR.setMaxDeceleration(3,300);
  mdL.setMaxDeceleration(3,300);
}

void setupQTR() {
  for(int i = 0; i < 3; ++i) {
    qtrs[i].setTimeout(600);
    qtrs[i].setSensorPins(sensors[i],count);
    qtrs[i].setThreshold(&blackthresholds[i]);
  }

  for(uint16_t i = 0; i < 20; i++) {
    for(int i = 0; i < 3; ++i) {
      qtrs[i].calibrate(10, Emitter::Off, Parity::EvenAndOdd);
      delay(100);
    }
  }


  for(int j = 0; j < 3; ++j) {
    for(uint8_t i = 0; i < count; i++) {
      Serial.print(qtrs[j].getCalMin(1));
      Serial.print(' ');
    }
    Serial.println();

    for(uint8_t i = 0; i < count; i++) {
      Serial.print(qtrs[j].getCalMax(1));
      Serial.print(' ');
    }
    Serial.println();
  }

  //qtr.setEvenEmitter(3);
  //qtr.setOddEmitter(2);

  //qtr.emitterTest();
}

void check_kill_switch(){
  //physical switch
  if (digitalRead(KILL_SWITCH)){
    Serial.println("Detect killswitch");
    unsigned long t0 = millis();

    while (digitalRead(KILL_SWITCH)){
      if (millis() - t0 > 100){
        Serial.println("Switching state");
        if (killStatus == 1){
          killStatus = 0;
        } else {
          killStatus = 1;
        }

        break;
      } 
    }
  }
  
}

void shutdown(){
  move(0,0); //stop robot movement

  bool stopped = true;

  //force get stuck in loop to stop all movement
  while (stopped){
    if(read_udp()) {
      msg = 0;
    }
    Serial.println("Kill switch activated!");

    //if switch detected again, continue operation
    if (digitalRead(KILL_SWITCH)){
      Serial.println("Resuming");
      stopped = false;
      delay(1000);
    }

    if(!msg) {
      Serial.println("Resuming");
      stopped = false;
      delay(1000);
    }
  }
  Serial.println("Finished");
}

/// @brief sets the motors to the correct PWMs
/// @param left percentage speed for left motors
/// @param right percentage speed for right motors
void move(int left, int right){
  //capping percentages
  if (left > 100){
    left = 100;
  } else if (left < -100){
    left = -100;
  }

  if (right > 100){
    right = 100;
  } else if (right < -100){
    right = -100;
  }

  //implement setting the PWMs for all 6 wheels

  front_left_speed = round((float)left / 100 * MAX_FRONT_PWM);
  front_right_speed = round((float)right / 100 * MAX_FRONT_PWM);
  back_left_speed = round((float)left / 100 * MAX_BACK_PWM);
  back_right_speed = round((float)right / 100 * MAX_BACK_PWM);

  //Used for debugging PWMs for left and right wheels
  // Serial.println("Speeds:");
  // Serial.println(front_left_speed);
  // Serial.println(front_right_speed);
  // Serial.println(back_left_speed);
  // Serial.println(back_right_speed);

  mdL.setSpeed(1, -front_left_speed);
  mdL.setSpeed(2, -front_left_speed);
  mdL.setSpeed(3, back_left_speed);

  mdR.setSpeed(1, front_right_speed);
  mdR.setSpeed(2, front_right_speed);
  mdR.setSpeed(3, -back_right_speed);
}

void check_stuck(){
  front_distance = frontIR.getDistance();
  left_distance = leftIR.getDistance();
  right_distance = rightIR.getDistance();
  down_distance = downIR.getDistance();

  //if distance to floor is greater than 25cm attempt to reverse to get unstuck
  if (down_distance > 25){
    move(-50,-50);
    delay(1000);
    move(0,0);
  }

  //should implement more logic to check if front, left, right distance don't change for a while then turn/wiggle etc
}

void PID(float velocity) {
    //sensors to errors
    float errors[3][count] = {{-12,-9,-6,-3,0,3,6,9,12},
                              {-8,-6,-4,-2,0,2,4,6,8},
                              {-4,-3,-2,-1,0,1,2,3,4}};

    //if either of left two and either of right two and no middle then
    if ((qtrs[0][6] || qtrs[0][7] || qtrs[0][8]) && (qtrs[0][0] || qtrs[0][1] || qtrs[0][2]) && qtrs[0][4] && (qtrs[0][3] || qtrs[0][5])){
      //turn left, then go forwards briefly
      move(-BASE_SPEED, BASE_SPEED);
      delay(500);

      move(BASE_SPEED, BASE_SPEED);
      delay(300);
    }

    if (frontIR.getDistance() < 40){
      move(-BASE_SPEED, -BASE_SPEED);
      delay(500);

      move(BASE_SPEED, -BASE_SPEED);
      delay(500);

      move(BASE_SPEED, BASE_SPEED);
      delay(300);
    }

    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < count; ++j) {
        errors[i][j] *= qtrs[i][j];
      }
    }

    //derivative errors
    float derivErrors[3][count];
    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < count; ++j) {
        derivErrors[i][j] = (errors[i][j] - prevErrors[i][j])*10; //divide optimised at compile time
      }
    } 

    //"integral" errors
    float integralErrors[3][count];
    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < count; ++j) {
        integralErrors[i][j] = (errors[i][j] + prevErrors[i][j])*samplet; //divide optimised at compile time
      }
    } 

    //total steering signal
    float pe[3] = {sum(errors[0]),sum(errors[1]),sum(errors[2])};
    float de[3] = {sum(derivErrors[0]),sum(derivErrors[1]),sum(derivErrors[2])};
    float ie[3] = {sum(integralErrors[0]),sum(integralErrors[1]),sum(integralErrors[2])};

    float mpe = (pe[0] + pe[1] + pe[2])*invArrayCount;
    float mde = (de[0] + de[1] + de[2])*invArrayCount;
    float mie = (ie[0] + ie[1] + ie[2])*invArrayCount;

    float dtheta = -kp*mpe - kd*mde + ki*mie;

    // if (dtheta < 0.05f){
    //   dtheta = 0;
    // }

    Serial.print("DTheta ");
    Serial.print(dtheta);
    Serial.println();
    float omega = dtheta*invSamplet;
    Serial.print("Omega ");
    Serial.print(omega);
    Serial.println();
    
    //inverse kinematics

    float phiL = (velocity*invRadius) - (omega*invRadius*0.3);
    float phiR = (velocity*invRadius) + (omega*invRadius*0.3);


    if (phiL < 0.7 && phiR < 0.7){
      phiL = phiL * 1.8;
      phiR = phiR * 1.8;
    }
    else {
      phiL = phiL * 1.25;
      phiR = phiR * 1.25;
    }

    Serial.print("PhiL: ");
    Serial.print(phiL);
    Serial.print("PhiR: ");
    Serial.print(phiR);
    Serial.println();

    //run motors
    move(250 + 800 * phiL, 250 + 800 * phiR);

    //set prev errors for next loop
    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < count; ++j) {
        prevErrors[i][j] = errors[i][j];
      }
    }

    delay(50);
}

void readSendArrays(QTR* a) {
  a[0].readCalibrated();
  a[1].readCalibrated();
  a[2].readCalibrated();

  for(int i = 0; i < 3; ++i) {
    std::string nums = std::to_string(i+1) + "c"; //key is calibrated
    for(int j = 0; j < count; ++j) {
      nums += std::to_string(a[i][j]);
      nums += " ";
    }
    Serial.println(nums.c_str());
    Udp.beginPacket("10.17.186.85",2300);
    Udp.write(nums.c_str(),nums.length());
    Udp.endPacket();
  }
}


bool line_following(){
  bool following = true;

  char command;
  String payload;
  //payload = readSerialCommand();
  payload = readUDPCommand(&command);
  killStatus = handleCommand(command, payload, killStatus);

  readSendArrays(qtrs);
  
  qtrs[0].readBlackLine();
  qtrs[1].readBlackLine();
  qtrs[2].readBlackLine();

  for(int i = 0; i < 3; ++i) {
    std::string nums = std::to_string(i+1) + "b"; //key is blacklined
    for(int j = 0; j < count; ++j) {
      nums += std::to_string(qtrs[i][j]);
      nums += " ";
    }
    Serial.println(nums.c_str());
    Udp.beginPacket("10.17.186.85",2300);
    Udp.write(nums.c_str(),nums.length());
    Udp.endPacket();
  }
  
  if(!killStatus) {
    Serial.println("PID");
    PID(2);
  }

  return following;
}

double read_distance(int pin){
  int raw = analogRead(pin);

  double calibrated = 800 / (double)raw;

  return calibrated;
}


bool wall_following(){
  bool following = true;

  double error = 0.0f;
  double previous_error = 0.0f;
  double sum_error = 0.0f;
  double gradient_error = 0.0f;
  double control_signal = 0.0f;

  const double Kp = 0.5f;
  const double Ki = 0.0001f;
  const double Kd = 0.3f;

  int left = 0;
  int right = 0;


  while (following){
    // front_distance = frontIR.getDistance();
    // left_distance = leftIR.getDistance();
    // right_distance = rightIR.getDistance();
    // down_distance = downIR.getDistance();


    front_distance = read_distance(FRONT_IR_PIN);
    left_distance = read_distance(LEFT_IR_PIN);
    right_distance = read_distance(RIGHT_IR_PIN);
    down_distance = read_distance(DOWN_IR_PIN);

    Serial.print("F: ");
    Serial.print(front_distance);
    Serial.print(" L: ");
    Serial.println(left_distance);
    // Serial.print(" R: ");
    // Serial.print(right_distance);
    // Serial.print(" D: ");
    // Serial.println(down_distance);

    std::string nums[4] = {"df", "db", "dl", "dr"}; //key is blacklined
    nums[0] += std::to_string(front_distance); nums[1] += std::to_string(down_distance);
    nums[2] += std::to_string(left_distance); nums[3] += std::to_string(right_distance);
    for(int i = 0; i < 4; ++i) {
      Udp.beginPacket("10.17.186.85",2300);
      Udp.write(nums[i].c_str(),nums[i].length());
      Udp.endPacket();
    }

    if (killStatus){
      continue;
    }

    
    error = 6.5f - left_distance;

    Serial.print("Error: ");
    Serial.println(error);

    gradient_error = error - previous_error;

    sum_error = sum_error + error;

    control_signal = floor(Kp * error + Ki * sum_error + Kd * gradient_error);

    Serial.print("Control signal: ");
    Serial.println(control_signal);

    previous_error = error;

    left = BASE_SPEED + control_signal;
    right = BASE_SPEED - control_signal;

    Serial.print(left);
    Serial.print(" ");
    Serial.println(right);

    //avoiding head on collision
    if (front_distance < 6){
      move(BASE_SPEED, -BASE_SPEED);
      
    } else {
      move(left, right);
    }

    check_kill_switch();
    //check_stuck();
    delay(50); //small delay to allow motors to move robot before overeacting

  }

  return true;
}

int readKillSwitch(int kill) {
  if (digitalRead(KILL_SWITCH)){
    unsigned long t0 = millis();

    while (digitalRead(KILL_SWITCH)){
      if (millis() - t0 > 100){
        if (kill == 1){
          return 0;
        } else {
          return 1;
        }

        break;
      } 
    }
  }
  return kill;
}

void lowerSensors() {
  sensorServo.write(50);
}

void raiseSensors() {
  sensorServo.write(0);
}

void raiseLift() {
  liftServo.write(100);
}

void lowerLift() {
  liftServo.write(0);
}

void lockHook() {
  lockServo.write(180);
}

void unlockHook() {
  unlockServo.write(90);
  lockServo.write(90);
  unlockServo.write(0);
}


// ---------- Section Functions ----------
// write each section as a seperate function

void lava_floor(){
  int aboveLava = 0;
  while(!aboveLava) {
    readSendArrays(qtrs);

    if(qtrs[0][0] > 400) {
      aboveLava = 1;
      break;
    }
  }

  raiseSensors();

  //drive until over

  lowerSensors();


}


void zipline(){

}


void tunnel(){

}



void setup(){
  Serial.begin(115200);
  Serial.setTimeout(1); 

  pinMode(KILL_SWITCH, INPUT);

  Serial.println("Initialising system...");

  setupUDP();
  setupQTR();
  setupMotors();
  setupServos();
  killStatus = 0;

  // pinMode(FRONT_IR_PIN, INPUT);
  // pinMode(LEFT_IR_PIN, INPUT);
  // pinMode(RIGHT_IR_PIN, INPUT);
  // pinMode(DOWN_IR_PIN, INPUT);

  Serial.println("Attaching Servos!");

  Serial.println("Initialisation completed!");

  delay(100);
  
  //wait for start button
  while(!button_not_pressed){
    
    button_not_pressed = digitalRead(KILL_SWITCH);
    Serial.println(button_not_pressed);
  }

  Serial.println("Starting main sequence");
}

void loop(){
  bool finished = false;
  String input_string = "";

  char command;
  String payload;
  //payload = readSerialCommand();
  payload = readUDPCommand(&command);
  killStatus = handleCommand(command, payload, killStatus);
  killStatus = readKillSwitch(killStatus);

  switch (current_section)
  {
    case LINE_FOLLOW: 
      finished = line_following();
      break;

    case WALL_FOLLOW:
      finished = wall_following();
      break;

    case TESTING:
      char direction;
      int speed;
      Serial.println("TESTING");
       // To hold incoming data
      while(1) {
        Serial.println("RUnining");
        unlockServo.write(90);
        delay(500);
        lockServo.write(90);
        delay(500);
        unlockServo.write(0);
        delay(500);
        lockServo.write(0);
        delay(500);
      }
    
    default:
      break;
  }

  

  if (finished){
    section_idx++;

    if (section_idx > sizeof(section_order) / sizeof(Section) -1){
      section_idx = 0;
      move(0,0);

      //enter infinite loop when finished all sections
      while (true){
        Serial.println("Finished!");
      }

    current_section = section_order[section_idx];

    finished = false;
  }

  check_kill_switch();
}
}