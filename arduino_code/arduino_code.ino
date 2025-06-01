#include <Motoron.h> //library for motoron motor driver
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SharpIR.h>
#include <qtr.h>



#include "constants.h"

//initialise classes
//MotoronI2C mdR;
//MotoronI2C mdL;
QTR qtr;

MotoronI2C mdL(15);
MotoronI2C mdR(17);

Servo sensor_servo;

WiFiUDP Udp; //UDP to receive WiFi signal

//infra-red distance sensor classes (library reference: https://github.com/guillaume-rico/SharpIR)
SharpIR frontIR(FRONT_IR_PIN, IR_MODEL);
SharpIR leftIR(LEFT_IR_PIN, IR_MODEL);
SharpIR rightIR(RIGHT_IR_PIN, IR_MODEL);
SharpIR downIR(DOWN_IR_PIN, IR_MODEL);

int count = 9;

int left_speed = 0; //percentage for left speed -100% to 100%
int right_speed = 0; //percentage for right speed -100% to 100%

//pwms for speed (need 4 as back wheels use different motors)
int front_left_speed = 0;
int front_right_speed = 0;
int back_left_speed = 0;
int back_right_speed = 0;

//distance sensor distances
float front_distance = 0.0f;
float left_distance = 0.0f;
float right_distance = 0.0f;
float down_distance = 0.0f;

//WiFi variables
char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS; // the WiFi radio's status
unsigned int localPort = 2390; // local port to listen for UDP packets


byte packetBuffer[PACKET_SIZE]; // buffer to hold incoming and outgoing packets


bool button_not_pressed = true;
int msg = 0;

//section logic
int section_idx = 0;
Section current_section = section_order[section_idx];


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

void check_kill_switch(){
  //physical switch
  if (digitalRead(KILL_SWITCH)){
    msg = 1;
    delay(1000);
    shutdown();
  }

  //wifi switch
  if (read_udp()){
    msg = 1;
    shutdown();
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
  mdR.setSpeed(3, back_right_speed);
}

/// @brief Hard coded 90 degree left turn using timings
void turn_left(){
  move(-50,50);
  delay(400);
  move(0,0);
}

/// @brief Hard coded 90 degree right turn using timings
void turn_right(){
  move(50,-50);
  delay(400);
  move(0,0);
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


bool line_following(){
  bool following = true;
  int t = 0;
  while (following){
    //tim tim please implement :)
    ++t;
    if(t > 1000) {
      following = false;
    }
    check_kill_switch();
  }

  //only return true once the section has been completed
  return true;
}


bool wall_following(){
  bool following = true;

  float error = 0.0f;
  float previous_error = 0.0f;
  float sum_error = 0.0f;
  float gradient_error = 0.0f;
  float control_signal = 0.0f;

  const float Kp = 1.5f;
  const float Ki = 0.2f;
  const float Kd = 0.4f;

  int left = 0;
  int right = 0;


  while (following){
    front_distance = frontIR.getDistance();
    left_distance = leftIR.getDistance();
    right_distance = rightIR.getDistance();
    down_distance = downIR.getDistance();

    
    error = left_distance - right_distance;

    gradient_error = error - previous_error;

    sum_error = sum_error + error;

    control_signal = floor(Kp * error + Ki * sum_error + Kd * gradient_error);

    previous_error = error;

    left = BASE_SPEED - control_signal;
    right = BASE_SPEED + control_signal;

    //avoiding head on collision
    if (front_distance < 40){
      move(0,0);
      delay(200);

      //if approac a wall in front, if the left wall is 5cm further away than the right assume 90 degree left turn and vice versa for right
      if (left_distance > right_distance + 50){
        turn_left();
        //continue moving forward so that moved out of the corner and will properly detect the distances to the walls.
        move(BASE_SPEED,BASE_SPEED);
        delay(1000);

      } else if (right_distance > left_distance + 50){
        turn_right();
        
        move(BASE_SPEED,BASE_SPEED);
        delay(1000);
      }
    } else {
      move(left, right);
    }

    check_kill_switch();
    check_stuck();
    delay(50); //small delay to allow motors to move robot before overeacting

  }

  return true;
}


// ---------- Section Functions ----------
// write each section as a seperate function

void lava_floor(){

}


void zipline(){

}


void tunnel(){

}



void setup(){
  Serial.begin(115200);
  Serial.setTimeout(1); 

  pinMode(KILL_SWITCH, INPUT);

  Wire.begin();

  Serial.println("Initialising system...");
  
  // check for the WiFi module:
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

  //initialise all sensors here:
  Serial.println("Initialising Left Motoron Motor Driver...");
  mdL.reinitialize();
  mdL.disableCrc();
  mdL.clearResetFlag();
  Serial.println("Left Motoron Motor Driver Connected!");

  Serial.println("Initialising Right Motoron Motor Driver...");
  mdR.reinitialize();
  mdR.disableCrc();
  mdR.clearResetFlag();
  Serial.println("Right Motoron Motor Driver Connected!");

  for(int i = 0; i < 3; ++i) {
    mdR.setMaxAcceleration(i,80);
    mdL.setMaxAcceleration(i,80);
    mdR.setMaxDeceleration(i,300);
    mdL.setMaxDeceleration(i,300);  
  }

  Serial.println("Attaching Servos!");
  sensor_servo.attach(SERVO_PIN);

  Serial.println("Initialisation completed!");

  delay(100);
  
  //wait for start button
  while(!button_not_pressed){
    
    button_not_pressed = digitalRead(KILL_SWITCH);
    Serial.println(button_not_pressed);
  }

  Serial.println("Starting main sequence");

  qtr.setTimeout(1000);
  uint8_t sensors[count] = {36,38,40,42,44,46,48,50,52};
  qtr.setSensorPins(sensors,count);
}

void loop(){
  bool finished = false;
  String input_string = "";
  

  switch (current_section)
  {
    case LINE_FOLLOW:
      for(uint16_t i = 0; i < 20; i++) {
        qtr.calibrate(10, Emitter::Off, Parity::EvenAndOdd);
        delay(500);
      }
      
      finished = line_following();
      break;

    case WALL_FOLLOW:
      finished = wall_following();
      break;

    case TESTING:
      char direction;
      int speed;
       // To hold incoming data

      input_string = Serial.readString();

      if (input_string.length() > 1) {
        direction = input_string.charAt(0); // L or R
        String number = input_string.substring(1);
        speed = number.toInt(); // Convert to integer

        if (direction == 'L'){
          left_speed = speed;
        } else if (direction == 'R'){
          right_speed = speed;
        }

        Serial.print("L: ");
        Serial.print(left_speed);
        Serial.print(" R: ");
        Serial.println(right_speed);
      }

      move(left_speed, right_speed);

    
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
        Serial.print("Front IR: ");
        Serial.println(frontIR.getDistance());

        qtr.readSensors();
      for(uint8_t i = 0; i < count; i++) {
        Serial.print(qtr[i]);
        Serial.print(' ');
      }
      Serial.println();
      qtr.readCalibrated();
      for(uint8_t i = 0; i < count; i++) {
        Serial.print(qtr[i]);
        Serial.print(' ');
      }
      Serial.println();
      qtr.readBlackLine();
      for(uint8_t i = 0; i < count; i++) {
        Serial.print(qtr[i]);
        Serial.print(' ');
      }
      Serial.println();
        delay(1000);
        check_kill_switch();
      }
    }

    current_section = section_order[section_idx];

    finished = false;
  }

  check_kill_switch();
}
