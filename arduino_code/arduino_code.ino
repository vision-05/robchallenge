#include <Motoron.h> //library for motoron motor driver
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "constants.h"

//initialise classes
MotoronI2C mdL;

//MotoronI2C mdL(0x10);
//MotoronI2C mdR(0xF);

Servo sensor_servo;

WiFiUDP Udp; //UDP to receive WiFi signal


int left_speed = 0; //percentage for left speed -100% to 100%
int right_speed = 0; //percentage for right speed -100% to 100%

//pwms for speed (need 4 as back wheels use different motors)
int front_left_speed = 0;
int front_right_speed = 0;
int back_left_speed = 0;
int back_right_speed = 0;

//distance sensor distances
float left_distance = 0.0f;
float right_distance = 0.0f;
float front_distance = 0.0f;
float floor_distance = 0.0f;

//WiFi variables
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status
unsigned int localPort = 2390; // local port to listen for UDP packets

const int PACKET_SIZE = 48;
byte packetBuffer[PACKET_SIZE]; // buffer to hold incoming and outgoing packets


bool button_pressed = false;

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
  for (int i = 0; i < NUM_SENSOR_AVG){
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

  Udp.read(packetBuffer, NTP_PACKET_SIZE);

  word(packetBuffer)
}

void check_kill_switch(){
  //physical switch
  if (digitalRead(KILL_SWITCH)){
    shutdown();
  }

  //wifi switch
  if (read_udp()){
    shutdown();
  }
  
}

void shutdown(){
  move(0,0); //stop robot movement

  bool stopped = true;

  //force get stuck in loop to stop all movement
  while (stopped){
    Serial.println("Kill switch activated!");
    delay(1000);

    //if switch detected again, continue operation
    if (digitalRead(KILL_SWITCH)){
      stopped = false;
    }
  }
}

/// @brief sets the motors to the correct PWMs
/// @param left percentage speed for left motors
/// @param right percentage speed for right motors
void move(int left, int right){
  //capping percentages
  if (left > 100){
    left = 100
  } else if (left < -100){
    left = -100;
  }

  if (right > 100){
    right = 100
  } else if (right < -100){
    right = -100;
  }

  //implement setting the PWMs for all 6 wheels

  front_left_speed = round(left / 100 * MAX_FRONT_PWM);
  front_right_speed = round(right / 100 * MAX_FRONT_PWM);
  back_left_speed = round(left / 100 * MAX_BACK_PWM);
  back_right_speed = round(right / 100 * MAX_BACK_PWM);

  // mdL.setSpeed(1, front_left_speed);
  // mdL.setSpeed(2, front_left_speed);
  // mdL.setSpeed(3, back_left_speed);

  // mdR.setSpeed(1, front_right_speed);
  // mdR.setSpeed(2, front_right_speed);
  // mdR.setSpeed(3, back_right_speed);
}


bool line_following(){
  bool following = true;

  while (following){
    //tim tim please implement :)

    check_kill_switch();
  }

  //only return true once the section has been completed
  return true;
}


bool wall_following(){
  left_speed = BASE_SPEED;
  right_speed = BASE_SPEED;

  bool following = true;

  while (following){
    left_distance = read_distance_sensor(LEFT_DISTANCE_PIN);
    right_distance = read_distance_sensor(RIGHT_DISTANCE_PIN);
    front_distance = read_distance_sensor(FRONT_DISTANCE_PIN);

    //change to PID controller if needed
    if (left_distance - right_distance > 10){
      left_speed = left_speed - 1;
      right_speed = right_speed + 1;
    }
    else if (right_distance - left_distance > 10){
      left_speed = left_speed + 1;
      right_speed = right_speed - 1;
    }

    if (front_distance < 50){
      move(0,0);
    } else {
      move(left_speed, right_speed);
    }

    check_kill_switch();
    delay(50);
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

  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();

  Udp.begin(localPort);

  //initialise all sensors here:
  // Serial.println("Initialising Left Motoron Motor Driver...");
  // mdL.reinitialize();
  // mdL.disableCrc();
  // mdL.clearResetFlag();
  // Serial.println("Left Motoron Motor Driver Connected!");

  // Serial.println("Initialising Right Motoron Motor Driver...");
  // mdR.reinitialize();
  // mdR.disableCrc();
  // mdR.clearResetFlag();
  // Serial.println("Right Motoron Motor Driver Connected!");

  // Serial.println("Attaching Servos!");
  // sensor_servo.attach(2);

  Serial.println("Initialisation completed!");

  delay(100);
  
  while(!button_pressed){
    button_pressed = digitalRead(KILL_SWITCH);
  }
}

void loop(){
  bool finished = false;

  switch (current_section)
  {
    case LINE_FOLLOW:
      finished = line_following();
      break;

    case WALL_FOLLOW:
      finished = wall_following();
      break;
    
    default:
      break;
  }

  if (finished){
    section_idx++;

    current_section = section_order[section_idx];
  }

  check_kill_switch();
}
