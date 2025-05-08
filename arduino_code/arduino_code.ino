#include <Motoron.h> //library for motoron motor driver
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>

#include "constants.h"

MotoronI2C mdL;

//MotoronI2C mdL(0x10);
//MotoronI2C mdR(0xF);

Servo arm_servo;

const int BASE_SPEED = 255; //base speed of robot in PWM
int left_speed = 0;
int right_speed = 0;

//distance sensor distances
float left_distance = 0.0f;
float right_distance = 0.0f;
float front_distance = 0.0f;
float floor_distance = 0.0f;

//WiFi variables
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status


// ---------- Sensor Reading Functions ----------

float read_distance_sensor(int pin){

  //read distance
  float distance = 0.0f; // (temp value for now until we implement specific sensors)

  //read multiple times and take average to reduce noise

  return distance;
}


// ---------- Util Functions ----------

void check_kill_switch(){
  //physical switch
  if (digitalRead(KILL_SWITCH)){
    shutdown();
  }

  //implement wifi switch here
}

void shutdown(){
  move(0,0); //stop robot movement

  //force get stuck in loop to stop all movement
  while (true){
    Serial.println("Kill switch activated!");
    delay(1000);
  }
}


void move(int left, int right){
  //implement setting the PWMs for all 6 wheels

  // mdL.setSpeed(1, left);
  // mdL.setSpeed(2, left);
  // mdL.setSpeed(3, left);

  // mdR.setSpeed(1, right);
  // mdR.setSpeed(2, right);
  // mdR.setSpeed(3, right);
}


void line_following(){
  //implement logic here

}


void wall_following(){
  int left_speed = BASE_SPEED;
  int right_speed = BASE_SPEED;

  bool following = true;

  while (following){
    left_distance = read_distance_sensor(LEFT_DISTANCE_PIN);
    right_distance = read_distance_sensor(RIGHT_DISTANCE_PIN);
    front_distance = read_distance_sensor(FRONT_DISTANCE_PIN);


    //implement logic here
  }
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
  // arm_servo.attach(2);

  Serial.println("Initialisation completed!");

  delay(100);
}

void loop(){
  // Serial.println("On!");
  // mdL.setSpeed(1, 32000);
  // delay(2000);

  // Serial.println("Off!");
  // mdL.setSpeed(1, 0);
  // delay(2000);

  // for (int pos = 0; pos <= 180; pos += 1) { // Move the servo from 0 to 180 degrees
  //   arm_servo.write(pos); // Set the servo position
  //   delay(15); // Wait for 15 milliseconds
  // }
  
  // for (int pos = 180; pos >= 0; pos -= 1) { // Move the servo from 180 to 0 degrees
  //   arm_servo.write(pos);
  //   delay(15);
  // }


  //implement stuff here idk
  //layout order of sections when we know and testing stuff

  check_kill_switch();

}
