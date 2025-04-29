
//temp values for now
#define KILL_SWITCH 2
#define LED_PIN 3

#define LEFT_DISTANCE_PIN 4
#define RIGHT_DISTANCE_PIN 5
#define FRONT_DISTANCE_PIN 6
#define FLOOR_DISTANCE_PIN 7

const int BASE_SPEED = 255; //base speed of robot in PWM

//distance sensor distances
float left_distance = 0.0f;
float right_distance = 0.0f;
float front_distance = 0.0f;
float floor_distance = 0.0f;


// ---------- Sensor Reading Functions ----------

float read_distance_sensor(int pin){

  //read distance
  float distance = 0.0f; // (temp value for now until we implement specific sensors)

  //read multiple times and take average to reduce noise

  return distance
}


// ---------- Util Functions ----------

void check_kill_switch(){
  //physical switch
  if (digitalRead(KILL_SWITCH)){
    move(0,0); //stop robot movement

    //force get stuck in loop to stop all movement
    while (true){
      Serial.println("Kill switch activated!");
      delay(1000);
    }
  }

  //implement wifi switch here
}


void move(float left, float right){
  //implement setting the PWMs for all 6 wheels
}


void line_following(){
  int left_speed = BASE_SPEED;
  int right_speed = BASE_SPEED;

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

  inputMode(KILL_SWITCH, INPUT);

  Serial.println("Initialising system...");

  //initialise all sensors here

  Serial.println("Initialisation completed!");
}

void loop(){

  //implement stuff here idk
  //layout order of sections when we know and testing stuff

  check_kill_switch();
}
