//pinouts
#define KILL_SWITCH 12
#define LED_PIN 3

#define LEFT_DISTANCE_PIN 4
#define RIGHT_DISTANCE_PIN 5
#define FRONT_DISTANCE_PIN 6
#define FLOOR_DISTANCE_PIN 7
#define DIAGONAL_DISTANCE_PIN 6

#define ARM_PIN 3
#define WINCH_PIN 2
#define HOOK_PIN 5
#define LATCH_PIN 4

#define SECRET_SSID "Timtimtimtimtim"
#define SECRET_PASS "ajs35ddagtxq7m2"
#define PACKET_SIZE 48

#define IR_MODEL 1080
#define FRONT_IR_PIN A4
#define LEFT_IR_PIN A1
#define RIGHT_IR_PIN A2
#define DOWN_IR_PIN A3

#define ARM_PIN 3
#define WINCH_PIN 2
#define HOOK_PIN 5
#define LATCH_PIN 4

enum Section{
    LINE_FOLLOW,
    WALL_FOLLOW,
    LAVA_FLOOR,
    LUNAR_SURFACE,
    GIANTS_CAUSEWAY,
    ZIPLINE,
    TESTING
};

const Section section_order[] = {Section::LINE_FOLLOW, Section::WALL_FOLLOW, Section::LINE_FOLLOW,};

const int BASE_SPEED = 70; //base speed of robot as percentage

const int MAX_FRONT_PWM = 700;
const int MAX_BACK_PWM = 800;
const int NUM_SENSOR_AVG = 5; //number of times to read average of distance sensor to reduce noise
