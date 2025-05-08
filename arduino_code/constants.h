//pinouts
#define KILL_SWITCH 2
#define LED_PIN 3

#define LEFT_DISTANCE_PIN 4
#define RIGHT_DISTANCE_PIN 5
#define FRONT_DISTANCE_PIN 6
#define FLOOR_DISTANCE_PIN 7

#define SECRET_SSID "yournetwork"
#define SECRET_PASS "yourpassword"

enum Section{
    LINE_FOLLOW,
    WALL_FOLLOW,
    LAVA_FLOOR,
    LUNAR_SURFACE,
    GIANTS_CAUSEWAY,
    ZIPLINE,
};

const Section section_order[] = [LINE_FOLLOW];

const int BASE_SPEED = 50; //base speed of robot as percentage

const int MAX_FRONT_PWM = 800;
const int MAX_BACK_PWM = 800;
const int NUM_SENSOR_AVG = 5; //number of times to read average of distance sensor to reduce noise