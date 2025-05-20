#include <Motoron.h>
#include <motoron_protocol.h>

#include <qtr.h>

const int count = 9;
QTR qtrs[3];
uint8_t sensors[3][count] = {{22,24,26,28,30,32,34,36,38},
                             {23,25,27,29,31,33,35,37,39},
                             {42,43,44,45,46,47,48,49,50}};

MotoronI2C mdL(15);
MotoronI2C mdR(17);

void setup() {
  Serial.begin(9600);

  Serial.println("Hello");
  for(int i = 0; i < 3; ++i) {
    qtrs[i].setTimeout(1000);
    qtrs[i].setSensorPins(sensors[i],count);
  }
  Serial.println("Set pins");

  for(uint16_t i = 0; i < 20; i++) {
    for(int i = 0; i < 3; ++i) {
      qtrs[i].calibrate(10, Emitter::Off, Parity::EvenAndOdd);
      delay(100);
    }
  }

  digitalWrite(LED_BUILTIN, LOW);

  for(uint8_t i = 0; i < count; i++) {
    Serial.print(qtrs[0].getCalMin(1));
    Serial.print(' ');
  }
  Serial.println();

  for(uint8_t i = 0; i < count; i++) {
    Serial.print(qtrs[0].getCalMax(1));
    Serial.print(' ');
  }
  Serial.println();

  //qtr.setEvenEmitter(3);
  //qtr.setOddEmitter(2);

  //qtr.emitterTest();

  //motor setup
  
  Serial.println("Initialising Left Motoron Motor Driver...");
  Wire.begin();
  mdL.reinitialize();
  Serial.println("Initd");
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

void loop() {
  qtrs[0].readBlackLine();
  for(uint8_t i = 0; i < count; i++) {
    Serial.print(qtrs[0][i]);
    Serial.print(' ');
  }
  Serial.println();
  qtrs[1].readBlackLine();
  for(uint8_t i = 0; i < count; i++) {
    Serial.print(qtrs[1][i]);
    Serial.print(' ');
  }
  Serial.println();
  qtrs[2].readBlackLine();
  for(uint8_t i = 0; i < count; i++) {
    Serial.print(qtrs[2][i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println("=================");
  delay(500);
  
  mdL.setSpeed(1-600);
  mdL.setSpeed(3,600);
  mdR.setSpeed(1,-600);
  mdR.setSpeed(3,-600);
  delay(1000);
  //mdL.setSpeed(1,0);
  //mdL.setSpeed(3,0);
  //mdR.setSpeed(1,0);
  //mdR.setSpeed(3,0);
}
