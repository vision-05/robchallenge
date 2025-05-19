#include <qtr.h>

const int count = 9;
QTR qtr;
QTR qtr2;

void setup() {
  Serial.begin(9600);

  Serial.println("Hello");
  qtr.setTimeout(1000);
  uint8_t sensors[count] = {36,38,40,42,44,46,48,50,52};
  qtr.setSensorPins(sensors,count);
  Serial.println("Set pins");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for(uint16_t i = 0; i < 20; i++) {
    qtr.calibrate(10, Emitter::Off, Parity::EvenAndOdd);
    delay(1000);
  }

  digitalWrite(LED_BUILTIN, LOW);

  for(uint8_t i = 0; i < count; i++) {
    Serial.print(qtr.getCalMin(1));
    Serial.print(' ');
  }
  Serial.println();

  for(uint8_t i = 0; i < count; i++) {
    Serial.print(qtr.getCalMax(1));
    Serial.print(' ');
  }
  Serial.println();

  //qtr.setEvenEmitter(3);
  //qtr.setOddEmitter(2);

  //qtr.emitterTest();
}

void loop() {
  qtr.readSensors();
  qtr.readCalibrated();
  qtr.readBlackLine();
  for(uint8_t i = 0; i < count; i++) {
    Serial.print(qtr[i]);
    Serial.print(' ');
  }
  Serial.println();

}
