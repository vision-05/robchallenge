#include <qtr.h>

const int count = 9;
uint16_t values[count];
QTR qtr;

void setup() {
  Serial.begin(9600);

  Serial.println("Hello");
  qtr.setTimeout(1000);
  uint8_t sensors[count] = {52,50,48,44,42,40,38,36,34};
  qtr.setSensorPins(sensors,count);
  Serial.println("Set pins");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for(uint16_t i = 0; i < 20; i++) {
    qtr.calibrate(10);
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
}

void loop() {
  uint32_t* values = new uint32_t[count];
  qtr.readSensors(values,count);
  qtr.readCalibrated(values,count);
  for(uint8_t i = 0; i < count; i++) {
    Serial.print(values[i]);
    Serial.print(' ');
  }
  Serial.println();
  delete[] values;

}
