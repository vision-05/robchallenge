#include <Arduino.h>
#include "qtr.h"

uint8_t QTR::sum(uint8_t* arr, uint8_t cnt) {
	uint8_t res = 0;
	for(uint8_t i = 0; i < cnt; ++i) {
		res += arr[i];
	}
	return res;
}

/**
* @details
* Sets the calibration status to 0
*/
QTR::QTR() {
	this->recalibrated = 0;
}

/**
* @details
* Sets the maxtime field to the `timeout` specified.
* If a sensor doesn't discharge after this length of time, 
* then this is the default sensor reading value */
void QTR::setTimeout(uint32_t timeout) {
	this->maxtime = timeout;
}

/**
* @details
* Sets the calibration status to 0.
* This allows for a "fresh" calibration, which ignores any previously calibrated maximum and minimum values
*/
void QTR::resetCalibrationStatus() {
	this->recalibrated = 0;
}

void QTR::setThreshold(int* t) {
	this->threshold = t;
}

int QTR::getThreshold() {
	return *(this->threshold);
}

/**
* @details
* Sets the count and pins of the sensor, with no bounds checking */
void QTR::setSensorPins(uint8_t* pins, uint8_t count) {
	this->count = count;
	if(this->sensors == nullptr) {
		this->sensors = new uint8_t[count];
		for(int i = 0; i < count; ++i) {
			this->sensors[i] = pins[i];
		}
	}

	this->readings = new uint32_t[this->count];
	//undefined for reassignment so far
}

/**
* @details
* Calibrate the sensor by finding the maximum and minimum of a set of values.
* Taking `times` sensor readings that hopefully have different results to provide a large range between max and min.
* Can specify whether the emitters will be on or off during the calibration, and until recalibration is set
* this is the only way to change which emitters are set to ensure continuity of readings
*/
void QTR::calibrate(uint8_t times, Emitter e, Parity p) {
	if(this->recalibrated == 0) {
		this->calmax = new uint32_t[this->count];
		this->calmin = new uint32_t[this->count];
		this->em = e;
		this->pa = p;
	}

	uint32_t* curmin = new uint32_t[this->count];
	uint32_t* curmax = new uint32_t[this->count];
	
	for(int i = 0; i < this->count; ++i) {
		curmin[i] = this->maxtime;
		curmax[i] = 0;
	}

	for(int i = 0; i < times; ++i) {
		this->readSensors();
		for(int j = 0; j < this->count; ++j) {
			if(this->readings[j] < curmin[j]) {
				curmin[j] = this->readings[j];
			}
			if(this->readings[j] > curmax[j]) {
				curmax[j] = this->readings[j];
			}
		}
	}

	for(int i = 0; i < this->count; ++i) {
		if(curmin[i] > this->calmax[i]) {
			this->calmax[i] = curmin[i];
		}
		if(curmax[i] < this->calmin[i]) {
			this->calmin[i] = curmax[i];
		}
	}



	uint32_t maxmax = 0;
	uint32_t minmin = this->maxtime;
	for(int i = 0; i < this->count; ++i) {
		if(this->calmax[i] > maxmax) {
			maxmax = this->calmax[i];
		}
		if(this->calmin[i] < minmin) {
			minmin = this->calmin[i];
		}
	}

	if(maxmax > this->maxtime) {
		maxmax = this->maxtime;
	}

	for(int i = 0; i < this->count; ++i) {
		this->calmax[i] = maxmax;
		this->calmin[i] = minmin;
	}

	delete[] curmin;
	delete[] curmax;

	this->recalibrated = 1;
	
}

/**
* @details
* Read the sensors by setting the circuits to `HIGH`, waiting 10 microseconds and then timing how long it takes to discharge to low.
* Does not use `noInterrupts()` as the Arduino GIGA crashes.
* Provides a raw value that needs calibration to be useful */
void QTR::readSensors() {
	this->switchEmittersOn(this->em, this->pa);
	uint8_t* readStatus = new uint8_t[this->count];

	for(int i = 0; i < this->count; ++i) {
		readStatus[i] = 0;
	}

	//for all sensors drive high
	//wait 10microseconds
	//change to input
	//wait until low reading
	for(int i = 0; i < this->count; ++i) {
		pinMode(this->sensors[i], OUTPUT);
		digitalWrite(this->sensors[i], HIGH);
	}
	delayMicroseconds(10);

	//noInterrupts();
	uint32_t timestart = micros();
	uint32_t timedelta = 0;

	for(int i = 0; i < this->count; ++i) {
		pinMode(this->sensors[i], INPUT);
	}

	do {
		for(int i = 0; i < this->count; ++i) {
			if(readStatus[i] == 1) {
				continue; //if already read, skip
			}
			int val = digitalRead(this->sensors[i]);
			timedelta = micros() - timestart;
			if(val == LOW) {
				this->readings[i] = timedelta;
				readStatus[i] = 1;
			}
		}
	} while(this->sum(readStatus, this->count) < this->count && timedelta < this->maxtime);
	//100 microsecond max time for now
	//interrupts();
	delete[] readStatus;
	this->switchEmittersOff(this->em, this->pa);
}

void QTR::readCalibrated() {
	float temp = 0;
	this->readSensors();
	for(int i = 0; i < this->count; ++i) {
		uint32_t invscale = this->calmax[i] - this->calmin[i];
		Serial.print("Inv: ");
		Serial.print(invscale);
		Serial.print(" ");
		temp = this->readings[i]*100/invscale; //set to autocalibrate between 0 and 1024
		this->readings[i] = (uint32_t)temp;
	}
	Serial.println(this->readings[0]);

	//average sensor readings to filter noise

	for(int i = 1; i < this->count -1; ++i) {
		this->readings[i] = (this->readings[i-1] + this->readings[i] + this->readings[i+1])*0.333;
	}
}

void QTR::readBlackLine() {
	this->readCalibrated();
	for(int i = 0; i < this->count; ++i) {
		if(this->readings[i] > *(this->threshold)) {
			this->readings[i] = 1;
		}
		else {
			this->readings[i] = 0;
		}
	}
}

QTR::~QTR() {
	delete[] this->calmin;
	delete[] this->calmax;
	delete[] this->sensors;
	delete[] this->readings;
}

uint32_t QTR::getCalMin(int index) {
	return this->calmin[index];
}

uint32_t QTR::getCalMax(int index) {
	return this->calmax[index];
}


///Set the pin number of the even emitter control pin
void QTR::setEvenEmitter(uint8_t pin) {
	this->evenEmitter = pin;
	pinMode(pin, OUTPUT);
}

///Set the pin number of the odd emitter control pin
void QTR::setOddEmitter(uint8_t pin) {
	this->oddEmitter = pin;
	pinMode(pin, OUTPUT);
}

void QTR::emittersOn(Parity p) {
	if(p == Parity::Even || p == Parity::EvenAndOdd) {
		digitalWrite(this->evenEmitter, HIGH);
	}
	if(p == Parity::Odd || p == Parity::EvenAndOdd) {
		digitalWrite(this->oddEmitter, HIGH);
	}
}

void QTR::emittersOff(Parity p) {
	if(p == Parity::Even || p == Parity::EvenAndOdd) {
		digitalWrite(this->evenEmitter, LOW);
	}
	if(p == Parity::Odd || p == Parity::EvenAndOdd) {
		digitalWrite(this->oddEmitter, LOW);
	}
}

void QTR::switchEmittersOn(Emitter e, Parity p) {
	switch(e) {
		case Emitter::OnAndOff:
		case Emitter::On:
			this->emittersOn(p);
			break;
		case Emitter::Off:
			this->emittersOff(p);
			break;
	}
}

void QTR::switchEmittersOff(Emitter e, Parity p) {
	switch(e) {
		case Emitter::Off:
		case Emitter::OnAndOff:
			this->emittersOff(p);
			break;
		case Emitter::On:
			this->emittersOn(p);
	}
}
