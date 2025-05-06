#include <Arduino.h>
#include "qtr.h"

uint8_t QTR::sum(uint8_t* arr, uint8_t cnt) {
	uint8_t res = 0;
	for(uint8_t i = 0; i < cnt; ++i) {
		res += arr[i];
	}
	return res;
}

QTR::QTR() {
	this->recalibrated = 0;
}

void QTR::setTimeout(uint32_t timeout) {
	this->maxtime = timeout;
}

void QTR::resetCalibrationStatus() {
	this->recalibrated = 0;
}

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

//try and find max and min values of reflectance based on trial period
//calibrate either with ambient light or no light
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

	delete[] curmin;
	delete[] curmax;

	this->recalibrated = 1;
	
}

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
	this->readSensors();
	for(int i = 0; i < this->count; ++i) {
		uint32_t invscale = this->calmax[i] - this->calmin[i];
		this->readings[i] = this->readings[i]*100/invscale; //set to autocalibrate between 0 and 1024
	}	
}

void QTR::readBlackLine() {
	this->readCalibrated();
	for(int i = 0; i < this->count; ++i) {
		if(this->readings[i] > 250) {
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
