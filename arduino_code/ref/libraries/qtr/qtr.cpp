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
	//undefined for reassignment so far
}

//try and find max and min values of reflectance based on trial period
//calibrate either with ambient light or no light
void QTR::calibrate(uint8_t times) {
	if(this->recalibrated == 0) {
		this->calmax = new uint32_t[this->count];
		this->calmin = new uint32_t[this->count];
	}

	uint32_t* vals = new uint32_t[this->count];
	uint32_t* curmin = new uint32_t[this->count];
	uint32_t* curmax = new uint32_t[this->count];
	
	for(int i = 0; i < this->count; ++i) {
		curmin[i] = this->maxtime;
		curmax[i] = 0;
	}

	for(int i = 0; i < times; ++i) {
		this->readSensors(vals, this->count);
		for(int j = 0; j < this->count; ++j) {
			if(vals[j] < curmin[j]) {
				curmin[j] = vals[j];
			}
			if(vals[j] > curmax[j]) {
				curmax[j] = vals[j];
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

void QTR::readSensors(uint32_t* values, uint8_t len) {
	uint8_t* readStatus = new uint8_t[len];

	for(int i = 0; i < len; ++i) {
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
				values[i] = timedelta;
				readStatus[i] = 1;
			}
		}
	} while(this->sum(readStatus, len) < len && timedelta < this->maxtime);
	//100 microsecond max time for now
	//interrupts();
	delete[] readStatus;
}

void QTR::readCalibrated(uint32_t* values, uint8_t len) {
	this->readSensors(values, len);
	for(int i = 0; i < len; ++i) {
		uint32_t invscale = this->calmax[i] - this->calmin[i];
		values[i] = values[i]*100/invscale; //set to autocalibrate between 0 and 1024
	}	
}

QTR::~QTR() {
	delete[] this->calmin;
	delete[] this->calmax;
	delete[] this->sensors; //free memory
}

uint32_t QTR::getCalMin(int index) {
	return this->calmin[index];
}

uint32_t QTR::getCalMax(int index) {
	return this->calmax[index];
}
