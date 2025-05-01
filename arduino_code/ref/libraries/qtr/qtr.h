#ifndef QTR_H
#define QTR_H

class QTR {
	public:
		QTR();
		void setTimeout(uint32_t timeout);
		void setSensorPins(uint8_t* pins, uint8_t count);
		void calibrate(uint8_t times);
		void resetCalibrationStatus();
		void readSensors(uint32_t* values, uint8_t len);
		void readCalibrated(uint32_t* values, uint8_t len);
		~QTR();
		uint8_t sum(uint8_t* arr, uint8_t cnt);

		//getters and setters here as needed
		uint32_t getCalMin(int index);
		uint32_t getCalMax(int index);

	private:
		uint8_t count;
		uint32_t recalibrated;
		uint32_t maxtime;
		uint8_t* sensors = nullptr;
		uint32_t* calmin = nullptr;
		uint32_t* calmax = nullptr;
		
		void emittersOn(uint8_t even, uint8_t odd); //flags for which emitters
		void emittersOff(uint8_t even, uint8_t odd);
};



#endif
