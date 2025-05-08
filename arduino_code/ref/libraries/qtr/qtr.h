#ifndef QTR_H
#define QTR_H

enum class Emitter {
	On,
	Off,
	OnAndOff
};

///Specify which emitters are on/off
enum class Parity {
	Even,
	Odd,
	EvenAndOdd
};

class QTR {
	public:
		QTR();
		void setTimeout(uint32_t timeout);
		void setSensorPins(uint8_t* pins, uint8_t count);
		void calibrate(uint8_t times, Emitter e, Parity p);
		void resetCalibrationStatus();
		void readSensors();
		void readCalibrated();
		void readBlackLine();
		~QTR();
		uint8_t sum(uint8_t* arr, uint8_t cnt);

		//getters and setters here as needed
		uint32_t getCalMin(int index);
		uint32_t getCalMax(int index);

		uint32_t operator[](int index) {return this->readings[index];};

		void setEvenEmitter(uint8_t pin);
		void setOddEmitter(uint8_t pin);
		

	private:
		uint8_t count;
		uint32_t recalibrated;
		uint32_t maxtime;

		uint8_t oddEmitter;
		uint8_t evenEmitter;

		Emitter em = Emitter::Off;
		Parity pa = Parity::EvenAndOdd;

		uint8_t* sensors = nullptr;
		uint32_t* calmin = nullptr;
		uint32_t* calmax = nullptr;

		uint32_t* readings = nullptr;
		
		void emittersOn(Parity p); //flags for which emitters
		void emittersOff(Parity p);
		
		void switchEmittersOn(Emitter e, Parity p);
		void switchEmittersOff(Emitter e, Parity p);
};



#endif
