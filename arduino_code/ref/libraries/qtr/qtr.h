#ifndef QTR_H
#define QTR_H

/**
* @brief Define whether emitters stay on, stay off or flash on then off for readings
*/
enum class Emitter {
	On,
	Off,
	OnAndOff
};

/**
* @brief Define which combination of even and odd emitters are acted upon
*/
enum class Parity {
	Even,
	Odd,
	EvenAndOdd
};

/**
 * @brief Class to take sensor readings from polulu QTR digital sensor arrays
 */
class QTR {
	public:
		/**
		* @brief Constructor for the sensor class
		* @par Parameters
		* 	None.
		*/
		QTR();

		/**
		* @brief Set sensor timeout for a maximum reading
		* @param [in]	timeout	The timeout in microseconds 
		*/
		void setTimeout(uint32_t timeout);

		/**
		* @brief Set the pins that correlate to the sensor data lines
		* @param [in]	pins	An array of pin numbers
		* @param [in]	count	The number of pins	
		*/
		void setSensorPins(uint8_t* pins, uint8_t count);

		/**
		* @brief Calibrate the sensor `times` times with options for emitter
		* @param [in]	times	Number of times to read the sensor for calibration
		* @param [in]	e	The status of the emitters
		* @param [in]	p	The selection of even/odd emitters
		*/
		void calibrate(uint8_t times, Emitter e, Parity p);

		/**
		* @brief Reset the calibration status, deleting old calibration values
		* @par Parameters
		* 	None.
		*/
		void resetCalibrationStatus();

		/**
		* @brief Read sensor values as time to discharge sensing circuit
		* @par Parameters
		* 	None.
		*/
		void readSensors();

		/**
		* @brief Read sensor values and scale by calibration constants
		* @par Parameters
		* 	None.
		*/
		void readCalibrated();

		/**
		* @brief Read calibrated values and convert to black line reading
		* @retval	1	sensor over black line
		* @retval	0	sensor not over black line
		*/
		void readBlackLine();

		/**
		* @brief Deallocate all heap memory
		* @par Parameters
		* 	None.
		*/
		~QTR();

		/**
		* @brief Sum an array of uint8_t, used in `function`
		* @par Parameters
		* 	None.
		*/
		uint8_t sum(uint8_t* arr, uint8_t cnt);

		//getters and setters here as needed
		/**
		* @brief Returns the max value for the nth sensor in the array determined from calibration
		* @param	[in]	index	index into the array of sensors we are getting the calibrated min of
		*/
		uint32_t getCalMin(int index);

		/**
		* @brief Returns the min value for the nth sensor in the array determined from calibration
		* @param	[in]	index	index into the array of sensors we are getting the calibrated max of
		*/
		uint32_t getCalMax(int index);

		/**
		* @brief Returns the last read value of the nth sensor in the array
		* @param	[in]	index	index of the sensor we want to take reading of
		* @details
		* Returns a copy of the reading at `index`, based on the last read function that was called
		* If `readCalibrated()` or `readSensors()` was called last, readings can be any integers.
		* If `readBlackLine()` was last called, readings will be 1 or 0
		*/
		uint32_t operator[](int index) {return this->readings[index];};

		/**
		* @brief Sets the pin for controlling the even numbered emitters
		* @param	[in]	pin	number assigned to the pin connected to the even emitters
		*/
		void setEvenEmitter(uint8_t pin);

		/**
		* @brief Sets the pin for controlling the odd numbered emitters
		* @param	[in]	pin number assigned to the pin connected to the odd emitters
		*/
		void setOddEmitter(uint8_t pin);

		/**
		* @brief Set the pointer to the threshold from a calibrated reading for black line detection
		* @param	[in]	t the pointer to the variable the threshold is stored in
		*/
		void setThreshold(int* t);

		/**
		* @brief Get the threshold value for this sensor array
		* @par Parameters
		* 	None.
		*/
		int getThreshold();

		/**
		* @brief Get a read only view of the whole readings array
		* @par Parameters
		* 	None.
		*/
		const uint32_t* getReadings() const {return this->readings;};

	private:
		int* threshold = nullptr;

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

		/**
		* @brief Turn emitters on 
		* @param	[in]	p States whether even and/or odd emitters are turned on
		*/
		void emittersOn(Parity p); //flags for which emitters

		/**
		* @brief Turn emitters off
		* @param	[in]	p States whether even and/or odd emitters are turned off
		*/
		void emittersOff(Parity p);
		
		void switchEmittersOn(Emitter e, Parity p);
		void switchEmittersOff(Emitter e, Parity p);
};



#endif
