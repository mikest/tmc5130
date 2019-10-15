/*
MIT License

Copyright (c) 2016 Mike Estee
Copyright (c) 2017 Tom Magnier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef ESTEE_TMC5130_H
#define ESTEE_TMC5130_H

#include <Arduino.h>
#include <SPI.h>
#include <Estee_TMC5130_registers.h>

class Estee_TMC5130 {
public:
	static constexpr uint8_t IC_VERSION = 0x11;

	enum MotorDirection { NORMAL_MOTOR_DIRECTION =	0x00, INVERSE_MOTOR_DIRECTION = 0x1 };
	enum RampMode { POSITIONING_MODE, VELOCITY_MODE, HOLD_MODE };

	Estee_TMC5130(uint32_t fclk = DEFAULT_F_CLK);
	~Estee_TMC5130();

	// start/stop this module
	bool begin(uint8_t ihold, uint8_t irun, MotorDirection stepper_direction/*=NORMAL_MOTOR_DIRECTION*/);
	void end();

	virtual uint32_t readRegister(uint8_t address) = 0;	// addresses are from TMC5130.h
	virtual uint8_t  writeRegister(uint8_t address, uint32_t data) = 0;

	/* Check if the last register read was successful. This should be checked whenever
	 a register read is used to take a decision.
	 Reasons for failure can be : data bus disconnected, transmission error (bad CRC), etc
	 This is mostly useful in UART mode.
	 */
	bool isLastReadSuccessful();

	// internal clock measuring
	// NOTE: Driver MUST BE DISABLED DURING THIS CALL
	float updateFrequencyScaling();

	/* Ramp mode selection :
		- Positioning mode : autonomous move to XTARGET using all A, D and V parameters.
		- Velocity mode : follows VMAX and AMAX. Call setMaxSpeed() AFTER switching to velocity mode.
		- Hold mode : Keep current velocity until a stop event occurs.
	*/
	void setRampMode(RampMode mode);

	long getCurrentPosition(); // Return the current internal position (steps)
	long getEncoderPosition(); // Return the current position according to the encoder counter (steps)
	long getLatchedPosition(); // Return the position that was latched on the last ref switch / encoder event (steps)
	long getLatchedEncoderPosition(); // Return the encoder position that was latched on the last encoder event (steps)
	long getTargetPosition(); // Get the target position (steps)
	float getCurrentSpeed(); // Return the current speed (steps / second)


	void setCurrentPosition(long position, bool updateEncoderPos = false); // Set the current internal position (steps) and optionally update the encoder counter as well to keep them in sync.
	void setTargetPosition(long position); // Set the target position /!\ Set all other motion profile parameters before
	void setMaxSpeed(float speed); // Set the max speed VMAX (steps/second)
	void setRampSpeeds(float startSpeed, float stopSpeed, float transitionSpeed); // Set the ramp start speed VSTART, ramp stop speed VSTOP, acceleration transition speed V1 (steps / second). /!\ Set VSTOP >= VSTART, VSTOP >= 0.1
	void setAcceleration(float maxAccel); // Set the ramp acceleration / deceleration (steps / second^2)
	void setAccelerations(float maxAccel, float maxDecel, float startAccel, float finalDecel); // Set the ramp accelerations AMAX, DMAX, A1, D1 (steps / second^2) /!\ Do not set startAccel, finalDecel to 0 even if transitionSpeed = 0

	void stop(); // Stop the current motion according to the set ramp mode and motion parameters. The max speed and start speed are set to 0 but the target position stays unchanged.

	//TODO chopper config functions ?
	//TODO driver status functions

	/* Set the speeds (in steps/second) at which the internal functions and modes will be turned on or off.
	 * Below pwmThrs, "stealthChop" PWM mode is used.
	 * Between pwmThrs and highThrs, "spreadCycle" classic mode is used.
	 * Between coolThrs and highThrs, "spreadCycle" is used ; "coolStep" current reduction and "stallGuard" load measurement can be enabled.
	 * Above highThrs, "constant Toff" mode and fullstep mode can be enabled.
	 * See the TMC 5130 datasheet for details and optimization.
	 * Setting a speed to 0 will disable this threshold.
	 */
	void setModeChangeSpeeds(float pwmThrs, float coolThrs, float highThrs);

	/* Set the encoder constant to match the motor and encoder resolutions.
	 * This function will determine if the binary or decimal mode should be used
	 * and return false if no exact match could be found (for example for an encoder
	 * with a resolution of 360 and a motor with 200 steps per turn). In this case
	 * the best approximation in decimal mode will be used.
	 *
	 * Params :
	 * 		motorSteps : the number of steps per turn for the motor
	 * 		encResolution : the actual encoder resolution (pulses per turn)
	 * 		inverted : whether the encoder and motor rotations are inverted
	 *
	 * Return :
	 * 		true if an exact match was found, false otherwise
	 */
	bool setEncoderResolution(int motorSteps, int encResolution, bool inverted = false);

	/* Configure the encoder N event context.
	 * Params :
	 * 		sensitivity : set to one of ENCODER_N_NO_EDGE, ENCODER_N_RISING_EDGE, ENCODER_N_FALLING_EDGE, ENCODER_N_BOTH_EDGES
	 * 		nActiveHigh : choose N signal polarity (true for active high)
	 * 		ignorePol : if true, ignore A and B polarities to validate a N event
	 * 		aActiveHigh : choose A signal polarity (true for active high) to validate a N event
	 * 		bActiveHigh : choose B signal polarity (true for active high) to validate a N event
	 */
	void setEncoderIndexConfiguration(TMC5130_Reg::ENCMODE_sensitivity_Values sensitivity, bool nActiveHigh = true, bool ignorePol = true, bool aActiveHigh = false, bool bActiveHigh = false);

	/* Enable/disable encoder and position latching on each encoder N event (on each revolution)
	 * The difference between the 2 positions can then be compared regularly to check
	 * for an external step loss.
	 */
	void setEncoderLatching(bool enabled);

	//TODO end stops and stallguard config functions ?

protected:
	static constexpr uint8_t WRITE_ACCESS = 0x80;	//Register write access for spi / uart communication
	static constexpr uint32_t DEFAULT_F_CLK = 13200000; // Typical internal clock frequency in Hz.

	bool _lastRegisterReadSuccess = false;

private:
	uint32_t _fclk;
	RampMode _currentRampMode;
	static constexpr uint16_t _uStepCount = 256; // Number of microsteps per step

	// Following §14.1 Real world unit conversions
	// v[Hz] = v[5130A] * ( f CLK [Hz]/2 / 2^23 )
	float speedToHz(long speedInternal) { return ((float)speedInternal * (float)_fclk / (float)(1ul << 24) / (float)_uStepCount); }
	long speedFromHz(float speedHz) { return (long)(speedHz / ((float)_fclk / (float)(1ul << 24)) * (float)_uStepCount); }

	// Following §14.1 Real world unit conversions
	// a[Hz/s] = a[5130A] * f CLK [Hz]^2 / (512*256) / 2^24
	long accelFromHz(float accelHz) { return (long)(accelHz / ((float)_fclk * (float)_fclk / (512.0*256.0) / (float)(1ul << 24)) * (float)_uStepCount); }

	// See §12 Velocity based mode control
	long thrsSpeedToTstep(float thrsSpeed) { return thrsSpeed != 0.0 ? (long)constrain((float)_fclk / (thrsSpeed * 256.0), 0, 1048575) : 0; }
};


/* SPI interface : 
 * the TMC5130 SWSEL input has to be low (default state).
 */
class Estee_TMC5130_SPI : public Estee_TMC5130 {
public:
	Estee_TMC5130_SPI( uint8_t chipSelectPin,	// pin to use for the SPI bus SS line
		uint32_t fclk = DEFAULT_F_CLK,
		const SPISettings &spiSettings = SPISettings(1000000, MSBFIRST, SPI_MODE0), // spi bus settings to use
		SPIClass& spi = SPI ); // spi class to use

	uint32_t readRegister(uint8_t address);	// addresses are from TMC5130.h
	uint8_t  writeRegister(uint8_t address, uint32_t data);
	uint8_t  readStatus();

private:
	uint8_t _CS;
	SPISettings _spiSettings;
	SPIClass *_spi;

	void _beginTransaction();
	void _endTransaction();
};


/* Base UART interface :
 * the TMC5130 SWSEL input must be tied high.
 *
 * This class does not handle TX/RX switch on the half-duplex bus.
 * It should be used only if there is another mechanism to switch between
 * transmission and reception (e.g. on Teensy the Serial class can be configured
 * to control an external transceiver).
 *
 * Serial must be initialized externally. Serial.setTimeout() must be set to a
 * decent value to avoid blocking for too long if there is a RX error.
 */
class Estee_TMC5130_UART : public Estee_TMC5130 {
public:
	/* Read/write register return codes */
	enum ReadStatus {SUCCESS, NO_REPLY, INVALID_FORMAT, BAD_CRC};

	/* Serial communication modes. In reliable mode, register writes are checked and
	 * retried if necessary, and register reads are retried multiple times in case
	 * of failure. In streaming mode, none of these checks are performed and register
	 * read / writes are tried only once. Default is Streaming mode. */
	enum CommunicationMode {RELIABLE_MODE, STREAMING_MODE};


	Estee_TMC5130_UART(Stream& serial = Serial, // Serial port to use
		uint8_t slaveAddress = 0, // TMC5130 slave address (default 0 if NAI is low, 1 if NAI is high)
		uint32_t fclk = DEFAULT_F_CLK);

	uint32_t readRegister(uint8_t address, ReadStatus *status);	// addresses are from TMC5130.h. Pass an optional status pointer to detect failures.
	uint32_t readRegister(uint8_t address) { return readRegister(address, nullptr); }
	uint8_t  writeRegister(uint8_t address, uint32_t data, ReadStatus *status); // Pass an optional status pointer to detect failures.
	uint8_t writeRegister(uint8_t address, uint32_t data) { return writeRegister(address, data, nullptr); }

	void resetCommunication(); // Reset communication with TMC5130 : pause activity on the serial bus.

	void setSlaveAddress(uint8_t slaveAddress, bool NAI=true); // Set the slave address register. Take into account the TMC5130 NAI input (default to high). Range : 0 - 253 if NAI is low, 1 - 254 if NAI is high.
	uint8_t getSlaveAddress() { return _slaveAddress; }

	void setCommunicationMode(CommunicationMode mode);

	/* Register read / write statistics */
	void resetCommunicationSuccessRate();
	float getReadSuccessRate();
	float getWriteSuccessRate();
protected:
	static constexpr uint8_t NB_RETRIES_READ = 3;
	static constexpr uint8_t NB_RETRIES_WRITE = 3;

	Stream *_serial;
	uint8_t _slaveAddress;
	CommunicationMode _currentMode;
	uint8_t _transmissionCounter;

	/* Read / write fail statistics */
	uint32_t _readAttemptsCounter;
	uint32_t _readSuccessfulCounter;
	uint32_t _writeAttemptsCounter;
	uint32_t _writeSuccessfulCounter;


	virtual void beginTransmission() {}
	virtual void endTransmission() {}

	uint32_t _readReg(uint8_t address, ReadStatus *status);
	void _writeReg(uint8_t address, uint32_t data);

private:
	static constexpr uint8_t SYNC_BYTE = 0x05;
	static constexpr uint8_t MASTER_ADDRESS = 0xFF;

	void computeCrc(uint8_t *datagram, uint8_t datagramLength);
};


/* UART interface with external transceiver support :
 * the TMC5130 SWSEL input must be tied high.
 * See TMC5130 datasheet §5.4 figure 5.2 for wiring details
 *
 * This interface switches a digital pin to control an external transceiver to
 * free the bus when not transmitting.
 *
 * This is not optimized : the interface has to wait for the end of the
 * transmission.
 *
 * Serial must be initialized externally. Serial.setTimeout() must be set to a
 * decent value to avoid blocking for too long if there is a RX error.
 */
class Estee_TMC5130_UART_Transceiver : public Estee_TMC5130_UART {
public:
	Estee_TMC5130_UART_Transceiver(uint8_t txEnablePin = -1, // pin to enable transmission on the external transceiver
		Stream& serial = Serial, // Serial port to use
		uint8_t slaveAddress = 0, // TMC5130 slave address (default 0 if NAI is low, 1 if NAI is high)
		uint32_t fclk = DEFAULT_F_CLK)
	: Estee_TMC5130_UART(serial, slaveAddress, fclk), _txEn(txEnablePin)
	{
		pinMode(_txEn, OUTPUT);
	}

protected:
	void beginTransmission()
	{
		digitalWrite(_txEn, HIGH);
	}

	void endTransmission()
	{
		_serial->flush();
		digitalWrite(_txEn, LOW);
	}

private:
	uint8_t _txEn;
};


#endif // ESTEE_TMC5130_H
