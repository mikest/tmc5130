/*
MIT License

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

#include "Estee_TMC5130.h"

//#define SERIAL_DEBUG

Estee_TMC5130_UART::Estee_TMC5130_UART(Stream& serial, uint8_t slaveAddress, uint32_t fclk)
: Estee_TMC5130(fclk), _serial(&serial), _slaveAddress(slaveAddress), _currentMode(STREAMING_MODE)
{

}

uint32_t Estee_TMC5130_UART::readRegister(uint8_t address, ReadStatus *status)
{
	uint32_t data = 0xFFFFFFFF;
	ReadStatus readStatus;

	switch (_currentMode)
	{
		case STREAMING_MODE:
			data = _readReg(address, &readStatus);
			break;

		case RELIABLE_MODE:
		{
			int retries = NB_RETRIES_READ;
			readStatus = NO_REPLY; //Worst case. If there is no reply for all retries this should be notified to the user.
			do {
				ReadStatus trialStatus;
				data = _readReg(address, &trialStatus);

				if (trialStatus == SUCCESS || (readStatus == NO_REPLY && trialStatus != NO_REPLY))
					readStatus = trialStatus;

				if (trialStatus == NO_REPLY)
					resetCommunication();

				retries--;
			} while (readStatus != SUCCESS && retries > 0);
			break;
		}
	}

	if (status != nullptr)
		*status = readStatus;

	_lastRegisterReadSuccess = (readStatus == SUCCESS);

	return data;
}

uint8_t Estee_TMC5130_UART::writeRegister(uint8_t address, uint32_t data, ReadStatus *status)
{
	switch (_currentMode)
	{
		case STREAMING_MODE:
			_writeReg(address, data);

			if (status != nullptr)
				*status = SUCCESS;
			break;

		case RELIABLE_MODE:
		{
			int retries = NB_RETRIES_WRITE;
			ReadStatus writeStatus = NO_REPLY;
			do {
				_writeReg(address, data);
				_writeAttemptsCounter++;

				ReadStatus readStatus;
				uint8_t counter = readRegister(TMC5130_Reg::IFCNT, &readStatus) & 0xFF;

				if (readStatus != NO_REPLY)
					writeStatus = readStatus;

				if (readStatus == SUCCESS)
				{
					if (counter != _transmissionCounter + 1)
						writeStatus = BAD_CRC;

					_transmissionCounter = counter;
				}

			retries--;
			} while (writeStatus != SUCCESS && retries > 0);

			if (status != nullptr)
				*status = writeStatus;

			if (writeStatus == SUCCESS)
				_writeSuccessfulCounter++;

			break;
		}
	}

	return 0;
}

void Estee_TMC5130_UART::resetCommunication()
{
	//FIXME should take into account the previous baud rate !
	// For now let's wait 1ms. The spec asks for ~75 bit times so this should be OK for baud rates > 75kbps
	delay(1);

#ifdef SERIAL_DEBUG
	Serial.println("Resetting communication.");
#endif

	//Flush input buffer.
	while (_serial->available())
		_serial->read();
}

void Estee_TMC5130_UART::setSlaveAddress(uint8_t slaveAddress, bool NAI)
{
	TMC5130_Reg::SLAVECONF_Register slaveConf = { 0 };
	slaveConf.senddelay = 4; // minimum if more than one slave is present.
	slaveConf.slaveaddr = constrain(NAI ? slaveAddress-1 : slaveAddress, 0, 253); //NB : if NAI is high SLAVE_ADDR is incremented.

	writeRegister(TMC5130_Reg::SLAVECONF, slaveConf.value);

	_slaveAddress = NAI ? slaveConf.slaveaddr+1 : slaveConf.slaveaddr;
}

void Estee_TMC5130_UART::setCommunicationMode(Estee_TMC5130_UART::CommunicationMode mode)
{
	_currentMode = mode;

	if (mode == RELIABLE_MODE)
	{
		//Initialize the 8-bit transmission counter.
		_transmissionCounter = readRegister(TMC5130_Reg::IFCNT) & 0xFF;
	}
}

void Estee_TMC5130_UART::resetCommunicationSuccessRate()
{
	_readAttemptsCounter = _readSuccessfulCounter = _writeAttemptsCounter = _writeSuccessfulCounter = 0;
}

float Estee_TMC5130_UART::getReadSuccessRate()
{
	if (_readAttemptsCounter == 0)
		return 0;

	return (float)_readSuccessfulCounter / (float)_readAttemptsCounter;
}

float Estee_TMC5130_UART::getWriteSuccessRate()
{
	if (_writeAttemptsCounter == 0)
		return 0;

	return (float)_writeSuccessfulCounter / (float)_writeAttemptsCounter;
}

uint32_t Estee_TMC5130_UART::_readReg(uint8_t address, ReadStatus *status)
{
	uint8_t outBuffer[4], inBuffer[8];

	outBuffer[0] = SYNC_BYTE;
	outBuffer[1] = _slaveAddress;
	outBuffer[2] = address;

	computeCrc(outBuffer, 4);

	beginTransmission();
	_serial->write(outBuffer, 4);
	endTransmission();

	_readAttemptsCounter++;

	if (_serial->readBytes(inBuffer, 8) != 8) //Stream.setTimeout has to be set to a decent value to avoid blocking
	{
		if (status != nullptr)
			*status = NO_REPLY;

	#ifdef SERIAL_DEBUG
		Serial.print("Read 0x");
		Serial.print(address, HEX);
		Serial.println(": No reply.");
	#endif

		return 0xFFFFFFFF;
	}

	if (inBuffer[0] != SYNC_BYTE || inBuffer[1] != MASTER_ADDRESS || inBuffer[2] != address)
	{
		if (status != nullptr)
			*status = INVALID_FORMAT;

	#ifdef SERIAL_DEBUG
		Serial.print("Read 0x");
		Serial.print(address, HEX);
		Serial.println(": Invalid answer format.");
		Serial.print("{");
		for (int i = 0; i < 8; i++)
		{
			Serial.print("0x");
			Serial.print(inBuffer[i], HEX);
			Serial.print(" ");
		}
		Serial.println("}");
	#endif

		return 0xFFFFFFFF;
	}

	uint8_t receivedCrc = inBuffer[7];
	computeCrc(inBuffer, 8);

	if (receivedCrc != inBuffer[7])
	{
		if (status != nullptr)
			*status = BAD_CRC;

	#ifdef SERIAL_DEBUG
		Serial.print("Read 0x");
		Serial.print(address, HEX);
		Serial.println(": Bad CRC.");
	#endif

		return 0xFFFFFFFF;
	}

	uint32_t data = 0;
	for (int i = 0; i < 4; i++)
		data += ((uint32_t)inBuffer[3+i] << ((3-i)*8));

	#ifdef SERIAL_DEBUG
	Serial.print("Read 0x");
	Serial.print(address, HEX);
	Serial.print(": 0x");
	Serial.println(data, HEX);
	#endif

	_readSuccessfulCounter++;

	if (status != nullptr)
		*status = SUCCESS;
	return data;
}

void Estee_TMC5130_UART::_writeReg(uint8_t address, uint32_t data)
{
#ifdef SERIAL_DEBUG
	Serial.print("Writing 0x");
	Serial.print(address, HEX);
	Serial.print(": 0x");
	Serial.println(data, HEX);
#endif

	uint8_t buffer[8];
	buffer[0] = SYNC_BYTE;
	buffer[1] = _slaveAddress;
	buffer[2] = address | WRITE_ACCESS;
	for (int i = 0; i < 4; i++)
		buffer[3+i] = (data & (0xFFul << ((3-i)*8))) >> ((3-i)*8);

	computeCrc(buffer, 8);

#if 0
	//Intentional interference to test the reliable mode : change the CRC
	if (random(256) < 64)
		buffer[7]++;
#endif

	beginTransmission();
	_serial->write(buffer, 8);
	endTransmission();
}

/* From Trinamic TMC5130A datasheet Rev. 1.14 / 2017-MAY-15 §5.2 */
void Estee_TMC5130_UART::computeCrc(uint8_t *datagram, uint8_t datagramLength)
{
	int i,j;
	uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
	uint8_t currentByte;

	*crc = 0;
	for (i = 0; i < (datagramLength-1); i++)
	{
		currentByte = datagram[i];
		for (j = 0; j < 8; j++)
		{
			if ((*crc >> 7) ^ (currentByte & 0x01))
				*crc = (*crc << 1) ^ 0x07;
			else
				*crc = (*crc << 1);

			currentByte = currentByte >> 1;
		} // for CRC bit
	} // for message byte
}
