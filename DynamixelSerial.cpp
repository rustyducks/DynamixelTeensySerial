/*
 * DynamixelSerial.cpp
 *
 *  Created on: 24 janv. 2019
 *      Author: guilhem
 */

#include <Arduino.h>
#include "DynamixelSerial.h"

DynamixelSerial::DynamixelSerial(unsigned int serialNumber, HardwareSerial& serial): _serialNumber(serialNumber), _serial(serial){

}

void DynamixelSerial::begin(long int baud){
	_serial.begin(baud, SERIAL_8O1);
	enableOpenDrain(true);
}

void DynamixelSerial::end(){
	_serial.end();
}

int DynamixelSerial::reset(unsigned char ID){
	sendInstruction(ID, I_Reset, nullptr, 0);
	return readError();
}

int DynamixelSerial::ping(unsigned char ID){
	sendInstruction(ID, I_Ping, nullptr, 0);
	return readError();
}

int DynamixelSerial::setID(unsigned char ID, unsigned char newID){
	byte data[2];
	data[0] = R_ServoID;
	data[1] = newID;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setBD(unsigned char ID, long baud){
	unsigned char baud_rate = (2000000/baud) - 1;
	byte data[2];
	data[0] = R_BaudRate;
	data[1] = baud_rate;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::move(unsigned char ID, int position){
	byte data[3];
	data[0] = R_GoalPosition;
	data[1] = position & 0xFF;
	data[2] = (position >> 8) & 0xFF;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::moveSpeed(unsigned char ID, int position, int speed){
	byte data[5];
	data[0] = R_GoalPosition;
	data[1] = position & 0xFF;
	data[2] = (position >> 8) & 0xFF;
	data[3] = speed & 0xFF;
	data[4] = (speed >> 8) & 0xFF;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setEndless(unsigned char ID, bool status){
	byte data[3];
	if (status){
		data[0] = R_CCW_AngleLimit;
		data[1] = 0;
		data[2] = 0;
	}else{
		turn(ID, RotationDirection::Clockwise, 0);
		data[0] = R_CCW_AngleLimit;
		data[1] = 0xFF;
		data[2] = 0x03;
	}
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::turn(unsigned char ID, RotationDirection direction, int speed){
	byte data[3];
	data[0] = R_MovingSpeed;
	data[1] = speed & 0xFF;
	data[2] = (speed >> 8) & 0xFF;
	data[2] |= direction << 3;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::moveRW(unsigned char ID, int position){
	byte data[3];
	data[0] = R_GoalPosition;
	data[1] = position & 0xFF;
	data[2] = (position >> 8) & 0xFF;
	sendInstruction(ID, I_RegWrite, data, sizeof(data));
	return readError();
}

int DynamixelSerial::moveSpeedRW(unsigned char ID, int position, int speed){
	byte data[5];
	data[0] = R_GoalPosition;
	data[1] = position & 0xFF;
	data[2] = (position >> 8) & 0xFF;
	data[3] = speed & 0xFF;
	data[4] = (speed >> 8) & 0xFF;
	sendInstruction(ID, I_RegWrite, data, sizeof(data));
	return readError();
}

void DynamixelSerial::action(){
	sendInstruction(0xFE, I_Action, nullptr, 0);
}

int DynamixelSerial::torqueStatus(unsigned char ID, bool status){
	byte data[2];
	data[0] = R_TorqueEnable;
	data[1] = status ? 1 : 0;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::ledStatus(unsigned char ID, bool status){
	byte data[2];
	data[0] = R_LED;
	data[1] = status ? 1 : 0;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::readTemperature(unsigned char ID){
	byte data[2];
	data[0] = R_PresentTemperature;
	data[1] = 1;  // Temperature is only 1 byte
	sendInstruction(ID, I_ReadData, data, sizeof(data));
	return readResponse(1);
}

int DynamixelSerial::readPosition(unsigned char ID){
	byte data[2];
	data[0] = R_PresentPosition;
	data[1] = 2;  // Position is 2 bytes long
	sendInstruction(ID, I_ReadData, data, sizeof(data));
	return readResponse(2);
}

int DynamixelSerial::readVoltage(unsigned char ID){
	byte data[2];
	data[0] = R_PresentVoltage;
	data[1] = 1;  // Voltage is 1 byte long
	sendInstruction(ID, I_ReadData, data, sizeof(data));
	return readResponse(1);
}

int DynamixelSerial::setTempLimit(unsigned char ID, unsigned char temperature){
	byte data[2];
	data[0] = R_HighestLimitTemperature;
	data[1] = temperature;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage){
	byte data[3];
	data[0] = R_LowestLimitVoltage;
	data[1] = DVoltage;
	data[2] = UVoltage;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit){
	byte data[6];
	data[0] = R_CW_AngleLimit;
	data[1] = CWLimit & 0xFF;
	data[2] = (CWLimit >> 8) & 0xFF;
	data[3] = R_CCW_AngleLimit;
	data[4] = CCWLimit & 0xFF;
	data[5] = (CCWLimit >> 8) & 0xFF;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setMaxTorque(unsigned char ID, int maxTorque){
	byte data[3];
	data[0] = R_MaxTorque;
	data[1] = maxTorque & 0xFF;
	data[2] = (maxTorque >> 8) & 0xFF;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setSRL(unsigned char ID, unsigned char SRL){
	byte data[2];
	data[0] = R_StatusReturnLevel;
	data[1] = SRL;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setRDT(unsigned char ID, unsigned char RDT){
	byte data[2];
	data[0] = R_ReturnDelayTime;
	data[1] = RDT;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setLEDAlarm(unsigned char ID, unsigned char LEDAlarm){
	byte data[2];
	data[0] = R_AlarmLED;
	data[1] = LEDAlarm;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setShutdownAlarm(unsigned char ID, unsigned char SALARM){
	byte data[2];
	data[0] = R_AlarmShutdown;
	data[1] = SALARM;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin){
	byte data[4];
	data[0] = R_CW_ComplianceMargin;
	data[1] = CWCMargin;
	data[2] = R_CCW_ComplianceMargin;
	data[3] = CCWCMargin;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope){
	byte data[4];
	data[0] = R_CW_ComplianceSlope;
	data[1] = CWCSlope;
	data[2] = R_CCW_ComplianceSlope;
	data[3] = CCWCSlope;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::setPunch(unsigned char ID, int punch){
	byte data[3];
	data[0] = R_Punch;
	data[1] = punch & 0xFF;
	data[2] = (punch >> 8) & 0xFF;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::moving(unsigned char ID){
	byte data[2];
	data[0] = R_Moving;
	data[1] = 1;  // moving is 1 byte long
	sendInstruction(ID, I_ReadData, data, sizeof(data));
	return readResponse(1);
}

int DynamixelSerial::lockRegister(unsigned char ID){
	byte data[2];
	data[0] = R_Lock;
	data[1] = 1;
	sendInstruction(ID, I_WriteData, data, sizeof(data));
	return readError();
}

int DynamixelSerial::RWStatus(unsigned char ID){
	byte data[2];
	data[0] = R_RegisteredInstruction;
	data[1] = 1;  // Registered Instruction is 1 byte long
	sendInstruction(ID, I_ReadData, data, sizeof(data));
	return readResponse(1);
}

int DynamixelSerial::readSpeed(unsigned char ID){
	byte data[2];
	data[0] = R_PresentSpeed;
	data[1] = 2;  // Speed is 2 bytes long
	sendInstruction(ID, I_ReadData, data, sizeof(data));
	return readResponse(2);
}

int DynamixelSerial::readLoad(unsigned char ID){
	byte data[2];
	data[0] = R_PresentLoad;
	data[1] = 2;  // Load is 2 bytes long
	sendInstruction(ID, I_ReadData, data, sizeof(data));
	return readResponse(2);
}

void DynamixelSerial::enableOpenDrain(bool enable){
	switch(_serialNumber){
	case 1:
		if (enable){
			CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_MUX(3); // set Open Drain Enabled
		}else{
			CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); // set Slew Rate Enabled
		}
		break;
	case 2:
		if (enable){
			CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_MUX(3); // set Open Drain Enabled
		}else{
			CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); // set Slew Rate Enabled
		}
		break;
	case 3:
		if (enable){
			CORE_PIN8_CONFIG = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_MUX(3); // set Open Drain Enabled
		}else{
			CORE_PIN8_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); // set Slew Rate Enabled
		}
		break;
	case 4:
		if (enable){
			CORE_PIN32_CONFIG = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_MUX(3); // set Open Drain Enabled
		}else{
			CORE_PIN32_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); // set Slew Rate Enabled
		}
		break;
	case 5:
		if (enable){
			CORE_PIN33_CONFIG = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_MUX(3); // set Open Drain Enabled
		}else{
			CORE_PIN33_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); // set Slew Rate Enabled
		}
		break;
	default:
		_serial = Serial1;
		break;
	}
}

bool DynamixelSerial::sendInstruction(byte ID, byte instruction, byte* params, int paramsLength){
	byte txPacket[100];
	// Send instruction
	int s=0;
	txPacket[s++] = 0xFF;
	txPacket[s++] = 0xFF; // Start bytes
	txPacket[s++] = ID;
	txPacket[s++] = paramsLength + 2; // Total size = instruction + params + checksum
	txPacket[s++] = instruction;
	for (int i=0; i < paramsLength; i++){
		txPacket[s++] = params[i];
	}
	byte checksum = 0;
	for (int i = 2; i<s; i++){
		checksum += txPacket[i];
	}
	txPacket[s++] = ~checksum;

	_serial.clear();
	enableOpenDrain(false);
	_serial.write(txPacket, s);
	_serial.flush();
	enableOpenDrain(true);

	// Read echo
	uint32_t start = millis();
	int r=0;
	while (r < s){
		if (millis() - start > 500){
			return false;
		}
		if (!_serial.available()){
			continue;
		}
		if (_serial.read() != txPacket[r++]){
			return false;
		}
	}
	return true;
}

int DynamixelSerial::readError(){
		uint32_t start = millis();
		while((_serial.available() < 5) & (millis() - start < 20)){  // Wait for Data
			delayMicroseconds(1000);
		}

		while (_serial.available() > 0){
			byte incoming = _serial.read();
			if ( (incoming == 255) & (_serial.peek() == 255) ){
				_serial.read();                                    // Start Bytes
				_serial.read();                                    // Ax-12 ID
				_serial.read();                                    // Length
				byte status = _serial.read();                       // Error
				return (status);
			}
		}
		return (-1);											 // No Ax Response
}

int DynamixelSerial::readResponse(int answerLength){
	uint32_t start = millis();
	int totalLength = 5 + answerLength;
	int answer = -1;
	byte errorByte, incoming;
	while((_serial.available() < totalLength) & (millis() - start < 20)){
		delayMicroseconds(1000);
	}

	while(_serial.available() > 0){
		incoming = _serial.read();
		if ((incoming == 255) & (_serial.peek() == 255)){
			_serial.read();
			_serial.read();
			_serial.read();
			if ((errorByte = _serial.read()) != 0){
				return errorByte * -1;
			}
			answer = 0;
			for (int i = 0; i < answerLength; i++){
				answer |= _serial.read() << (8 * i);
			}
		}
	}
	return answer;
}


