/* LED.cpp - Control LED lights represnting desired game piece

 */

#include <stdio.h> // printf
#include <stdlib.h> // atoi
#include <Prefs.h>
#include "LED.h"
#include "misc.h"

LED::LED()
#ifdef COMP_BOT  // Not available on the practice bot
// :
	// m_serial(BAUD_RATE_ARDUINO, USB_PORT_ARDUINO, DATA_BITS_ARDUINO, PARITY_ARDUINO, STOP_BITS_ARDUINO)
#endif
	// comments from 2018:
	// USB1 is the onboard port closest to the center of the rio
	// I dunno which one USB2 is yet. (Rio docs aren't very helpful)

{
	Reset();
}

void LED::Reset()
{
	LED_currentCommand = RIO_msgs_enum::MSG_IDLE;
	LED_prevCommand = RIO_msgs_enum::WHITE;


	// If m_pserial has already been created by the Reset function, calling make_unique again
	// will delete the previous instance of m_pserial and create a new one without leaking memory.
	m_pserial = std::make_unique<frc::SerialPort>(BAUD_RATE_ARDUINO,
												  USB_PORT_ARDUINO,
												  DATA_BITS_ARDUINO,
												  PARITY_ARDUINO,
												  STOP_BITS_ARDUINO);
	m_pserial->DisableTermination();
	m_pserial->SetFlowControl(frc::SerialPort::kFlowControl_None);
	m_pserial->SetReadBufferSize(0);
	m_pserial->SetWriteBufferMode(frc::SerialPort::kFlushOnAccess);

	rx_index = 0;
	for (int i = 0; i < BUFF_SIZE; i++) {
		rx_buff[i] = 0;
	}
}

void LED::Update()
{
#ifdef COMP_BOT  // Not available on the practice bot
	// call this routine periodically to check for any readings and store
	// into result registers

	// get report if there is one
	// every time called, and every time through loop, get repotr chars if available
	// and add to rx buffer
	// when '\r' (or '\t') found, process reading

	// printf("LED: in Update, attempting to receive\n");

	while (m_pserial->GetBytesReceived() > 0) {
		m_pserial->Read(&rx_buff[rx_index], 1);

		//printf("LED LED LED LED: %c\n", rx_buff[rx_index]);


		if((rx_buff[rx_index] == '\r')
			 || (rx_buff[rx_index] == '\n')) {

			// process report
			if(rx_index == 0) {
				// no report
				continue;
			}

			// terminate the report string
			rx_buff[rx_index] = 0;

			// print the report:
			printf("RX: %s\n", rx_buff);
			m_pserial->Flush();

			ProcessReport();

			// printf("LED report: rx_buff\n");

			// reset for next report
			rx_index = 0;
		} else {
			// have not received end of report yet
			if(rx_index < BUFF_SIZE - 1) {
				rx_index++;
			}
		}
	}

	// Send the command to update the LEDs if the command is different OR if our timeout has been exceeded.
	// The timeout will ensure that we periodically update the LEDs to prevent a serial connection
	// problem from leaving them stuck in the wrong state.
	if ((LED_currentCommand != LED_prevCommand) ||
		(misc::gettime_d() > m_lastUpdateTimeSeconds + kUpdatePeriodSeconds)) {

		m_lastUpdateTimeSeconds = misc::gettime_d();
		LED_prevCommand = LED_currentCommand;

		bool serial_write_success = false;
		switch (LED_currentCommand)
		{
			case RIO_msgs_enum::WHITE:
				serial_write_success = SendWhiteMSG();
				break;

			case RIO_msgs_enum::MSG_IDLE:
				serial_write_success = SendIdleMSG();
				break;

			case RIO_msgs_enum::NO_COMMS:
				serial_write_success = SendNoCommsMSG();
				break;

			case RIO_msgs_enum::NOTE_ON_BOARD:
				serial_write_success = SendNoteOnBoardMSG();
				break;

			case RIO_msgs_enum::ANGLE_TO_NOTE:
				serial_write_success = SendAngleToNoteMSG(valAngle);
				break;

			case RIO_msgs_enum::SHOOTER_READY:
				serial_write_success = SendShooterReadyMSG();
				break;

			default:
				break;
		}

		// If a serial write failed, then attempt to re-establish the connection. This may happen
		// if the USB cable comes unplugged temporarily.
		if (false == serial_write_success) {
			Reset();
		}
	}
#endif
}

// instead of atoi(), UltrasonicSerial used strtol(&readBuffer[1], (char **)NULL, 10);


void LED::ProcessReport()
{
	// parse report
	// no action needed, no report expected


}

void LED::SetWhite() {
	LED_currentCommand = RIO_msgs_enum::WHITE;
}


bool LED::SendWhiteMSG() {
	char cmd[10];
	strncpy(cmd, "(0,0)\r\n", 8);
	int num_bytes_written = m_pserial->Write(cmd, strlen(cmd));
	m_pserial->Flush();

	// If nothing was written to the serial port, return false so we can attempt to reconnect
	return (num_bytes_written > 0);
}


void LED::SetMSGIdle(){

	LED_currentCommand = RIO_msgs_enum::MSG_IDLE;
}


bool LED::SendIdleMSG() {
	char cmd[10];
	strncpy(cmd, "1,0\r\n", 8);
	int num_bytes_written = m_pserial->Write(cmd, strlen(cmd));
	m_pserial->Flush();

	// If nothing was written to the serial port, return false so we can attempt to reconnect
	return (num_bytes_written > 0);
}

void LED::SetNoComms() {
	LED_currentCommand = RIO_msgs_enum::NO_COMMS;
}


bool LED::SendNoCommsMSG() {

	printf("in SetNoComms\n");
	char cmd[10];
	strncpy(cmd, "2,0\r\n", 8);
	printf("about to Write: %s\n", cmd);
	int num_bytes_written = m_pserial->Write(cmd, strlen(cmd));
	printf("written: %d characters\n", num_bytes_written);
	printf("about to Flush\n");
	m_pserial->Flush();
	printf("flushed\n\n");

	// If nothing was written to the serial port, return false so we can attempt to reconnect
	return (num_bytes_written > 0);
}


void LED::SetNoteOnBoard(){
	LED_currentCommand = RIO_msgs_enum::NOTE_ON_BOARD;
}



bool LED::SendNoteOnBoardMSG() {
	char cmd[10];
	strncpy(cmd, "3,0\r\n", 8);
	int num_bytes_written = m_pserial->Write(cmd, strlen(cmd));
	m_pserial->Flush();

	return (num_bytes_written > 0);
}


void LED::SetAngleToNote(float inputAngle){
	valAngle = inputAngle;
	LED_currentCommand = RIO_msgs_enum::ANGLE_TO_NOTE;
}



bool LED::SendAngleToNoteMSG(float angle) {
	char cmd[10];
	sprintf(cmd, "4,1,%1.2f\r\n", angle);
	printf("%s\n", cmd);
	int num_bytes_written = m_pserial->Write(cmd, strlen(cmd));
	m_pserial->Flush();

	// If nothing was written to the serial port, return false so we can attempt to reconnect
	return (num_bytes_written > 0);
}

void LED::SetShooterReady() {
	LED_currentCommand = RIO_msgs_enum::SHOOTER_READY;
}

bool LED::SendShooterReadyMSG() {
	char cmd[10];
	strncpy(cmd, "5,0\r\n", 8);
	int num_bytes_written = m_pserial->Write(cmd, strlen(cmd));
	m_pserial->Flush();

	// If nothing was written to the serial port, return false so we can attempt to reconnect
	return (num_bytes_written > 0);
}

