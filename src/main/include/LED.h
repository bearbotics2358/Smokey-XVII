/* LED.cpp - Control LED lights represnting desired game piece

 */

#ifndef H_LED
#define H_LED

#include "Protocol.h" //yay i can use CONE and CUBE
#include <frc/SerialPort.h>

#define BUFF_SIZE 256

// enum LED_STAGE_enum {
//   WHITE = 0,
//   LED_IDLE,
//   NO_COMMS,
//   NOTE_COLLECTED
// };

class LED
{
public:

	LED();
	virtual ~LED() = default;

	void Reset();
	void Update();

	void SetWhite();
	void SetMSGIdle();
	void SetNoComms();
	void SetNoteOnBoard();
	void SetAngleToNote(float angle);
	void SetShooterReady();

	void ProcessReport();

private:

	// Constant for how often to send the serial command for the LEDs. These are sent at a slower
	// rate than RobotPeriodic (20ms) to avoid spamming buffers since the LEDs do not need to be
	// updated that frequently.
	static constexpr double kUpdatePeriodSeconds = 1.0;
	double m_lastUpdateTimeSeconds = 0.0;

	std::unique_ptr<frc::SerialPort> m_pserial;
	char rx_buff[BUFF_SIZE];
	int rx_index = 0;
	float valAngle  = 0;
	RIO_msgs_enum LED_prevCommand = RIO_msgs_enum::MSG_IDLE;
	RIO_msgs_enum LED_currentCommand = RIO_msgs_enum::MSG_IDLE;
	bool SendWhiteMSG();
	bool SendIdleMSG();
	bool SendNoCommsMSG();
	bool SendNoteOnBoardMSG();
	bool SendAngleToNoteMSG(float angle);
	bool SendShooterReadyMSG();
} ;

#endif
