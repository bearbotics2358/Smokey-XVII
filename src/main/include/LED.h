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

	void Init();
	void Update();

	void SetWhite();
	void SetMSGIdle();
	void SetNoComms();
	void SetNoteOnBoard();
	void SetAngleToNote(float angle);
	void SetShooterReady();

	void ProcessReport();
	//enum LED_STAGE_enum GetTargetRangeIndicator();
	//void SetTargetType(LED_STAGE_enum target_type_param);
	//LED_STAGE_enum GetTargetType();

private:
	
	frc::SerialPort m_serial;
	char rx_buff[BUFF_SIZE];
	int rx_index = 0;
	//LED_STAGE_enum target_type = LED_STAGE_enum::WHITE;
} ;

#endif
