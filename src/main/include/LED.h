/* LED.cpp - Control LED lights represnting desired game piece

 */

#ifndef H_LED
#define H_LED

#include "TOF_protocol.h" //yay i can use CONE and CUBE
#include <frc/SerialPort.h>

#define BUFF_SIZE 256

class LED
{
public:

	LED();
	virtual ~LED() = default;

	void Init();
	void Update();

	void ProcessReport();
	enum LED_STAGE_enum GetTargetRangeIndicator();
	void SetTargetType(LED_STAGE_enum target_type_param);
	LED_STAGE_enum GetTargetType();

private:
#ifdef COMP_BOT  // Not available on the practice bot
	frc::SerialPort m_serial;
#endif
	char rx_buff[BUFF_SIZE];
	int rx_index = 0;
	LED_STAGE_enum target_type = LED_STAGE_enum::WHITE;
} ;

#endif
