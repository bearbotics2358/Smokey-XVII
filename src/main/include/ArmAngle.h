#pragma once

#include "frc/WPILib.h"
#include <frc/CAN.h>

#define CAN_ID 0x0a080081

// CAN MASK is 1 bits in all 29 bit positions, except for the Device ID
#define CAN_MASK 0x01ffffc0
// all CAN followers will have the following result when ANDed with the CAN_MASK
#define CAN_REV 0x00080080



class ArmAngle
{
public:
	explicit ArmAngle(int x_deviceID);
	virtual ~ArmAngle();

	virtual void Update();
	virtual double GetAngle();


private:
    frc::CAN a_FeatherCAN;
    int deviceID;
    unsigned char rxBuf[8];

    double angle_f = 0.0;

    void decodeREVEncoderMsg();

};
