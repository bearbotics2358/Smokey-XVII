#include "ArmAngle.h"
#include "Prefs.h"

ArmAngle::ArmAngle(int x_deviceID):
    deviceID(x_deviceID),
    a_FeatherCAN(x_deviceID)
{
}

ArmAngle::~ArmAngle()
{
}

void ArmAngle::Update()
{
	frc::CANData data1;
	int i;

	bool ret = a_FeatherCAN.ReadPacketNew(4, &data1);

	if(ret) {
		for(i = 0; i < data1.length; i++) {
			rxBuf[i] = data1.data[i];
		}

		decodeArmAngleMsg();
	}
}

double ArmAngle::GetAngle()
{
	return m_rawAngleDeg + ARM_ANGLE_OFFSET_DEGREES;
}

void ArmAngle::decodeArmAngleMsg()
{
	int anglex10 = 0;

	anglex10 = (rxBuf[0] << 8) | rxBuf[1];
	m_rawAngleDeg = (anglex10 * 1.0) / 10.0;
}
