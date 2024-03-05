#include "ArmAngle.h"
#include <HAL/HAL.h>

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

	bool ret = a_FeatherCAN.ReadPacketNew(2, &data1);


	if(ret) {
		for(i = 0; i < data1.length; i++) {
			rxBuf[i] = data1.data[i];
		}

		decodeArmAngleMsg();
	}


}

double ArmAngle::GetAngle()
{
  return angle_f;

}

void ArmAngle::decodeArmAngleMsg()
{
  int anglex10 = 0;

  anglex10 = (rxBuf[0] << 8) | rxBuf[1];
  angle_f = (anglex10 * 1.0) / 10.0;
}

