#include "AmpTrap.h"
#include <units/length.h>

AmpTrap::AmpTrap(int rollerMotorID, int extensionMotorID, int rotationMotorID):

rollerMotor(rollerMotorID),
extensionMotor(extensionMotorID),
rotationMotor(rotationMotorID),
extendPID(0.0, 0.0, 0.0),
rotationPID(0.0, 0.0, 0.0)
{

}
void AmpTrap::runRoller(){
    rollerMotor.Set(0.0);
}
void AmpTrap::stopRoller(){
    rollerMotor.StopMotor();
}
void AmpTrap::extendExtender(){
    extendPID.SetSetpoint(0.0);//neeed to change from 0.0
    double dist = GetExtensionPosition();
    double speed = extendPID.Calculate(dist, 0.0);
    speed = std::clamp(speed, -.2, .2);
    extensionMotor.Set(speed);
    if(extendPID.AtSetpoint()){
        extensionMotor.StopMotor();
    }
}
double AmpTrap::GetExtensionPosition(){
    return units::meter_t((extensionMotor.GetPosition().GetValue())).value();    
}
void AmpTrap::setPosition(){
    extensionMotor.SetPosition(units::angle::turn_t{0.0});
}
void AmpTrap::moveToPosition(){
    rotationPID.SetSetpoint(0.0);//neeed to change from 0.0
    double angle = GetRotationPosition();
    double speed = rotationPID.Calculate(angle, 0.0);
    speed = std::clamp(speed, -.2, .2);
    rotationMotor.Set(speed);
    if(rotationPID.AtSetpoint()){
        rotationMotor.StopMotor();
    }
}
double AmpTrap::GetRotationPosition(){
    return (10.0*rotationMotor.GetPosition().GetValue().value());
}

