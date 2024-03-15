#include "AmpTrap.h"
#include <units/length.h>
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>


AmpTrap::AmpTrap(int rollerMotorID, int rotationMotorID, int extensionMotorID):

rollerMotor(rollerMotorID),
extensionMotor(extensionMotorID),
rotationMotor(rotationMotorID),
extendPID(0.0, 0.0, 0.0),
rotationPID(0.0, 0.0, 0.0),
a_BeamBreak(AMP_BEAM_BREAK_PORT), 
m_rotationMotorSignal(rotationMotor.GetPosition()),
m_extensionMotorSignal(extensionMotor.GetPosition()),

a_ArmAngle(1)
{
    rotationPID.SetTolerance(3.0);
    rotationMotor.SetNeutralMode(1);

    m_rotationMotorSignal.SetUpdateFrequency(units::frequency::hertz_t(10.0));
    m_extensionMotorSignal.SetUpdateFrequency(units::frequency::hertz_t(10.0));
    
}
void AmpTrap::runRoller(){
    rollerMotor.Set(-0.25);
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
    double rotations = extensionMotor.GetPosition().GetValue().value();
    return rotations * M_PI * EXTENSION_DIAMETER; //need to update extension diameter and double check calculations


}
void AmpTrap::setPosition(){
    extensionMotor.SetPosition(units::angle::turn_t{0.0});
}
bool AmpTrap::moveToPosition(double desiredaAngle){
    frc::SmartDashboard::PutNumber("desired arm angle", desiredaAngle);
    rotationPID.SetSetpoint(desiredaAngle);//neeed to change from 0.0
    double angle = GetArmAngle();
    frc::SmartDashboard::PutNumber("PID arm angle", angle);
    double speed = rotationPID.Calculate(angle, desiredaAngle);
    speed = std::clamp(speed, -.2, .2);
    frc::SmartDashboard::PutNumber("Calc speed", speed);
    rotationMotor.Set(speed);
    if(fabs(angle - desiredaAngle) <= 3.0){
        rotationMotor.StopMotor();
        return true;
    }
    else{
        return false;
    }
}
double AmpTrap::GetArmAngle(){
    a_ArmAngle.Update();
    return a_ArmAngle.GetAngle();
}
void AmpTrap::update(){
    frc::SmartDashboard::PutNumber("ArmAnlge", GetArmAngle());
    frc::SmartDashboard::PutNumber("ExtensionPosition", GetExtensionPosition());

}
bool AmpTrap::beamBroken(){
    return a_BeamBreak.beamBroken();
}
void AmpTrap::setPID(double p, double i, double d){
    rotationPID.SetP(p);
    rotationPID.SetI(i);
    rotationPID.SetD(d);
}