#include "AmpTrap.h"
#include <units/length.h>
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>


AmpTrap::AmpTrap(int rollerMotorID, int rotationMotorID, int extensionMotorID):

rollerMotor(rollerMotorID),
extensionMotor(extensionMotorID),
rotationMotor(rotationMotorID),
//.08 .008
extendPID(0.3, 0.03, 0.0),
rotationPID(0.0, 0.0, 0.0),
a_BeamBreak(AMP_BEAM_BREAK_PORT) ,
// m_rotationMotorSignal(rotationMotor.GetPosition()),
// m_extensionMotorSignal(extensionMotor.GetPosition()),

a_ArmAngle(1)
{
    rotationPID.SetTolerance(3.0);
    rotationMotor.SetNeutralMode(1);


    // m_rotationMotorSignal.SetUpdateFrequency(units::frequency::hertz_t(10.0));
    // m_extensionMotorSignal.SetUpdateFrequency(units::frequency::hertz_t(10.0));
    
}
void AmpTrap::runRoller(){
    rollerMotor.Set(-0.32);
}
void AmpTrap::stopRoller(){
    rollerMotor.StopMotor();
}
bool AmpTrap::extendExtender(double goal){
    extendPID.SetSetpoint(goal);//neeed to change from 0.0
    double dist = GetExtensionPosition();
    double speed = extendPID.Calculate(dist, goal);
    speed = std::clamp(speed, -1.0, 1.0);
    extensionMotor.Set(-speed);
    if(fabs(dist-goal) < .25){
        extensionMotor.StopMotor();
        return true;
    }
    return false;
}
void AmpTrap::stopExtension(){
    extensionMotor.StopMotor();
}

double AmpTrap::GetExtensionPosition(){
    double conversion = (-3.0)/(.30975 + 19.312988);
    return conversion*(extensionMotor.GetPosition().GetValue().value());
    // .30975 -19.312988
    

}
void AmpTrap::setPosition(){
    extensionMotor.SetPosition(units::angle::turn_t{0.0});
}
bool AmpTrap::moveToPosition(double desiredaAngle){
    rotationPID.SetSetpoint(desiredaAngle);//neeed to change from 0.0
    double angle = GetArmAngle();
    double speed = rotationPID.Calculate(angle, desiredaAngle);
    speed = std::clamp(speed, -.1, .1);
    rotationMotor.Set(speed);
    if(fabs(angle - desiredaAngle) <= 6.0){
        rotationMotor.StopMotor();
        return true;
    }
    else{
        return false;
    }
}
void AmpTrap::stopArm(){
    rotationMotor.StopMotor();
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
