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
trapPID(.004, 0.0, 0.0001),
a_BeamBreak(AMP_BEAM_BREAK_PORT) ,
// m_rotationMotorSignal(rotationMotor.GetPosition()),
// m_extensionMotorSignal(extensionMotor.GetPosition()),

a_ArmAngle(1)
{
    rotationPID.SetTolerance(3.0);
    rotationMotor.SetNeutralMode(1);

    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kV = .12;
    slot0Configs.kP = 0.0; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.0; // no output for error derivative
    rollerMotor.GetConfigurator().Apply(slot0Configs);
    ctre::phoenix6::controls::VelocityVoltage m_request = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

    // m_rotationMotorSignal.SetUpdateFrequency(units::frequency::hertz_t(10.0));
    // m_extensionMotorSignal.SetUpdateFrequency(units::frequency::hertz_t(10.0));
    
}
void AmpTrap::runRoller(double rps){
    rollerMotor.SetControl(m_request.WithVelocity(units::angular_velocity::turns_per_second_t{rps} ));
    //rollerMotor.Set(-0.35);
}
void AmpTrap::stopRoller(){
    rollerMotor.StopMotor();
}
bool AmpTrap::extendExtender(double goal){
    extendPID.SetSetpoint(goal);//neeed to change from 0.0
    double dist = GetExtensionPosition();
    double speed = extendPID.Calculate(dist, goal);
    speed = std::clamp(speed, -1.0, 1.0);
    
    frc::SmartDashboard::PutNumber("extension goal", 1000000000000);
    if(fabs(dist-goal) < .25){
        frc::SmartDashboard::PutString("a;owieguh;", "oiahg;r");
        extensionMotor.StopMotor();
        return true;
    }
    else{
        extensionMotor.Set(-speed);
        return false;
    }
    
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
    if(fabs(angle - desiredaAngle) <= 6.0){
        rotationMotor.StopMotor();
        return true;
    }
    else{
        rotationMotor.Set(speed);
        return false;
    }
}
bool AmpTrap::trapMoveToPosition(double desiredaAngle){
    rotationPID.SetSetpoint(desiredaAngle);//neeed to change from 0.0
    double angle = GetArmAngle();
    double speed = trapPID.Calculate(angle, desiredaAngle);
    speed = std::clamp(speed, -.1, .1);
    if(fabs(angle - desiredaAngle) <= 6.0){
        rotationMotor.StopMotor();
        return true;
    }
    else{
        rotationMotor.Set(speed);
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
