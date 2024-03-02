#include "Shooter.h"
#include "misc.h"
#include "Prefs.h"
#include <cmath>
#include <ctre/phoenix6/configs/Configs.hpp>

Shooter::Shooter(int rightShooterMotorID, int leftShooterMotorID, int pivotMotorID): //int limitSwitchID):

rightShooterMotor(rightShooterMotorID),
leftShooterMotor(leftShooterMotorID),
pivotMotor(pivotMotorID),
// pivotEncoder(pivotMotor),
//shooterLimitSwitch(limitSwitchID),
pivotPID(0.005, 0.0005, 0.0),
leftShooterPID(0.0, 0.0, 0.0),
rightShooterPID(0.0, 0.0, 0.0)
{
    ctre::phoenix6::configs::TalonFXConfiguration pivot_config_angle{};
    pivot_config_angle.Feedback.SensorToMechanismRatio = .74;
    pivotMotor.GetConfigurator().Apply(pivot_config_angle);
    ctre::phoenix6::configs::MotorOutputConfigs pivot_output_config{};
    pivot_output_config.Inverted = true;
    pivotMotor.GetConfigurator().Apply(pivot_output_config);
    pivotMotor.SetNeutralMode(1);
    //pivotPID.SetTolerance(5.0);
    stopShooter();
}
void Shooter::setSpeed(double rpm){
    double velo = rpm/6000.0;
    rightShooterMotor.Set(-velo);
    leftShooterMotor.Set(-velo);
 }
double Shooter::getSpeed(){
    double units = rightShooterMotor.GetVelocity().GetValue().value(); //+ leftShooterMotor.GetVelocity())/2;
    //return; //(units * 600.0) / FALCON_UNITS_PER_REV;; 
    return units;
}
void Shooter::setShooterAngle(){
    pivotMotor.SetPosition(units::angle::turn_t{units::degree_t{20.0}});
}
void Shooter::stopShooter(){
   rightShooterMotor.StopMotor();
   leftShooterMotor.StopMotor();
}
void Shooter::moveToAngle(double angle){
    pivotPID.SetSetpoint(angle);
    double degrees = GetShooterAngle();
    double speed = pivotPID.Calculate(degrees, angle);
    speed = std::clamp(speed, -.2, .2);
    pivotMotor.Set(speed);
    if(pivotPID.AtSetpoint()){
        pivotMotor.StopMotor();
    }
}
double Shooter::GetShooterAngle(){
    return (10.0*pivotMotor.GetPosition().GetValue().value());
}