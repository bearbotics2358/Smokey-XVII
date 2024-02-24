#include "Shooter.h"
#include "misc.h"
#include "Prefs.h"
#include <cmath>
#include <ctre/phoenix6/configs/Configs.hpp>

Shooter::Shooter(int rightShooterMotorID, int leftShooterMotorID, int pivotMotorID, int limitSwitchID):

rightShooterMotor(rightShooterMotorID),
leftShooterMotor(leftShooterMotorID),
pivotMotor(pivotMotorID),
// pivotEncoder(pivotMotor),
shooterLimitSwitch(limitSwitchID),
pivotPID(0.0125, 0.001, 0.0),
leftShooterPID(0.0, 0.0, 0.0),
rightShooterPID(0.0, 0.0, 0.0)
{
    ctre::phoenix6::configs::TalonFXConfiguration pivot_config_angle{};
    pivot_config_angle.Feedback.SensorToMechanismRatio = .1545;
    pivotMotor.GetConfigurator().Apply(pivot_config_angle);
    ctre::phoenix6::configs::MotorOutputConfigs pivot_output_config{};
    pivot_output_config.Inverted = true;
    pivotMotor.GetConfigurator().Apply(pivot_output_config);
    pivotMotor.SetNeutralMode(1);
    pivotPID.SetTolerance(5.0);
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
    double degrees = GetShooterAngle().value();
    double speed = pivotPID.Calculate(degrees, angle);
    speed = std::clamp(speed, -.5, .5);
    pivotMotor.Set(speed);
    if(pivotPID.AtSetpoint()){
        // pivotMotor.Set(0.0);
        pivotMotor.StopMotor();
    }
}
units::degree_t Shooter::GetShooterAngle(){

   // return 360.0*(pivotMotor.GetPosition().GetValue().value() / 5.0);
    return units::degree_t{10.0*pivotMotor.GetPosition().GetValue().value()};
    // double scaledTicks = pivotMotor.GetPosition().GetValueAsDouble() / SHOOTER_GEAR_RATIO;
    // double rotations = (scaledTicks / FALCON_UNITS_PER_REV);
    // //angular position in radians
    // double angularPosition = rotations * 2 * M_PI;
    // double degrees = angularPosition * (180.0/M_PI);
     //degrees;
}

double Shooter::velocity_needed_to_reach_target(double theta) {
	return sqrt((19.6 * SPEAKER_HEIGHT) / pow(sin(theta), 2)) * VELOCITY_CONSTANT;
}

double Shooter::range(double x, double y) {
	return sqrt(x * x + y * y);
}

double Shooter::calculate_shooting_angle(double range) {
	return atan2((2 * SPEAKER_HEIGHT), range);
}

double Shooter::velocity_to_rpm(double velocity) {
	return (30 * velocity) / (M_PI * RADIUS_OF_MOTOR);
}
