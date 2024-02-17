#include "Shooter.h"
#include "misc.h"
#include "Prefs.h"


Shooter::Shooter(int rightShooterMotorID, int leftShooterMotorID, int pivotMotorID, int limitSwitchID):

// rightShooterMotor(rightShooterMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
// leftShooterMotor(leftShooterMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
rightShooterMotor(rightShooterMotorID),
leftShooterMotor(leftShooterMotorID),
pivotMotor(pivotMotorID),
pivotEncoder(pivotMotor),
shooterLimitSwitch(limitSwitchID),
pivotPID(0.0, 0.0, 0.0),
leftShooterPID(0.0, 0.0, 0.0),
rightShooterPID(0.0, 0.0, 0.0)
{

}
void Shooter::setSpeed(double rpm){
    double velo = (rpm * FALCON_UNITS_PER_REV) / 600.0;
    rightShooterMotor.Set(TalonFXControlMode::Velocity, velo);
    leftShooterMotor.Set(TalonFXControlMode::Velocity, -velo);
 }
double Shooter::getSpeed(){
    double units = (rightShooterMotor.GetSelectedSensorVelocity() + leftShooterMotor.GetSelectedSensorVelocity())/2;
    return (units * 600.0) / FALCON_UNITS_PER_REV;; 
}
void Shooter::setShooterAngle(){
    if(shooterLimitSwitch.limitSwitchPressed()){
        pivotEncoder.SetIntegratedSensorPosition(369.778);
    }

}
void Shooter::stopShooter(){

   rightShooterMotor.Set(TalonFXControlMode::PercentOutput, 0);
   leftShooterMotor.Set(TalonFXControlMode::PercentOutput, 0);
}
void Shooter::moveToAngle(double angle){
    pivotPID.SetSetpoint(angle);
    double degrees = GetShooterAngle();
    double speed = pivotPID.Calculate(degrees, angle);
    speed = std::clamp(speed, -.25, .25);
    pivotMotor.Set(TalonFXControlMode::PercentOutput, speed);
    if(pivotPID.AtSetpoint()){
        pivotMotor.Set(TalonFXControlMode::PercentOutput, 0.0);
    }
}
double Shooter::GetShooterAngle(){
    double scaledTicks = pivotEncoder.GetIntegratedSensorPosition() / SHOOTER_GEAR_RATIO;
    double rotations = (scaledTicks / FALCON_UNITS_PER_REV);
    // angular position in radians
    double angularPosition = rotations * 2 * M_PI;
    double degrees = angularPosition * (180.0/M_PI);
    return degrees;
}
