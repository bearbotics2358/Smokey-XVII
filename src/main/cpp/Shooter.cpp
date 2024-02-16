#include "Shooter.h"
#include "misc.h"
#include "Prefs.h"


Shooter::Shooter(int rightShooterMotorID, int leftShooterMotorID, int pivotMotorID):

// rightShooterMotor(rightShooterMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
// leftShooterMotor(leftShooterMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
rightShooterMotor(rightShooterMotorID),
leftShooterMotor(leftShooterMotorID),
pivotMotor(pivotMotorID),
pivotEncoder(pivotMotor),
pivotPID(0.0, 0.0, 0.0),
leftShooterPID(0.0, 0.0, 0.0),
rightShooterPID(0.0, 0.0, 0.0)
{

}
 void Shooter::setSpeed(double percent){

     rightShooterMotor.Set(TalonFXControlMode::PercentOutput, percent);
     leftShooterMotor.Set(TalonFXControlMode::PercentOutput, -percent);
 }
void Shooter::stopShooter(){

   rightShooterMotor.Set(TalonFXControlMode::PercentOutput, 0);
   leftShooterMotor.Set(TalonFXControlMode::PercentOutput, 0);
}
void Shooter::moveToAngle(double angle){
    pivotPID.SetSetpoint(angle);
    double scaledTicks = pivotEncoder.GetIntegratedSensorPosition() / SWERVE_DRIVE_MOTOR_GEAR_RATIO;
    double rotations = (scaledTicks / FALCON_UNITS_PER_REV);
    // angular position in radians
    double angularPosition = rotations * 2 * M_PI;
    double degrees = angularPosition * (180.0/M_PI);
    
    double speed = pivotPID.Calculate(degrees, angle);
    speed = std::clamp(speed, -1.0, 1.0);
    pivotMotor.Set(TalonFXControlMode::PercentOutput, speed);
    if(pivotPID.AtSetpoint()){
        pivotMotor.Set(TalonFXControlMode::PercentOutput, 0.0);
    }
}
