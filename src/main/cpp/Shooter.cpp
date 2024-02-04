#include "Shooter.h"
#include "misc.h"


Shooter::Shooter(int rightShooterMotorID, int leftShooterMotorID, int pivotMotorID):

// rightShooterMotor(rightShooterMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
// leftShooterMotor(leftShooterMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
rightShooterMotor(rightShooterMotorID),
leftShooterMotor(leftShooterMotorID),
pivotMotor(pivotMotorID),
pivotEncoder(pivotMotor),
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
