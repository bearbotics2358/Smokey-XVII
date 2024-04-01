#include "Shooter.h"
#include "misc.h"
#include "Prefs.h"
#include <cmath>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>

Shooter::Shooter(int rightShooterMotorID, int leftShooterMotorID, int pivotMotorID, int limitSwitchID):

rightShooterMotor(rightShooterMotorID),
leftShooterMotor(leftShooterMotorID),
pivotMotor(pivotMotorID),
shooterLimitSwitch(limitSwitchID),
// .005, .0045, 0.00025
//0.003, 0.0015, 0.00012
pivotPID(0.005, 0.000, 0.000),
leftShooterPID(0.0, 0.0, 0.0),
rightShooterPID(0.0, 0.0, 0.0)
{
    pivotPID.SetTolerance(3.0);
    // ctre::phoenix6::configs::TalonFXConfiguration pivot_config_angle{};
    // pivot_config_angle.Feedback.SensorToMechanismRatio = .62;
    // pivotMotor.GetConfigurator().Apply(pivot_config_angle);

    // Set to brake mode so the shooter can hold its position
    pivotMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    // Configure the pivotMotor to zero out the encoder when reaching the limit
    ctre::phoenix6::configs::HardwareLimitSwitchConfigs pivot_limit_config{};
    pivot_limit_config.ForwardLimitAutosetPositionEnable = true;
    pivot_limit_config.ReverseLimitAutosetPositionEnable = true;
    pivot_limit_config.ForwardLimitAutosetPositionValue = 0.0;
    pivot_limit_config.ReverseLimitAutosetPositionValue = 0.0;
    pivotMotor.GetConfigurator().Apply(pivot_limit_config);

    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kV = .12;
    slot0Configs.kP = 0.0; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.0; // no output for error derivative
    leftShooterMotor.GetConfigurator().Apply(slot0Configs);
    rightShooterMotor.GetConfigurator().Apply(slot0Configs);

    ctre::phoenix6::controls::VelocityVoltage m_request = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
    
    

    //pivotPID.SetTolerance(5.0);
    stopShooter();
}
void Shooter::setSpeed(double rpm){
  
   
    rightShooterMotor.SetControl(m_request.WithVelocity(units::angular_velocity::turns_per_second_t{-rpm/60.0} ));
    leftShooterMotor.SetControl(m_request.WithVelocity(units::angular_velocity::turns_per_second_t{-rpm/60.0} ));
    
    // double velo = rpm/6000;
    // rightShooterMotor.Set(-velo);
    // leftShooterMotor.Set(-velo);
 }
double Shooter::getSpeed(){
    double units = rightShooterMotor.GetVelocity().GetValue().value(); //+ leftShooterMotor.GetVelocity())/2;
    //return; //(units * 600.0) / FALCON_UNITS_PER_REV;; 
    return units;
}

void Shooter::UpdateSensors() {
    if(shooterLimitSwitch.limitSwitchPressed()){
        if(!shooterAlreadyZeroed){
            pivotMotor.SetPosition(zeroShooter);
            shooterAlreadyZeroed = true;
        }
    } else{
        shooterAlreadyZeroed = false;
    }
    // bool pivot_limit_pressed = shooterLimitSwitch.limitSwitchPressed();

    // // Since we're not using the closed-loop controls for the motor, put the SetControl behind the
    // // limit switch check since setting a Duty Cycle of 0 will stop the motor
    // if (pivot_limit_pressed) {
    //     // When the limit switch is pressed, stop the motor and zero out the encoder
    //     pivotMotor.SetControl(m_pivotDutyCycleRequest.WithOutput(0)
    //                           .WithLimitForwardMotion(pivot_limit_pressed)
    //                           .WithLimitReverseMotion(pivot_limit_pressed));
    // }
}

void Shooter::stopShooter(){
   rightShooterMotor.StopMotor();
   leftShooterMotor.StopMotor();
   pivotMotor.StopMotor();
}
bool Shooter::moveToAngle(double angle){
    pivotPID.SetSetpoint(angle);
    double degrees = GetShooterAngle();
    double speed = pivotPID.Calculate(degrees, angle);
    speed = std::clamp(speed, -.2, .2);
    pivotMotor.Set(-speed);
    if (fabs(GetShooterAngle() - angle) <= 3.0){
        pivotMotor.StopMotor();
        return true;
    }
    else{
       return false;
    }

}
double Shooter::GetShooterAngle(){
    return (((-8.876)*pivotMotor.GetPosition().GetValue().value()));
}