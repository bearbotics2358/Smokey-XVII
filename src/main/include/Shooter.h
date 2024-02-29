#pragma once
#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/controller/PIDController.h>
#include "LimitSwitch.h"

class Shooter{
    public:
        Shooter(int rightShooterMotorID, int leftShooterMotorID, int pivotMotorID);
        void setSpeed(double percent);
        void stopShooter();
        void moveToAngle(double rpm);
        double getSpeed();
        void setShooterAngle();
        double GetShooterAngle();
        double velocity_needed_to_reach_target(double theta);
        double range(double x, double y);
        double calculate_shooting_angle(double angle);
        double velocity_to_rpm(double velocity);

    private:
    // rev::CANSparkMax rightShooterMotor;
    // rev::CANSparkMax leftShooterMotor;
        frc::PIDController leftShooterPID;
        frc::PIDController rightShooterPID;
        frc::PIDController pivotPID;

        ctre::phoenix6::hardware::TalonFX rightShooterMotor;
        ctre::phoenix6::hardware::TalonFX leftShooterMotor;
        ctre::phoenix6::hardware::TalonFX pivotMotor;

        //TalonFXSensorCollection pivotEncoder;

        
        //LimitSwitch shooterLimitSwitch;
    
};