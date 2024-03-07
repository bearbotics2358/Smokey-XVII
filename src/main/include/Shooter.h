#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include "LimitSwitch.h"
#include <wpi/interpolating_map.h>

class Shooter{
    public:
        Shooter(int rightShooterMotorID, int leftShooterMotorID, int pivotMotorID);
        void setSpeed(double percent);
        void stopShooter();
        void moveToAngle(double angle);
        double getSpeed();
        void setShooterAngle();
        double GetShooterAngle();
    private:
    
        frc::PIDController leftShooterPID;
        frc::PIDController rightShooterPID;
        frc::PIDController pivotPID;

        ctre::phoenix6::hardware::TalonFX rightShooterMotor;
        ctre::phoenix6::hardware::TalonFX leftShooterMotor;
        ctre::phoenix6::hardware::TalonFX pivotMotor;
      
        //LimitSwitch shooterLimitSwitch;
};