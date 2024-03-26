#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include "LimitSwitch.h"
#include <InterpolatingMap.h>
#include <InterpolationValues.h>

class Shooter{
    public:
        Shooter(int rightShooterMotorID, int leftShooterMotorID, int pivotMotorID, int limitSwitchID);
        void setSpeed(double percent);
        void stopShooter();
        bool moveToAngle(double angle);
        double getSpeed();
        void UpdateSensors();
        double GetShooterAngle();
    private:

        frc::PIDController leftShooterPID;
        frc::PIDController rightShooterPID;
        frc::PIDController pivotPID;

        ctre::phoenix6::hardware::TalonFX rightShooterMotor;
        ctre::phoenix6::hardware::TalonFX leftShooterMotor;
        ctre::phoenix6::hardware::TalonFX pivotMotor;

        ctre::phoenix6::controls::VelocityVoltage m_request = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
        ctre::phoenix6::controls::DutyCycleOut m_pivotDutyCycleRequest{0.0};


        LimitSwitch shooterLimitSwitch;
};