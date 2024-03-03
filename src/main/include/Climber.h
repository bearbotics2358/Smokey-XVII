#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>


class Climber {
    public:
        Climber(int climberMotorID);          
        void stopClimber();
        void extendClimnber();
        void retractClimber();
        double GetClimberPosition();
        void setPosition();
    private:
        
        ctre::phoenix6::hardware::TalonFX climberMotor;
        frc::PIDController climberPID;
        
};