#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include "LimitSwitch.h"

class Climber {
    public:
        Climber(int climberMotorID);//, int topLimitSwitchPort);//,int topPort, int bottomPort);          
        void stopClimber();
        void extendClimnber();
        void retractClimber();
        double GetClimberPosition();
        void setPosition();

    private:
        
        ctre::phoenix6::hardware::TalonFX climberMotor;

        //ctre::phoenix6::StatusSignal<units::turn_t> m_climberMotorSignal;
        
        frc::PIDController climberPID;
        //LimitSwitch topLimitSwitch; 
        
};