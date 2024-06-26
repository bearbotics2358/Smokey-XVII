#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include "LimitSwitch.h"

class Climber {
    public:
        Climber(int climberMotorID, int topLimitSwitchPort);//, int topLimitSwitchPort);//,int topPort, int bottomPort);          
        void stopClimber();
        bool extendClimnber(double position);
        void retractClimber();
        double GetClimberPosition();
        void setPosition();
        void runClimberUp();
        void runClimberDown();

    private:
        
        ctre::phoenix6::hardware::TalonFX climberMotor;

        //ctre::phoenix6::StatusSignal<units::turn_t> m_climberMotorSignal;
        
        frc::PIDController climberPID;
        LimitSwitch topLimitSwitch; 

        units::angle::turn_t zeroClimber{0.0};
        bool climberAlreadyZeroed = false;
        
};