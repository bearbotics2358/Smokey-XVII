#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include "LimitSwitch.h"

class Climber {
    public:
        Climber(int climberMotorID,int topPort, int bottomPort);          
        void stopClimber();
        void extendClimnber();
        void retractClimber();

    private:
        
        ctre::phoenix6::hardware::TalonFX climberMotor;
        LimitSwitch topLimitSwitch;
        LimitSwitch bottomLimitSWitch;
        
        
};