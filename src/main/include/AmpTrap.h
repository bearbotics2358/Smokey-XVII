#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>


class AmpTrap {
    public:
        AmpTrap(int rollerMotorID, int rotationMotorID, int extensionMotorID); 
        void runRoller();
        void stopRoller();
        void extendExtender();  
        double extensionPosition();      
        double GetExtensionPosition(); 
        void setPosition();
        void moveToPosition();
        double GetRotationPosition();
    private:
        
        ctre::phoenix6::hardware::TalonFX rollerMotor;
        ctre::phoenix6::hardware::TalonFX rotationMotor;
        ctre::phoenix6::hardware::TalonFX extensionMotor;

        frc::PIDController extendPID;
        frc::PIDController rotationPID;
        
};