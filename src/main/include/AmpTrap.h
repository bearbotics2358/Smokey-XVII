#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include "ArmAngle.h"
#include "BeamBreak.h"


class AmpTrap {
    public:
        AmpTrap(int rollerMotorID, int rotationMotorID, int extensionMotorID); 
        void runRoller();
        void stopRoller();
        void extendExtender();  
        double extensionPosition();      
        double GetExtensionPosition(); 
        void setPosition();
        bool moveToPosition(double desiredaAngle);
        double GetArmAngle();
        void update();
        bool beamBroken();
        void setPID(double p, double i, double d);
    private:
        
        ctre::phoenix6::hardware::TalonFX rollerMotor;
        ctre::phoenix6::hardware::TalonFX rotationMotor;
        ctre::phoenix6::hardware::TalonFX extensionMotor;

        frc::PIDController extendPID;
        frc::PIDController rotationPID;

        BeamBreak a_BeamBreak;
        ArmAngle a_ArmAngle;
        
};