#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include "ArmAngle.h"
#include "BeamBreak.h"
#include "LimitSwitch.h"


class AmpTrap {
    public:
        AmpTrap(int rollerMotorID, int rotationMotorID, int extensionMotorID); 
        void runRoller(double rps);
        void stopRoller();
        bool extendExtender(double goal);  
        double extensionPosition();      
        double GetExtensionPosition(); 
        void setPosition();
        bool moveToPosition(double desiredaAngle);
        bool trapMoveToPosition(double desiredaAngle);
        double GetArmAngle();
        void update();
        bool beamBroken();
        void setPID(double p, double i, double d);
        void stopArm();
        void stopExtension();
    private:
        
        ctre::phoenix6::hardware::TalonFX rollerMotor;
        ctre::phoenix6::hardware::TalonFX rotationMotor;
        ctre::phoenix6::hardware::TalonFX extensionMotor;

        // ctre::phoenix6::StatusSignal<units::turn_t> m_rotationMotorSignal;
        // ctre::phoenix6::StatusSignal<units::turn_t> m_extensionMotorSignal;

        frc::PIDController extendPID;
        frc::PIDController rotationPID;
        frc::PIDController trapPID;

        BeamBreak a_BeamBreak;
        ArmAngle a_ArmAngle;

        ctre::phoenix6::controls::VelocityVoltage m_request = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

        units::angle::turn_t zeroExtension{0.0};
        bool extensionAlreadyZeroed = false;

        LimitSwitch extensionLimitSwitch;
        
};