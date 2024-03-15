#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include "BeamBreak.h"
#include "Shooter.h"
#include <frc/XboxController.h>

class Collector {
    public:
        Collector(int collectorMotorID, int indexerMotorID);          
        void startCollector(double speed);
        void stopCollector();
        void indexToShoot();
        void indexToAmp();
        void stopIndexer();
        void stopShooter();
        bool beamBroken();
        void runCollectorback();
        double getShooterAngle();
        void setShooterAngle();
        void indexToCollect();
        BeamBreak beamBreak;
    private:
        // rev::CANSparkMax collectorMotor;
        // rev::CANSparkMax indexerMotor;
        ctre::phoenix6::hardware::TalonFX indexerMotor;
        ctre::phoenix6::hardware::TalonFX collectorMotor;
};