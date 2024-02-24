#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include "BeamBreak.h"
#include "Shooter.h"

class Collector {
    public:
        Collector(int collectorMotorID, int indexerMotorID);          
        void startCollector();
        void stopCollector();
        void indexToShoot();
        void indexToAmp();
        void stopIndexer();
        void update();
        void setSpeed(double rpm);
        void stopShooter();
    private:
        // rev::CANSparkMax collectorMotor;
        // rev::CANSparkMax indexerMotor;
        ctre::phoenix6::hardware::TalonFX indexerMotor;
        ctre::phoenix6::hardware::TalonFX collectorMotor;
        BeamBreak beamBreak;
        Shooter a_Shooter;
};