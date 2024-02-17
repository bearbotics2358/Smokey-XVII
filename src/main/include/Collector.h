#pragma once
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include "BeamBreak.h"

class Collector {
    public:
        Collector(int collectorMotorID, int indexerMotorID);          
        void startCollector();
        void stopCollector();
        void indexToShoot();
        void indexToAmp();
        void stopIndexer();
        void update();
    private:
        // rev::CANSparkMax collectorMotor;
        // rev::CANSparkMax indexerMotor;
        TalonFX indexerMotor;
        TalonFX collectorMotor;
        BeamBreak beamBreak;
};