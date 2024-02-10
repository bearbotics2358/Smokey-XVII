#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>

class Collector{
    public:
        Collector(int collectorMotorID, int indexerMotorID);          
        void startCollector();
        void stopCollector();
        void indexToShoot();
        void indexToAmp();
        void stopIndexer();
        private:
        // rev::CANSparkMax collectorMotor;
        // rev::CANSparkMax indexerMotor;
        TalonFX indexerMotor;
        TalonFX collectorMotor;
};