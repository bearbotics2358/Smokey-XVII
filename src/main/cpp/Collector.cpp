#include "Collector.h"
#include "Prefs.h"

Collector::Collector(int collectorMotorID, int indexerMotorID):
collectorMotor(collectorMotorID),
indexerMotor(indexerMotorID)
//beamBreak(BEAMBREAK_PORT)
{

}
void Collector::startCollector() {
    // if(beamBreak.beamBroken()) {
    //     return;
    // }
    collectorMotor.Set(TalonFXControlMode::PercentOutput, .25);
}
void Collector::stopCollector() {
    collectorMotor.Set(TalonFXControlMode::PercentOutput, 0);
}
void Collector::indexToShoot() {
    indexerMotor.Set(TalonFXControlMode::PercentOutput, .1);
}
void Collector::indexToAmp() {
    indexerMotor.Set(TalonFXControlMode::PercentOutput, -.1);
}
void Collector::stopIndexer() {
    indexerMotor.Set(TalonFXControlMode::PercentOutput, 0);
}
void Collector::update() {
    // if(beamBreak.beamBroken()) {
    //     stopCollector();
    // }
}

