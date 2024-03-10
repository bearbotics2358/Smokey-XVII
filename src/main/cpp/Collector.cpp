#include "Collector.h"
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>


Collector::Collector(int collectorMotorID, int indexerMotorID):
collectorMotor(collectorMotorID),
indexerMotor(indexerMotorID),
beamBreak(BEAMBREAK_PORT)
{
    stopCollector();
    stopIndexer();
}
void Collector::startCollector(double speed) {
    collectorMotor.Set(speed);
}
void Collector::runCollectorback(){
    collectorMotor.Set(.25);
}
void Collector::stopCollector() {
    collectorMotor.StopMotor();
}
void Collector::indexToCollect(){
    indexerMotor.Set(-.2);
}
void Collector::indexToShoot() {
    indexerMotor.Set(-0.40);
}
void Collector::indexToAmp() {
    indexerMotor.Set(.25);
}
void Collector::stopIndexer() {
    indexerMotor.StopMotor();
}
bool Collector::beamBroken(){
    return beamBreak.beamBroken();
}