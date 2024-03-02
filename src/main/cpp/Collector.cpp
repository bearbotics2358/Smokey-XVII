#include "Collector.h"
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>


Collector::Collector(int collectorMotorID, int indexerMotorID):
collectorMotor(collectorMotorID),
indexerMotor(indexerMotorID),
beamBreak(BEAMBREAK_PORT),
a_Shooter(SHOOTER_RIGHT_MOTOR_ID, SHOOTER_LEFT_MOTOR_ID, PIVOT_MOTOR_ID),
a_DriverXboxController(DRIVER_PORT)
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
void Collector::indexToShoot() {
    indexerMotor.Set(-0.20);
}
void Collector::indexToAmp() {
    indexerMotor.Set(.25);
}
void Collector::stopIndexer() {
    indexerMotor.StopMotor();
}
void Collector::stopShooter(){
    a_Shooter.stopShooter();
}
bool Collector::beamBroken(){
    return beamBreak.beamBroken();
}