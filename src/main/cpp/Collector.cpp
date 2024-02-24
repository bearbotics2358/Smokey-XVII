#include "Collector.h"
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>


Collector::Collector(int collectorMotorID, int indexerMotorID):
collectorMotor(collectorMotorID),
indexerMotor(indexerMotorID),
beamBreak(BEAMBREAK_PORT),
a_Shooter(SHOOTER_RIGHT_MOTOR_ID, SHOOTER_LEFT_MOTOR_ID, PIVOT_MOTOR_ID, LIMIT_SWITCH)
{
    stopCollector();
    stopIndexer();
}
void Collector::startCollector() {
    if(beamBreak.beamBroken()) {
        return;
    }
    collectorMotor.Set(-.25);
}
void Collector::stopCollector() {
    collectorMotor.StopMotor();
}
void Collector::indexToShoot() {
    indexerMotor.Set(-0.25);
}
void Collector::indexToAmp() {
    indexerMotor.Set(.1);
}
void Collector::stopIndexer() {
    indexerMotor.StopMotor();
}
void Collector::update() {
    frc::SmartDashboard::PutBoolean("BeamBroken", beamBreak.beamBroken());
    if(beamBreak.beamBroken()) {
        stopCollector();
        if(a_Shooter.getSpeed() == 0.0){
            stopIndexer();
        }
        
    }
}
void Collector::setSpeed(double rpm){
    a_Shooter.setSpeed(rpm);
}
void Collector::stopShooter(){
    a_Shooter.stopShooter();
}

