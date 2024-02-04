#include "Collector.h"

Collector::Collector(int collectorMotorID, int indexerMotorID):
collectorMotor(collectorMotorID),
indexerMotor(indexerMotorID)
{

}
void Collector::startCollector(){
    collectorMotor.Set(TalonFXControlMode::PercentOutput, .1);
}
void Collector::stopCollector(){
    collectorMotor.Set(TalonFXControlMode::PercentOutput, 0);
}
void Collector::indexToShoot(){
    indexerMotor.Set(TalonFXControlMode::PercentOutput, .1);
}
void Collector::indexToAmp(){
    indexerMotor.Set(TalonFXControlMode::PercentOutput, -.1);
}
void Collector::stopIndexer(){
    indexerMotor.Set(TalonFXControlMode::PercentOutput, 0);
}

