#include "Climber.h"
#include <units/length.h>

Climber::Climber(int climberMotorID):
climberMotor(climberMotorID),
climberPID(0.0, 0.0, 0.0)
{

}
void Climber::stopClimber(){
    climberMotor.StopMotor();
}
void Climber::extendClimnber(){
    climberPID.SetSetpoint(0.0);//neeed to change from 0.0
    double dist = GetClimberPosition();
    double speed = climberPID.Calculate(dist, 0.0);
    speed = std::clamp(speed, -.2, .2);
    climberMotor.Set(speed);
    if(climberPID.AtSetpoint()){
        climberMotor.StopMotor();
    }
}
void Climber::retractClimber(){
    climberPID.SetSetpoint(0.0);
    double dist = GetClimberPosition();
    double speed = climberPID.Calculate(dist, 0.0);
    speed = std::clamp(speed, -.2, .2);
    climberMotor.Set(speed);
    if(climberPID.AtSetpoint()){
        climberMotor.StopMotor();
    }
}
double Climber::GetClimberPosition(){
    return units::meter_t((climberMotor.GetPosition().GetValue())).value();    
}
void Climber::setPosition(){
    climberMotor.SetPosition(units::angle::turn_t{0.0});
}