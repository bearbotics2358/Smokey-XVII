#include "Climber.h"
#include "Prefs.h"

Climber::Climber(int climberMotorID, int topLimitSwitchPort)://, int topLimitSwitchPort)://, int topPort, int bottomPort):
climberMotor(climberMotorID),
topLimitSwitch(topLimitSwitchPort),
climberPID(0.04, 0.0, 0.0)
//m_climberMotorSignal(climberMotor.GetPosition())
{
    //m_climberMotorSignal.SetUpdateFrequency(units::frequency::hertz_t(10.0));
    climberMotor.SetNeutralMode(1);
}
void Climber::stopClimber(){
    climberMotor.StopMotor();
}
void Climber::extendClimnber(){
    
    climberPID.SetSetpoint(10.0);//neeed to change from 0.0
    double dist = GetClimberPosition();
    double speed = climberPID.Calculate(dist, 10.0);
    speed = std::clamp(speed, -.2, .2);
    climberMotor.Set(-speed);
    if(climberPID.AtSetpoint() ){
        climberMotor.StopMotor();
    }
}
void Climber::runClimberUp(){
    if(topLimitSwitch.limitSwitchPressed()){
        climberMotor.StopMotor();
    }
    else{
    climberMotor.Set(.05);
    }
}
void Climber::runClimberDown(){
    climberMotor.Set(-.05);
}

void Climber::retractClimber(){
//    if(climberPID.AtSetpoint() || topLimitSwitch.limitSwitchPressed()){
//         climberMotor.StopMotor();
//     }
//     else{
        climberPID.SetSetpoint(0.0);//neeed to change from 0.0
        double dist = GetClimberPosition();
        double speed = climberPID.Calculate(dist, 0.0);
        speed = std::clamp(speed, -.2, .2);
        climberMotor.Set(speed);
   // }
    
}
double Climber::GetClimberPosition(){
    // HOOK moved 6 in and the change in turns was below
    double conversion = 6.0/(-69.629395- -31.272461);
    return (climberMotor.GetPosition().GetValue().value())*conversion;
}
void Climber::setPosition(){
    if(topLimitSwitch.limitSwitchPressed()){
        climberMotor.SetPosition(units::angle::turn_t{0.0});
    }
}

