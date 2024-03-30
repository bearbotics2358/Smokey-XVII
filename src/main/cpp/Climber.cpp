#include "Climber.h"
#include "Prefs.h"

Climber::Climber(int climberMotorID, int topLimitSwitchPort)://, int topLimitSwitchPort)://, int topPort, int bottomPort):
climberMotor(climberMotorID),
topLimitSwitch(topLimitSwitchPort),
climberPID(0.08, 0.008, 0.0)
//m_climberMotorSignal(climberMotor.GetPosition())
{
    climberPID.SetTolerance(1.0);
    //m_climberMotorSignal.SetUpdateFrequency(units::frequency::hertz_t(10.0));
    climberMotor.SetNeutralMode(1);
}
void Climber::stopClimber(){
    climberMotor.StopMotor();
}
bool Climber::extendClimnber(double position){
        climberPID.SetSetpoint(position);//neeed to change from 0.0
        double dist = GetClimberPosition();
        double speed = climberPID.Calculate(dist, position);
        speed = std::clamp(speed, -1.0, 1.0);
        /* Code that I believe is equivalent.
            if(topLimitSwitch.limitSwitchPressed()) {
                stopClimber();
            } else if(-speed < 0.0) {
                climberMotor.Set(-speed);
            }
        */
        if(-speed > 0.0){
            if(topLimitSwitch.limitSwitchPressed()){
                stopClimber();
            }
        }
        if(fabs(dist - position) < .25){
            climberMotor.StopMotor();
            return true;
        }
        else{
            climberMotor.Set(-speed);
            return false;
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

