#include "Climber.h"
#include "Prefs.h"

Climber::Climber(int climberMotorID, int topPort, int bottomPort):
climberMotor(climberMotorID),
topLimitSwitch(topPort),
bottomLimitSWitch(bottomPort)
{

}
void Climber::stopClimber(){
    climberMotor.StopMotor();
}
void Climber::extendClimnber(){
    
    if (topLimitSwitch.limitSwitchPressed()){
        stopClimber();
    }
    else {
       climberMotor.Set(.5);
    }
}
void Climber::retractClimber(){
   
   if (bottomLimitSWitch.limitSwitchPressed()){
        stopClimber();
   }
   else {
     climberMotor.Set(-.5);
   }
}
