#include "Autonomous.h"
#include "buttons.h"
#include "misc.h"
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <sys/time.h>

//left if positive degrees right is negative
Autonomous::Autonomous(Gyro *Gyro, SwerveDrive *SwerveDrive, NoteHandler *NoteHandler):
a_Gyro(Gyro),
a_SwerveDrive(SwerveDrive),
a_NoteHandler(NoteHandler),
autoDrivePID(.4, .1, 0) 
{}



//-------------------------------------Auto Stuff---------------------------------------------//

void Autonomous::StartAuto(const std::string autoMode) {
    if(autoMode == RobotDoNothing){
        DoNothing();
    }
    else if (autoMode == firstNote){
        NoteOne();
    }
    else if (autoMode == secondNote){
        NoteTwo();
    }
    else if (autoMode == thirdNote){
        NoteThree();
    }
    else if (autoMode == fourthNote){
        NoteFour();
    }
    else if (autoMode == fifthNote){
        NoteFive();
    }
    else if (autoMode == sixthNote){
        NoteSix();
    }
    else if (autoMode ==  seventhNote){
        NoteSeven();
    }
    else if (autoMode == eighthNote){
        NoteEight();
    }

    a_AutoSelected = autoMode; 
    
}
void Autonomous::PeriodicAuto(const std::string periodicAutoMode) {
    if(periodicAutoMode == RobotDoNothing){
        PeriodicDoNothing();
    }
    else if (periodicAutoMode == firstNote){
        PeriodicNoteOne();
    }
    else if (periodicAutoMode == secondNote){
        PeriodicNoteTwo();
    }
    else if (periodicAutoMode ==  thirdNote){
        PeriodicNoteThree();
    }
    else if (periodicAutoMode == fourthNote){
        PeriodicNoteFour();
    }
    else if (periodicAutoMode == fifthNote){
        PeriodicNoteFive();
    }
    else if (periodicAutoMode == sixthNote){
        PeriodicNoteSix();
    }
    else if (periodicAutoMode == seventhNote){
        PeriodicNoteSeven();
    }
    else if (periodicAutoMode == eighthNote){
        PeriodicNoteEight();
    }
   

    a_PeriodicAutoSelected = periodicAutoMode; 
    
}




// s----------------------------------AUTONOMOUS ROUTINES---------------------------------------- //
//DO NOTHING
void Autonomous::DoNothing() {
    a_AutoState0 = kAutoIdle0;
}

void Autonomous::PeriodicDoNothing() {
    
    AutoState0 nextState = a_AutoState0;

    switch (a_AutoState0) {
        case kAutoIdle0:
            StopSwerves();
            break;
    }
    a_AutoState0 = nextState;
}

//RIGHT 2 NOTE
void Autonomous::NoteOne() {
    a_SwerveDrive->zeroPose(frc::Pose2d(units::meter_t(0.0), units::meter_t(0.0), units::degree_t(0.0)));
    a_Gyro->Zero(0.0);
    a_AutoState1 = kRotateToShoot1;
    state_time = misc::gettime_d();
}

void Autonomous::PeriodicNoteOne() {

    AutoState1 nextState = a_AutoState1;

    switch (a_AutoState1) {
        case kAutoIdle1:
            StopSwerves();
            break;
        case kRotateToShoot1:
            if(a_SwerveDrive->odometryGoToPose(0.5, 0.0, 5*(M_PI/3))){
                state_time = misc::gettime_d();
                nextState = kShootFirstNote1;
            }
            break;
        case kShootFirstNote1:
            if(misc::gettime_d() > state_time + 1.0){
                a_NoteHandler->shootNote(-.25);
                state_time = misc::gettime_d();
                nextState = kGoToSecondNote1;
            }
            break;
        case kGoToSecondNote1:
            a_NoteHandler->collectNote(-.4, true);
            if(misc::gettime_d() > state_time + 1.0){
                if(a_SwerveDrive->odometryGoToPose(-1.602, 0.665, 0.0)){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeaker1;
                }
            }
            break;
        case kGoToSpeaker1:
            if(a_SwerveDrive->odometryGoToPose(0.83, 0.0, 5*(M_PI/3))){
                state_time = misc::gettime_d();
                nextState = kShootSecondNote1;
            }
            break;
        case kShootSecondNote1:
            a_NoteHandler->shootNote(-.25);
            if(misc::gettime_d() > state_time + 1.0){
                state_time = misc::gettime_d();
                nextState = kAutoIdle1;
            }
            break;   
    }
    a_AutoState1 = nextState;
}

//CENTER TWO NOTE
void Autonomous::NoteTwo() {
    a_SwerveDrive->zeroPose(frc::Pose2d(units::meter_t(1.3), units::meter_t(0.0), units::degree_t(0.0)));
    a_Gyro->Zero(0.0);
    //a_Gyro->Zero(300.0);
    a_AutoState2 = kShootFirstNote2;   
    state_time = misc::gettime_d();
}

void Autonomous::PeriodicNoteTwo() {

    AutoState2 nextState = a_AutoState2;
    
    

    switch (a_AutoState2) {
        case kAutoIdle2:
            StopSwerves();
            break;
        case kShootFirstNote2:
            
            if(misc::gettime_d() > state_time + 1.0){
                a_NoteHandler->shootNote(-.25);
                state_time = misc::gettime_d();
                nextState = kGoToSecondNote2;
            }
            break;
        case kGoToSecondNote2:
            a_NoteHandler->collectNote(-.4, true);
            if(misc::gettime_d() > state_time + 1.0){
                if(a_SwerveDrive->odometryGoToPose(-.3, 0.0, 0.0)){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeaker2;
                }
            }
            break;
        case kGoToSpeaker2:
            if(a_SwerveDrive->odometryGoToPose(1.3, 0.0, 0.0)){
                state_time = misc::gettime_d();
                nextState = kShootSecondNote2;
            }
            break;
        case kShootSecondNote2:
            a_NoteHandler->shootNote(-.25);
            if(misc::gettime_d() > state_time + 1.0){
                state_time = misc::gettime_d();
                nextState = kAutoIdle2;
            }
            break;


              
    }
    a_AutoState2 = nextState;
}

//LEFT 2 NOTE
void Autonomous::NoteThree() 
{
   a_SwerveDrive->zeroPose(frc::Pose2d(units::meter_t(0.0), units::meter_t(0.0), units::degree_t(0.0)));
    a_Gyro->Zero(0.0);
    a_AutoState3 = kRotateToShoot3;   
    state_time = misc::gettime_d();
}

void Autonomous::PeriodicNoteThree() {
    AutoState3 nextState = a_AutoState3;
    frc::SmartDashboard::PutNumber("Auto State", nextState);
   
    switch (a_AutoState3) {
        case kAutoIdle3:
            StopSwerves();
            break;
        case kRotateToShoot3:
            if(a_SwerveDrive->odometryGoToPose(0.5, 0.0, M_PI/3)){
                state_time = misc::gettime_d();
                nextState = kShootFirstNote3;
            }
            break;
        case kShootFirstNote3:
            if(misc::gettime_d() > state_time + 1.0){
                a_NoteHandler->shootNote(-.25);
                state_time = misc::gettime_d();
                nextState = kGoToSecondNote3;
            }
            break;
        case kGoToSecondNote3:
            a_NoteHandler->collectNote(-.4, true);
            if(misc::gettime_d() > state_time + 1.0){
                if(a_SwerveDrive->odometryGoToPose(-1.602, -0.665, 0.0)){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeaker3;
                }
            }
            break;
        case kGoToSpeaker3:
            if(a_SwerveDrive->odometryGoToPose(0.5, 0.0, M_PI/3)){
                state_time = misc::gettime_d();
                nextState = kShootSecondNote3;
            }
            break;
        case kShootSecondNote3:
            a_NoteHandler->shootNote(-.25);
            if(misc::gettime_d() > state_time + 1.0){
                state_time = misc::gettime_d();
                nextState = kAutoIdle3;
            }
            break;   
    }
    a_AutoState3 = nextState;
}
//4 Note Center
void Autonomous::NoteFour(){
    a_SwerveDrive->zeroPose(frc::Pose2d(units::meter_t(1.3), units::meter_t(0.0), units::degree_t(0.0)));
    a_Gyro->Zero(0.0);
    a_AutoState4 = kShootFirstNote4;   
    state_time = misc::gettime_d();
}

void Autonomous::PeriodicNoteFour() {

    AutoState4 nextState = a_AutoState4;

    switch (a_AutoState4) {
        case kAutoIdle4:
            StopSwerves();
            break;
        case kShootFirstNote4:
            if(misc::gettime_d() > state_time + 0.5){
                a_NoteHandler->shootNote(-.25);
                if(a_NoteHandler->ampBeamBreak()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSecondNote4;
                }
            }
            break;
        case kGoToSecondNote4:
            a_NoteHandler->collectNote(-.4, true);
                if(a_SwerveDrive->odometryGoToPose(-.2, 0.0, 0.0) || a_NoteHandler->beamBroken()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeaker4;
                }
            break;
        case kGoToSpeaker4:
            a_NoteHandler->collectNote(-.4, true);
            if(a_SwerveDrive->odometryGoToPose(1.3, 0.0, 0.0)){
                state_time = misc::gettime_d();
                nextState = kShootSecondNote4;
            }
            break;
        case kShootSecondNote4:
            a_NoteHandler->shootNote(-.25);
            if(a_NoteHandler->ampBeamBreak()){
                state_time = misc::gettime_d();
                nextState = kGoToThirdNote4;
            }
            break;
        case kGoToThirdNote4:
            a_NoteHandler->collectNote(-.4, true);
                if(a_SwerveDrive->odometryGoToPose(-.2, -1.455, M_PI/8) || a_NoteHandler->beamBroken()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeakerAgain4;
                }
            break;
        case kGoToSpeakerAgain4:
            a_NoteHandler->collectNote(-.4, true);
            if(a_SwerveDrive->odometryGoToPose(1.3, 0.0, 0.0)){
                state_time = misc::gettime_d();
                nextState = kShootThirdNote4;
            }
            break;
        case kShootThirdNote4:
            a_NoteHandler->shootNote(-.25);
            if(a_NoteHandler->ampBeamBreak()){
                state_time = misc::gettime_d();
                nextState = kGoToFourthNote4;
            }
            break;
        case kGoToFourthNote4:
            a_NoteHandler->collectNote(-.4, true);
                if(a_SwerveDrive->odometryGoToPose(-.2, 1.455, 15.0*M_PI/8) || a_NoteHandler->beamBroken()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeakerThirdTime4;
            }
            break;
        case kGoToSpeakerThirdTime4:
            a_NoteHandler->collectNote(-.4, true);
            if(a_SwerveDrive->odometryGoToPose(1.3, 0.0, 0.0)){
                state_time = misc::gettime_d();
                nextState = kShootFourthNote4;
            }
            break;
        case kShootFourthNote4:
            a_NoteHandler->shootNote(-.25);
            if(a_NoteHandler->ampBeamBreak()){
                state_time = misc::gettime_d();
                nextState = kAutoIdle4;
            }
            break;
    }
    a_AutoState4 = nextState;
 }
 //RED RIGHT 3 NOTE
 void Autonomous::NoteFive() 
{
    a_SwerveDrive->zeroPose(frc::Pose2d(units::meter_t(0.0), units::meter_t(0.0), units::degree_t(0.0)));
    a_Gyro->Zero(0.0);
    a_AutoState5 = kRotateToShoot5;
    state_time = misc::gettime_d();
}
    
void Autonomous::PeriodicNoteFive() {
    
    AutoState5 nextState = a_AutoState5;
    frc::SmartDashboard::PutNumber("Auto State", nextState);
    switch (a_AutoState5) {
        case kAutoIdle5:
            StopSwerves();
            break;
        case kRotateToShoot5:
            if(a_SwerveDrive->odometryGoToPose(0.5, 0.0, 5*(M_PI/3)) || (a_SwerveDrive->getAvgVelocity() < .25)){
                state_time = misc::gettime_d();
                nextState = kShootFirstNote5;
            }
            break;
        case kShootFirstNote5:
            if(misc::gettime_d() > state_time + 0.5){
                a_NoteHandler->shootNote(-.25);
                if(a_NoteHandler->ampBeamBreak()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSecondNote5;
                }
            }
            break;
        case kGoToSecondNote5:
            a_NoteHandler->collectNote(-.4, true);
            if(misc::gettime_d() > state_time + 1.0){
                if(a_SwerveDrive->odometryGoToPose(-1.602, 0.665, 0.0) || a_NoteHandler->beamBroken()){
                    state_time = misc::gettime_d();

                    nextState = kGoToSpeaker5;
                }
            }
            break;
        case kGoToSpeaker5:
            if(a_SwerveDrive->odometryGoToPose(0.83, 0.0, 5*(M_PI/3))){
                state_time = misc::gettime_d();
                nextState = kShootSecondNote5;
            }
            break;
        case kShootSecondNote5:
            a_NoteHandler->shootNote(-.25);
            if(a_NoteHandler->ampBeamBreak()){
                state_time = misc::gettime_d();
                nextState = kGoToThirdNote5;
            }
            break;   
        case kGoToThirdNote5:
            a_NoteHandler->collectNote(-.4, true);
            if(misc::gettime_d() > state_time + 1.0){
                if(a_SwerveDrive->odometryGoToPose(-6.976, 1.122, 0.0) || a_NoteHandler->beamBroken()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeakerAgain5;
                }
            }
            break;
        case kGoToSpeakerAgain5:
            if(a_SwerveDrive->odometryGoToPose(0.83, 0.0, 5*(M_PI/3))){
                state_time = misc::gettime_d();
                nextState = kShootThirdNote5;
            }
            break;
        case kShootThirdNote5:
            a_NoteHandler->shootNote(-.25);
            if(a_NoteHandler->ampBeamBreak()){
                state_time = misc::gettime_d();
                nextState = kAutoIdle5;
            }
            break;
    }
    a_AutoState5 = nextState;
}
//BLUE LEFT 3 NOTE
void Autonomous::NoteSix(){
    a_SwerveDrive->zeroPose(frc::Pose2d(units::meter_t(0.0), units::meter_t(0.0), units::degree_t(0.0)));
    a_Gyro->Zero(0.0);
    a_AutoState6 = kRotateToShoot6;   
    state_time = misc::gettime_d();
}

void Autonomous::PeriodicNoteSix() {
    AutoState6 nextState = a_AutoState6;
   
    switch (a_AutoState6) {
        case kAutoIdle6:
            StopSwerves();
            break;
        case kRotateToShoot6:
            if(a_SwerveDrive->odometryGoToPose(0.5, 0.0, M_PI/3)){
                state_time = misc::gettime_d();
                nextState = kShootFirstNote6;
            }
            break;
        case kShootFirstNote6:
            if(misc::gettime_d() > state_time + 0.5){
                a_NoteHandler->shootNote(-.25);
                if(a_NoteHandler->ampBeamBreak()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSecondNote6;
                }
            }
            break;
        case kGoToSecondNote6:
            a_NoteHandler->collectNote(-.4, true);
            if(misc::gettime_d() > state_time + 1.0){
                if(a_SwerveDrive->odometryGoToPose(-1.602, -0.665, 0.0) || a_NoteHandler->beamBroken()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeaker6;
                }
            }
            break;
        case kGoToSpeaker6:
            if(a_SwerveDrive->odometryGoToPose(0.5, 0.0, M_PI/3)){
                state_time = misc::gettime_d();
                nextState = kShootSecondNote6;
            }
            break;
        case kShootSecondNote6:
            a_NoteHandler->shootNote(-.25);
            if(a_NoteHandler->ampBeamBreak()){
                state_time = misc::gettime_d();
                nextState = kGoToThirdNote6;
            }
            break;   
         case kGoToThirdNote6:
            a_NoteHandler->collectNote(-.4, true);
            if(misc::gettime_d() > state_time + 1.0){
                if(a_SwerveDrive->odometryGoToPose(-6.977, -1.122, 0.0) || a_NoteHandler->beamBroken()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeakerAgain6;
                }
            }
            break;
        case kGoToSpeakerAgain6:
            if(a_SwerveDrive->odometryGoToPose(0.5, 0.0, (M_PI/3))){
                state_time = misc::gettime_d();
                nextState = kShootThirdNote6;
            }
            break;
        case kShootThirdNote6:
            a_NoteHandler->shootNote(-.25);
            if(a_NoteHandler->ampBeamBreak()){
                state_time = misc::gettime_d();
                nextState = kAutoIdle6;
            }
            break;
    }
    a_AutoState6 = nextState;
}
// RED LEFT 3 NOTE
void Autonomous::NoteSeven() 
{
    a_SwerveDrive->zeroPose(frc::Pose2d(units::meter_t(0.0), units::meter_t(0.0), units::degree_t(0.0)));
    a_Gyro->Zero(0.0);
    a_AutoState7 = kRotateToShoot7;   
    state_time = misc::gettime_d();
}

void Autonomous::PeriodicNoteSeven() {
    AutoState7 nextState = a_AutoState7;
   
    switch (a_AutoState7) {
        case kAutoIdle7:
            StopSwerves();
            break;
        case kRotateToShoot7:
            if(a_SwerveDrive->odometryGoToPose(0.5, 0.0, M_PI/3)){
                state_time = misc::gettime_d();
                nextState = kShootFirstNote7;
            }
            break;
        case kShootFirstNote7:
            if(misc::gettime_d() > state_time + 0.5){
                a_NoteHandler->shootNote(-.25);
                if(a_NoteHandler->ampBeamBreak()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSecondNote7;
                }
            }
            break;
        case kGoToSecondNote7:
            a_NoteHandler->collectNote(-.4, true);
            if(misc::gettime_d() > state_time + 1.0){
                if(a_SwerveDrive->odometryGoToPose(-1.602, -0.665, 0.0) || a_NoteHandler->beamBroken()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeaker7;
                }
            }
            break;
        case kGoToSpeaker7:
            if(a_SwerveDrive->odometryGoToPose(0.5, 0.0, M_PI/3)){
                state_time = misc::gettime_d();
                nextState = kShootSecondNote7;
            }
            break;
        case kShootSecondNote7:
            a_NoteHandler->shootNote(-.25);
            if(a_NoteHandler->ampBeamBreak()){
                state_time = misc::gettime_d();
                nextState = kGoToThirdNote7;
            }
            break;   
         case kGoToThirdNote7:
            a_NoteHandler->collectNote(-.4, true);
            if(misc::gettime_d() > state_time + 1.0){
                if(a_SwerveDrive->odometryGoToPose(-6.977, -4.017, 0.0) || a_NoteHandler->beamBroken()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeakerAgain7;
                }
            }
            break;
        case kGoToSpeakerAgain7:
            if(a_SwerveDrive->odometryGoToPose(0.5, 0.0, (M_PI/3))){
                state_time = misc::gettime_d();
                nextState = kShootThirdNote7;
            }
            break;
        case kShootThirdNote7:
            a_NoteHandler->shootNote(-.25);
            if(a_NoteHandler->ampBeamBreak()){
                state_time = misc::gettime_d();
                nextState = kAutoIdle7;
            }
            break;
    }
    a_AutoState7 = nextState;
}
//BLUE RIGHT 3 Note
void Autonomous::NoteEight() 
{
    a_SwerveDrive->zeroPose(frc::Pose2d(units::meter_t(0.0), units::meter_t(0.0), units::degree_t(0.0)));
    a_Gyro->Zero(0.0);
    a_AutoState8 = kRotateToShoot8;
    state_time = misc::gettime_d();
}

void Autonomous::PeriodicNoteEight() {
    AutoState8 nextState = a_AutoState8;
   
    switch (a_AutoState8) {
        case kAutoIdle8:
            StopSwerves();
            break;
        case kRotateToShoot8:
            if(a_SwerveDrive->odometryGoToPose(0.5, 0.0, 5*(M_PI/3))){
                state_time = misc::gettime_d();
                nextState = kShootFirstNote8;
            }
            break;
        case kShootFirstNote8:
            if(misc::gettime_d() > state_time + 0.5){
                a_NoteHandler->shootNote(-.25);
                if(a_NoteHandler->ampBeamBreak()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSecondNote8;
                }
            }
            break;
        case kGoToSecondNote8:
            a_NoteHandler->collectNote(-.4, true);
            if(misc::gettime_d() > state_time + 1.0){
                if(a_SwerveDrive->odometryGoToPose(-1.602, 0.665, 0.0) || a_NoteHandler->beamBroken()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeaker8;
                }
            }
            break;
        case kGoToSpeaker8:
            if(a_SwerveDrive->odometryGoToPose(0.83, 0.0, 5*(M_PI/3))){
                state_time = misc::gettime_d();
                nextState = kShootSecondNote8;
            }
            break;
        case kShootSecondNote8:
            a_NoteHandler->shootNote(-.25);
            if(a_NoteHandler->ampBeamBreak()){
                state_time = misc::gettime_d();
                nextState = kGoToThirdNote8;
            }
            break;   
        case kGoToThirdNote8:
            a_NoteHandler->collectNote(-.4, true);
            if(misc::gettime_d() > state_time + 1.0){
                if(a_SwerveDrive->odometryGoToPose(-6.976, 4.017, 0.0) || a_NoteHandler->beamBroken()){
                    state_time = misc::gettime_d();
                    nextState = kGoToSpeakerAgain8;
                }
            }
            break;
        case kGoToSpeakerAgain8:
            if(a_SwerveDrive->odometryGoToPose(0.83, 0.0, 5*(M_PI/3))){
                state_time = misc::gettime_d();
                nextState = kShootThirdNote8;
            }
            break;
        case kShootThirdNote8:
            a_NoteHandler->shootNote(-.25);
            if(a_NoteHandler->ampBeamBreak()){
                state_time = misc::gettime_d();
                nextState = kAutoIdle8;
            }
            break;
    }
    a_AutoState8 = nextState;
}

void Autonomous::StopSwerves() {
    a_SwerveDrive->stop();
}

void Autonomous::StartTimer() {
    waitTimeStart = misc::getSeconds();
}




bool Autonomous::TurnToAngle(float angle, bool positive) { // rotates bot in place to specific angle
   
//     if(!positive){
//         speed = -1*speed;
//     }
    if (a_SwerveDrive->turnToAngle(angle, positive)) {
//     //if((a_Gyro->getAngle() - angle <= 10) || ((a_Gyro->getAngle() - angle - 360) <= 10) || (a_Gyro->getAngle() - (angle + 360) <= 10)){
//        if(fabs(a_Gyro->getAngleClamped() - angle) <= 5){
        a_SwerveDrive->stop();
        return true;
    }
    else{
        return false;
    }
       

}
    // else {
    //     a_SwerveDrive->turnToAngle(angle, positive);
    //     return false;
    // }


bool Autonomous::DriveDirection(double dist, double angle, double speed, bool fieldOriented) { // true is done, false is not done
    autoDrivePID.SetSetpoint(dist);
    double calcspeed = autoDrivePID.Calculate(a_SwerveDrive->getAvgDistance(), dist);
    calcspeed = std::clamp(speed, -.25, .25);
    a_SwerveDrive->driveDirection(calcspeed, angle);    
    if (autoDrivePID.AtSetpoint()) {
        // a_SwerveDrive->stop();
        return true;
    } else { 
        return false;
    }
}

bool Autonomous::Balance(float direction) {
    a_SwerveDrive->brakeOnStop();
    float currentTime = misc::gettime_d();
    double tiltAngle = a_Gyro->getPitch() - PITCH_OFFSET;
    double percentTilt = tiltAngle / 15;
    double speed = percentTilt * MAX_FREE_SPEED;
    startedClimb = true;
    if(startedClimb) {
        a_SwerveDrive->driveDirectionVelocity(speed, direction);
        if(abs(tiltAngle) < 5) {
            startTime = misc::gettime_d();
        }
        return false;
    }
    if ((currentTime - startTime > 1.0) && (abs(tiltAngle) < 5) && startedClimb){
        StopSwerves();
        return true;
    }
    else{
        a_SwerveDrive->driveDirection(MAX_CLIMB_PERCENT, direction);
        if(abs(tiltAngle) > 5){
            startedClimb = true;
            a_SwerveDrive->driveDirection(0.2, direction);
        }
        return false;
    }
}