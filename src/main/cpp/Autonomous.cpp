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
a_NoteHandler(),
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


void Autonomous::NoteOne() {
    a_SwerveDrive->zeroPose(frc::Pose2d(units::meter_t(.47), units::meter_t(.79), units::degree_t(60.0)));
    a_AutoState1 = kShoot1stNote1;
    state_time = gettime_d();
}

void Autonomous::PeriodicNoteOne() {

    AutoState1 nextState = a_AutoState1;

    switch (a_AutoState1) {
         case kAutoIdle1:
            StopSwerves();
            break;
        case kShoot1stNote1:
            a_NoteHandler->indexToShoot();
            nextState = kAutoIdle1;
            break;
        case kGoTo2ndNote1:
            a_NoteHandler->collectNote(-0.4, false);
            if(a_SwerveDrive -> odometryGoToPose(8.277, -1.912, 0.0)){
                nextState = kShoot2ndNote1;
            }
            break;
        case kShoot2ndNote1:
            if(a_SwerveDrive -> odometryGoToPose(3.5, -0.5, 0.0)){
                a_NoteHandler->indexToShoot();
                nextState = kGoTo3rdNote1;
            }
            break;
        case kGoTo3rdNote1:
            a_NoteHandler->collectNote(-0.4, false);
            if(a_SwerveDrive -> odometryGoToPose(8.277, -.236, 0.0)){
                nextState = kShoot3rdNote1;
            }
            break;
        case kShoot3rdNote1:
             if(a_SwerveDrive -> odometryGoToPose(3.5, -.5, 0.0)){
                a_NoteHandler->indexToShoot();
                nextState = kAutoIdle1;
            }
       }
    a_AutoState1 = nextState;
}


void Autonomous::NoteTwo() {
    a_AutoState2 = kGoToNote2;   
    state_time = gettime_d();
}

void Autonomous::PeriodicNoteTwo() {

    AutoState2 nextState = a_AutoState2;

    switch (a_AutoState2) {
       case kAutoIdle2:
            StopSwerves();
            break;
         case kGoToNote2:
            a_SwerveDrive -> odometryGoToPose(2.9, -1.45, 0.0);
            nextState = kShootNote2;
            break;
        case kShootNote2:
            a_NoteHandler->indexToShoot();
            nextState = kAutoIdle2;
            break;      
    }
    a_AutoState2 = nextState;
}


void Autonomous::NoteThree() 
{
    a_AutoState3 =   kGoToNote3;
    state_time = gettime_d();
}

void Autonomous::PeriodicNoteThree() {
    AutoState3 nextState = a_AutoState3;
   
    switch (a_AutoState3) {
         case kAutoIdle3:
            StopSwerves();
            break;
        case kGoToNote3:
            a_SwerveDrive -> odometryGoToPose(2.9, 1.45, 0.0);
            nextState = kShootNote3;
            break;
        case kShootNote3:
            a_NoteHandler->indexToShoot();
            nextState = kAutoIdle3;
            break;
    }
    a_AutoState3 = nextState;
}

void Autonomous::NoteFour(){
    a_AutoState4 = kGoToNote4;
    state_time = gettime_d();
}

void Autonomous::PeriodicNoteFour() {

    AutoState4 nextState = a_AutoState4;

    switch (a_AutoState4) {
        case kAutoIdle4:
            StopSwerves();
            break;
        case kGoToNote4:
            a_SwerveDrive -> odometryGoToPose(8.23, -1.68, 0.0);
            nextState = kShootNote4;
            break;
        case kShootNote4:
            a_NoteHandler->indexToShoot();
            nextState = kAutoIdle4;
            break;
    }
    a_AutoState4 = nextState;
 }
 void Autonomous::NoteFive() 
{
    state_time = gettime_d();
    a_AutoState5 = kGoToNote5;
}
    
void Autonomous::PeriodicNoteFive() {

    AutoState5 nextState = a_AutoState5;

    switch (a_AutoState5) {
        case kAutoIdle5:
            StopSwerves();
            break;
        case kGoToNote5:
            a_SwerveDrive -> odometryGoToPose(8.23, -0.2, 0.0);
            nextState = kShootNote5;
            break;
        case kShootNote5:
            a_NoteHandler->indexToShoot();
            nextState = kAutoIdle5;
            break;
        }
    a_AutoState5 = nextState;
}

void Autonomous::NoteSix(){
    state_time = gettime_d();
    a_AutoState6 = kGoToNote6;
}

void Autonomous::PeriodicNoteSix() {
    AutoState6 nextState = a_AutoState6;
   
    switch (a_AutoState6) {
        case kAutoIdle6:
            StopSwerves();
            break;
        case kGoToNote6:
            a_SwerveDrive -> odometryGoToPose(8.23, 1.48, 0.0);
            nextState = kShootNote6;
            break;
        case kShootNote6:
            a_NoteHandler->indexToShoot();
            nextState = kAutoIdle6;
            break;
    }
    a_AutoState6 = nextState;
}

void Autonomous::NoteSeven() 
{
    state_time = gettime_d();
    a_AutoState7 = kGoToNote7;
    drivestart = 0.0;
}

void Autonomous::PeriodicNoteSeven() {
    AutoState7 nextState = a_AutoState7;
   
    switch (a_AutoState7) {
        case kAutoIdle7:
            StopSwerves();
            break;
        case kGoToNote7:
            a_SwerveDrive -> odometryGoToPose(8.23, 2.16, 0.0);
            nextState = kShootNote7;
            break;
        case kShootNote7:
            a_NoteHandler->indexToShoot();
            nextState = kAutoIdle7;
            break;
    }
    a_AutoState7 = nextState;
}

void Autonomous::NoteEight() 
{
    state_time = gettime_d();
    a_AutoState8 = kGoToNote8;
}

void Autonomous::PeriodicNoteEight() {
    AutoState8 nextState = a_AutoState8;
   
    switch (a_AutoState8) {
        case kAutoIdle8:
            StopSwerves();
            break;
        case kGoToNote8:
            a_SwerveDrive -> odometryGoToPose(8.23, 3.84, 0.0);
            nextState = kShootNote8;
            break;
        case kShootNote8:
            a_NoteHandler->indexToShoot();
            nextState = kAutoIdle8;
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

double Autonomous::gettime_d(){
	// return time in seconds as a double
	double t0;
	struct timeval tv0;

	gettimeofday(&tv0, NULL);
	t0 = 1.0 * tv0.tv_sec + (1.0 * tv0.tv_usec) / 1000000.0;
	// printf("seconds: %ld\n", tv0.tv_sec);
	// printf("usecs:   %ld\n", tv0.tv_usec);
	// printf("time:    %lf\n", t0);

	return t0;
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
    float currentTime = gettime_d();
    double tiltAngle = a_Gyro->getPitch() - PITCH_OFFSET;
    double percentTilt = tiltAngle / 15;
    double speed = percentTilt * MAX_FREE_SPEED;
    startedClimb = true;
    if(startedClimb) {
        a_SwerveDrive->driveDirectionVelocity(speed, direction);
        if(abs(tiltAngle) < 5) {
            startTime = gettime_d();
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