#include "Autonomous.h"
#include "buttons.h"
#include "misc.h"
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <sys/time.h>

//left if positive degrees right is negative
Autonomous::Autonomous(Gyro *Gyro, SwerveDrive *SwerveDrive, TOF *tof):
a_Gyro(Gyro),
a_SwerveDrive(SwerveDrive),

a_TOF(tof),
autoDrivePID(.4, .1, 0) 
{}



//-------------------------------------Auto Stuff---------------------------------------------//

void Autonomous::StartAuto(const std::string autoMode) {
    if(autoMode == onePieceAMP){
        oneAMP();
    }
    else if (autoMode == twoPieceAMP){
        twoAMP();
    }
    else if (autoMode == BlueMiddleOneNote){
        BMOneNote();
    }
    else if (autoMode == BlueMiddleTwoNote){
        BMTwoNote();
    }
    else if (autoMode == BlueRightOneNote){
        BROneNote();
    }
    else if (autoMode == BlueRightTwoNote){
        BRTwoNote();
    }
    else if (autoMode == RedDropAndGoLeft){
        RDGL();
    }
    else if (autoMode ==  RedChargeStationLeft){
        RCSL();
    }
    else if (autoMode == RobotDoNothing){
        DoNothing();
    }

    a_AutoSelected = autoMode; 
    
}
void Autonomous::PeriodicAuto(const std::string periodicAutoMode) {
    if(periodicAutoMode == onePieceAMP){
        PeriodiconeAMP();
    }
    else if (periodicAutoMode == twoPieceAMP){
        PeriodictwoAMP();
    }
    else if (periodicAutoMode == BlueMiddleOneNote){
        PeriodicBMOneNote();
    }
    else if (periodicAutoMode ==  BlueMiddleTwoNote){
        PeriodicBMTwoNote();
    }
    else if (periodicAutoMode == BlueRightOneNote){
        PeriodicBROneNote();
    }
    else if (periodicAutoMode == BlueRightTwoNote){
        PeriodicBRTwoNote();
    }
    else if (periodicAutoMode == RedDropAndGoLeft){
        PeriodicRDGL();
    }
    else if (periodicAutoMode == RedChargeStationLeft){
        PeriodicRCSL();
    }
    else if (periodicAutoMode == RobotDoNothing){
        PeriodicDoNothing();
    }
   

    a_PeriodicAutoSelected = periodicAutoMode; 
    
}




// s----------------------------------AUTONOMOUS ROUTINES---------------------------------------- //

void Autonomous::oneAMP() {
    a_AutoState0 = kBlueGo0;
    drivestart = 0.0;

    a_Gyro->setYaw(270);
    // reset state time
    state_time = gettime_d();
}

void Autonomous::PeriodiconeAMP() {
    
    AutoState0 nextState = a_AutoState0;

    switch (a_AutoState0) {
        case kBlueAutoIdle0:
            StopSwerves();
            break;
         case kBlueGo0:
            if (DriveDirection(.45, 180, .25, true)) {
                nextState = kBlueStartShooter0;
            }  
            break;
        case kBlueStartShooter0:
            nextState = kBlueShoot0;
            break;
        case kBlueShoot0:
            nextState = kBlueDriveAway0;

            break;
        case kBlueDriveAway0:
            if (DriveDirection(3, 160, .25, true)) {
                nextState = kBlueAutoIdle0;
            }
            break;
    }
    a_AutoState0 = nextState;
}


void Autonomous::twoAMP() {
    a_Gyro->setYaw(270);
    a_AutoState1 = kBlueGo1;
    drivestart = 0.0;

    state_time = gettime_d();
}

void Autonomous::PeriodictwoAMP() {

    AutoState1 nextState = a_AutoState1;

    switch (a_AutoState1) {
         case kBlueAutoIdle1:
            StopSwerves();
            break;
        case kBlueGo1:
             if (DriveDirection(.45, 180, .25, true)) {
                nextState = kBlueStartShooter1;
            }  
            break;
        case kBlueStartShooter1:
            nextState = kBlueShoot1;
            break;
        case kBlueShoot1:
            nextState = kBlueTurn1;
            break;
        case kBlueTurn1:
            if(TurnToAngle(325, false)){
                nextState = kBlueGetPiece1;
            }
            break;
        case kBlueGetPiece1:
            if (DriveDirection(1.78, 0, 0.25, true)) {
                nextState = kBlueRotateBack1;
            }
            break;
        case kBlueRotateBack1:
             if(DriveDirection(1.78, 180, 0.25, true)){
                nextState = kBlueGoToAmp1;
            }
            break;
        case kBlueGoToAmp1:
            if (TurnToAngle(270, true)) {
                nextState = kBlueRestartShooter1;
            }
            break;
        case kBlueRestartShooter1:
            nextState = kBlueShootAgain1;     
            break;
        case kBlueShootAgain1:
            nextState = kBlueAutoIdle1;
            break;

       }
    a_AutoState1 = nextState;
}

void Autonomous::BMOneNote() {
    a_AutoState2 = kBlueStartShooter2;   
    state_time = gettime_d();
    drivestart = 0.0;

}

void Autonomous::PeriodicBMOneNote() {

    AutoState2 nextState = a_AutoState2;

    switch (a_AutoState2) {
       case kBlueAutoIdle2:
            StopSwerves();
            break;
         case kBlueStartShooter2:
            nextState = kBlueShoot2;
            break;
        case kBlueShoot2:
            nextState = kBlueDriveBack2;
            break;
        case kBlueDriveBack2:
            if (DriveDirection(1.94, 0, 0.25, true)) {
                nextState = kBlueAutoIdle2;
            }
            break;        
    }
    a_AutoState2 = nextState;
}


void Autonomous::BMTwoNote() 
{
    drivestart = 0.0;
    a_AutoState3 =   kBlueStartShooter3;

    state_time = gettime_d();
}

void Autonomous::PeriodicBMTwoNote() {
    AutoState3 nextState = a_AutoState3;
   
    switch (a_AutoState3) {
         case kBlueAutoIdle3:
            StopSwerves();
            frc::SmartDashboard::PutNumber("DriveStart", drivestart);
            break;
        case kBlueStartShooter3:
            nextState = kBlueShoot3;
            frc::SmartDashboard::PutNumber("DriveStart", drivestart);
            break;
        case kBlueShoot3:
            nextState = kBlueGetNote3;
            frc::SmartDashboard::PutNumber("DriveStart", drivestart);
            break;
        case kBlueGetNote3:
        frc::SmartDashboard::PutNumber("DriveStart", drivestart);
            if (DriveDirection(1.94, 0, 0.25, true)) {
            nextState = kBlueGoToSpeaker3;
            }
            break;
        case kBlueGoToSpeaker3:
            frc::SmartDashboard::PutNumber("DriveStart", drivestart);
            if (DriveDirection(1.94, 180, 0.25, true)) {
                nextState = kBlueRestartShooter3;
            }
            break;
        case kBlueRestartShooter3:
            frc::SmartDashboard::PutNumber("DriveStart", drivestart);
            nextState = kBlueShootAgain3;
            break; 
        case kBlueShootAgain3:
            nextState = kBlueAutoIdle3;
            break;
    }
    a_AutoState3 = nextState;
}

void Autonomous::BROneNote(){
    a_AutoState4 = kBlueStartShooter4;
    state_time = gettime_d();
    a_Gyro->setYaw(300);
    drivestart = 0.0;

}

void Autonomous::PeriodicBROneNote() {

    AutoState4 nextState = a_AutoState4;

    switch (a_AutoState4) {
         case kBlueAutoIdle4:
            StopSwerves();
            break;
         case kBlueStartShooter4:
            nextState = kBlueShoot4;
            break;
        case kBlueShoot4:
            nextState = kBlueRotate4;
            break;
        case kBlueRotate4:
            if (TurnToAngle(0.0, false)) {
                nextState = kBlueDriveAway4;
            }
            break;
        case kBlueDriveAway4:
            if (DriveDirection(2.0, 0, 0.25, true)) {
                nextState = kBlueAutoIdle4;
            }
            break;
    }
    a_AutoState4 = nextState;
 }
 void Autonomous::BRTwoNote() 
{
    state_time = gettime_d();
    a_AutoState5 = kBlueStartShooter5;
    a_Gyro->setYaw(300);
    drivestart = 0.0;

}
    
void Autonomous::PeriodicBRTwoNote() {

    AutoState5 nextState = a_AutoState5;

    switch (a_AutoState5) {
        case kBlueAutoIdle5:
            StopSwerves();
            break;
         case kBlueStartShooter5:
            nextState = kBlueShoot5;
            break;
        case kBlueShoot5:
            nextState = kBlueRotate5;
            break;
        case kBlueRotate5:
            if (TurnToAngle(0.0, false)) {
                nextState = kBlueGetPiece5;
            }
            break;

        case kBlueGetPiece5:
            if (DriveDirection(2.0, 0, 0.25, true)) {
                nextState = kBlueGoToSpeaker5;
            }
            break;

        case kBlueGoToSpeaker5:
             if (DriveDirection(2.0, 180, 0.25, true)) {
                nextState = kBlueTurnBack5;
            }
            break;
        case kBlueTurnBack5:
            if (TurnToAngle(300, true)) {
                nextState = kRestartShooter5;
            }
            break;
        case kRestartShooter5:
                nextState = kShootAgain5;
            break;
        case kShootAgain5:
                nextState = kBlueAutoIdle5;
            break;    
    }
    a_AutoState5 = nextState;
}

void Autonomous::RDGL(){
    state_time = gettime_d();
    a_AutoState6 = kBlueStartShooter6;
    drivestart = 0.0;
    
}

void Autonomous::PeriodicRDGL() {
    AutoState6 nextState = a_AutoState6;
   
    switch (a_AutoState6) {
         case kBlueAutoIdle6:
            StopSwerves();
            break;
        case kBlueStartShooter6:
            nextState = kBlueShoot6;
            break;
        case kBlueShoot6:
            nextState = kBlueGetNote6;
            break;
        case kBlueGetNote6:
            if (DriveDirection(1.94, 0, 0.25, true)) {
            nextState = kBlueGoToSpeaker6;
            }
            break;
        case kBlueGoToSpeaker6:
            if (DriveDirection(1.94, 180, 0.25, true)) {
                nextState = kBlueRestartShooter6;
            }
            break;
        case kBlueRestartShooter6:
            nextState = kBlueShootAgain6;
            break; 
        case kBlueShootAgain6:
            nextState = kBlueTurn6;
            break;
        case kBlueTurn6:
            if (TurnToAngle(36.77, false)) {
            nextState = kBlueGetThirdNote6;
            }
            break;
        case kBlueGetThirdNote6:
            if(DriveDirection(2.42, 0, .25, true)) { 
                nextState = kBlueGoBackToSpeaker6;
            }
            break;
        case kBlueGoBackToSpeaker6:
            if(DriveDirection(2.42, 180, .25, true)) { 
                nextState = kBlueTurnBack6;
            }
            break;
        case kBlueTurnBack6:
            if (TurnToAngle(0, true)) {
            nextState = kBluePrepShooter6;
            }
            break;
        case kBluePrepShooter6:
                nextState = kBlueShootThirdNote6;
        case kBlueShootThirdNote6:
                nextState = kBlueAutoIdle6;
        
    }
    a_AutoState6 = nextState;
}

void Autonomous::RCSL() 
{
    state_time = gettime_d();
    a_AutoState7 = kRedExtend7;
    drivestart = 0.0;
}

void Autonomous::PeriodicRCSL() {
    AutoState7 nextState = a_AutoState7;
   
    switch (a_AutoState7) {
        case kRedAutoIdle7:
            StopSwerves();
            break;
        case kRedExtend7:
              if(DriveDirection(1, 0, .25, true)) { 
                 nextState = kRedAutoIdle7;
            }
           
            break;
        case kRedDrop7:
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedRetract7;
        }
            break;
        case kRedRetract7:
            
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway7;
            }
            break;
        case kRedDriveAway7:
            if (DriveDirection(3.5052, 0, 0.3, true)) {
                nextState = kRedGoToStation7;
            }
            break;
        case kRedGoToStation7:
            if (DriveDirection(1.9812, 90, 0.25, true)) {
                nextState = kRedBalance7;
            }
            break; 
        case kRedBalance7:
            if(Balance(180)){
                 nextState = kRedAutoIdle7;
            }
           
            break;
    }
    a_AutoState7 = nextState;
}

void Autonomous::DoNothing() {
    a_AutoState12 = kIdle;
}

void Autonomous::PeriodicDoNothing() {
    AutoState12 nextState = a_AutoState12;

    switch(a_AutoState12){
        case kIdle:
            StopSwerves();
            break;
    }
a_AutoState12 = nextState;

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
        frc::SmartDashboard::PutNumber("calcspeed", calcspeed);
        calcspeed = std::clamp(speed, -.25, .25);
        a_SwerveDrive->driveDirection(calcspeed, angle);    
        if(autoDrivePID.AtSetpoint()){
            // a_SwerveDrive->stop();
            return true;
        }
        else{
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