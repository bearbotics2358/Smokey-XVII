#include "Autonomous.h"
#include "buttons.h"
#include "misc.h"
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <sys/time.h>

//left if positive degrees right is negative
Autonomous::Autonomous(Gyro *Gyro, SwerveDrive *SwerveDrive, Claw *Claw, TOF *tof):
a_Gyro(Gyro),
a_SwerveDrive(SwerveDrive),
a_Claw(Claw),
a_TOF(tof){}



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
    else if (autoMode == RedDropAndGoMiddle){
        RDGM();
    }
    else if (autoMode == RedChargeStationMiddle){
        RCSM();
    }
    else if (autoMode == RedDropAndGoRight ){
        RDGR();
    }
    else if (autoMode == RedChargeStationRight){
        RCSR();
    }
    else if (autoMode == RobotDoNothing){
        DoNothing();
    }
    else if(autoMode == LeftTwoPiece){
        LeftPiece2();
    }
    else if(autoMode == RightTwoPiece){
        RightPiece2();
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
    else if (periodicAutoMode == RedDropAndGoMiddle){
        PeriodicRDGM();
    }
    else if (periodicAutoMode == RedChargeStationMiddle){
        PeriodicRCSM();
    }
    else if (periodicAutoMode == RedDropAndGoRight){
        PeriodicRDGR();
    }
    else if (periodicAutoMode == RedChargeStationRight){
        PeriodicRCSR();
    }
    else if (periodicAutoMode == RobotDoNothing){
        PeriodicDoNothing();
    }
    else if(periodicAutoMode == LeftTwoPiece){
        LeftPeriodicPiece2();
    }
    else if(periodicAutoMode == RightTwoPiece){
        RightPeriodicPiece2();
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
            // if (DriveDirection(.45, 180, .25, true)) {
                 nextState = kBlueStartShooter0;
            // }  
            // break;
        case kBlueStartShooter0:
            nextState = kBlueShoot0;
            break;
        case kBlueShoot0:
            nextState = kBlueDriveAway0;

            break;
        case kBlueDriveAway0:
           // if (DriveDirection(3, 160, .25, true)) {
                nextState = kBlueAutoIdle0;
            //}
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
        //     if (DriveDirection(.45, 180, .25, true)) {
                nextState = kBlueStartShooter1;
        //    }  
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
           // if (DriveDirection(1.78, 0, 0.25, true)) {
                nextState = kBlueRotateBack1;
           // }
            break;
        case kBlueRotateBack1:
        //     if(DriveDirection(1.78, 180, 0.25, true)){
                nextState = kBlueGoToAmp1;
          //  }
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
         //   if (DriveDirection(1.94, 0, 0.25, true)) {
                nextState = kBlueAutoIdle2;
        //    }
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
          //  if (DriveDirection(1.94, 0, 0.25, true)) {
            nextState = kBlueGoToSpeaker3;
          //  }
            break;
        case kBlueGoToSpeaker3:
            frc::SmartDashboard::PutNumber("DriveStart", drivestart);
        //    if (DriveDirection(1.94, 180, 0.25, true)) {
                nextState = kBlueRestartShooter3;
        //    }
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
        //    if (DriveDirection(2.0, 0, 0.25, true)) {
                nextState = kBlueAutoIdle4;
        //    }
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
         //   if (DriveDirection(2.0, 0, 0.25, true)) {
                nextState = kBlueGoToSpeaker5;
         //   }
            break;

        case kBlueGoToSpeaker5:
           //  if (DriveDirection(2.0, 180, 0.25, true)) {
                nextState = kBlueTurnBack5;
          //  }
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
       //     if (DriveDirection(1.94, 0, 0.25, true)) {
            nextState = kBlueGoToSpeaker6;
        //    }
            break;
        case kBlueGoToSpeaker6:
     //       if (DriveDirection(1.94, 180, 0.25, true)) {
                nextState = kBlueRestartShooter6;
      //      }
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
     //       if(DriveDirection(2.42, 0, .25, true)) { 
                nextState = kBlueGoBackToSpeaker6;
    //        }
            break;
        case kBlueGoBackToSpeaker6:
     //       if(DriveDirection(2.42, 180, .25, true)) { 
                nextState = kBlueTurnBack6;
      //      }
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
              if(DriveDirection(3, 0.0, 135.0, .25, true)) { 
                 nextState = kRedAutoIdle7;
            }
            break;
        case kRedDrop7:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedRetract7;
        }
            break;
        case kRedRetract7:
            a_Claw->TransformClaw(125, -15, false);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway7;
            }
            break;
        case kRedDriveAway7:
        //    if (DriveDirection(3.5052, 0, 0.3, true)) {
                nextState = kRedGoToStation7;
         //   }
            break;
        case kRedGoToStation7:
         //   if (DriveDirection(1.9812, 90, 0.25, true)) {
                nextState = kRedBalance7;
         //   }
            break; 
        case kRedBalance7:
            if(Balance(180)){
                 nextState = kRedAutoIdle7;
            }
           
            break;
    }
    a_AutoState7 = nextState;
}

void Autonomous::RDGM(){
    state_time = gettime_d();
    a_AutoState8 =  kRedExtend8;
}

void Autonomous::PeriodicRDGM(){
    AutoState8 nextState = a_AutoState8;
   
    switch (a_AutoState8) {
        case kRedAutoIdle8:
            StopSwerves();
            break;
        case kRedExtend8:
           if (TurnToAngle(180, false)) {
                nextState = kRedAutoIdle8;
            }
            break;
        case kRedDrop8:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedRetract8;
        }
            break;
        case kRedRetract8:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway8;
             }
            break;
        case kRedDriveAway8:
     //    if (DriveDirection(4.8768, 0, 0.4, true)) {
                nextState = kRedAutoIdle8;
      //      }
            break;
    }
    a_AutoState8 = nextState;
}


void Autonomous::RCSM() {
    state_time = gettime_d();
    a_AutoState9 = kRedExtend9;
}

void Autonomous::PeriodicRCSM() {
    
    AutoState9 nextState = a_AutoState9;
    
    switch(a_AutoState9){
        case kRedAutoIdle9:
            StopSwerves();
            break;
        case kRedExtend9:
            a_Claw->TransformClaw(170, 510, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDrop9;
            }
            break;
        case kRedDrop9:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedRetract9;
        }
            break;
        case kRedRetract9:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway9;
             }
            break;

        case kRedDriveAway9:
          //  if(DriveDirection(3.5052, 0, .4, true)) { 
                nextState = kRedGoToStation9;
          //  }
            break;
            
        case kRedGoToStation9:
                nextState = kRedBalance9;
            break;
        
        case kRedBalance9:
           if(Balance(180)){
            nextState = kRedAutoIdle9;
           }
            break;
    }   
    a_AutoState9 = nextState;
}

void Autonomous::RDGR(){
    state_time = gettime_d();
    a_AutoState10 = kRedExtend10;
}

void Autonomous::PeriodicRDGR() {

    AutoState10 nextState = a_AutoState10;

    switch (a_AutoState10) {
        case kRedAutoIdle10:
            StopSwerves();
            break;
        case kRedExtend10:
            a_Claw->TransformClaw(170, 510, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDrop10;
            }
            break;
        case kRedDrop10:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();    
            nextState = kRedRetract10;
        }
            break;

        case kRedRetract10:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway10;
             }
            break;
        case kRedDriveAway10:
         //   if (DriveDirection(4.8768, 0, 0.25, true)) {
                nextState = kRedAutoIdle10;
         //   }
            break;
    }
    a_AutoState10 = nextState;
 }

void Autonomous::RCSR() {
    state_time = gettime_d();
    a_AutoState11 = kRedExtend11;
}

void Autonomous::PeriodicRCSR() {
    AutoState11 nextState = a_AutoState11;

    switch(a_AutoState11){
        case kRedAutoIdle11:
            StopSwerves();
            break;
        case kRedExtend11:
            a_Claw->TransformClaw(170, 510, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDrop11;
            }
            break;
        case kRedDrop11:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedRetract11;
        }
            break;
        case kRedRetract11:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway11;
             }
            break;
        case kRedDriveAway11:
       //     if(DriveDirection(3.5052, 0, .25, true)) { 
                nextState = kRedGoToStation11;
       //     }
            break;
        case kRedGoToStation11:
        //    if(DriveDirection(1.9812, -90, .25,true)) { 
                nextState = kRedBalance11;
         //   }
            break;
        case kRedBalance11:
        if(Balance(180)){
                nextState = kRedAutoIdle11;
        } 
            break; 
        
    }
a_AutoState11 = nextState;

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
void Autonomous::LeftPiece2() {
    a_AutoState13 = kExtend13;
}
void Autonomous::LeftPeriodicPiece2(){
    AutoState13 nextState = a_AutoState13;

    switch(a_AutoState13){
        case kAutoIdle13:
            StopSwerves();
            break;
        case kExtend13:
            a_Claw->TransformClaw(170, 510, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            distance = 
            state_time = gettime_d();
            nextState = kDrop13;
            }
            break;
        case kDrop13:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRetract13;
            }
            break;
        case kRetract13:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kDriveAway13;
             }
            break;
        case kDriveAway13:
         //   if(DriveDirection(5.69, 0, .25, true)) { 
                nextState = kTurn13;
         //   }
            break;
        case kTurn13:
            if(TurnToAngle(-90, false)){
                nextState = kPickUp13;
            }
            break;
        case kPickUp13:
        //    if(DriveDirection(3, 90, .25, true)||a_TOF->GetTargetRangeIndicator() == target_range_enum::TARGET_IN_RANGE){
                distance = a_SwerveDrive->getAvgDistance();
                a_Claw->ClawClose();
                nextState = kGoBack13;
        //    }
            break;
        case kGoBack13:
        //TODO subtract the distance we need to get to the cube platform
        //    if(DriveDirection(distance-.47, -90, .25, true)){
                nextState = kTurnBack13;
        //    }
            break;
        case kTurnBack13:
            if(TurnToAngle(90, false)){
                nextState = kGoToGrid13;
            }
            break;
        case kGoToGrid13:
        //    if(DriveDirection(5.69, 180, .25, true)){
                nextState = kAutoIdle13;
         //       }
            
        case kExtendAgain13:
            a_Claw->TransformClaw(170, 510, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kPlace13;
            }
            break;
        case kPlace13:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kAutoIdle13;
            }
            break;
    }
   a_AutoState13 = nextState;
}
void Autonomous::RightPiece2() {
    a_AutoState13 = kExtend13;
}
void Autonomous::RightPeriodicPiece2(){
AutoState14 nextState = a_AutoState14;

    switch(a_AutoState14){
        case kBlueAutoIdle14:
            StopSwerves();
            break;
        case kBlueExtend14:
            a_Claw->TransformClaw(170, 510, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            distance = 
            state_time = gettime_d();
            nextState = kBlueDrop14;
            }
            break;
        case kBlueDrop14:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueRetract14;
            }
            break;
        case kBlueRetract14:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueDriveAway14;
             }
            break;
        case kBlueDriveAway14:
        //    if(DriveDirection(5.69, 0, .25, true)) { 
                nextState = kTurn14;
        //    }
            break;
        case kTurn14:
            if(TurnToAngle(90, false)){
                nextState = kBluePickUp14;
            }
            break;
        case kBluePickUp14:
        //    if(DriveDirection(3, -90, .25, true)||a_TOF->GetTargetRangeIndicator() == target_range_enum::TARGET_IN_RANGE){
                distance = a_SwerveDrive->getAvgDistance();
                a_Claw->ClawClose();
                nextState = kGoBack14;
         //   }
            break;
        case kGoBack14:
        //TODO subtract the distance we need to get to the cube platform
       //     if(DriveDirection(distance-.47, 90, .25, true)){
                nextState = kTurnBack14;
       //     }
            break;
        case kTurnBack14:
            if(TurnToAngle(-90, false)){
                nextState = kGoToGrid14;
            }
            break;
        case kGoToGrid14:
       //     if(DriveDirection(5.69, 180, .25, true)){
                nextState = kBlueAutoIdle14;
         //       }
            
        case kExtendAgain14:
            a_Claw->TransformClaw(170, 510, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kPlace14;
            }
            break;
        case kPlace14:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueAutoIdle14;
            }
            break;
    }
   a_AutoState14 = nextState;
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


bool Autonomous::DriveDirection(double dist, double angleface, double anglerotate, double speed, bool fieldOriented) { // true is done, false is not done
    
    if (fabs(a_SwerveDrive->getAvgDistance()) < (dist + drivestart)) {

        if (a_SwerveDrive->getAvgDistance() > (0.80 * (dist + drivestart))) {
            // for the second part of the move, drive slower
            // after the second part of the move, allow goToTheDon() to slam on the brakes
            a_SwerveDrive->goToTheDon(speed / 2, angleface, anglerotate, dist, fieldOriented);

        } else {
            // first part of the move, at the user specified speed
            // after the first part of the move, do not slam on the brakes
            a_SwerveDrive->goToTheDon(speed, angleface, anglerotate, dist, fieldOriented);
        }
        return false;

    } else {
        drivestart += dist;
        a_SwerveDrive->stop();
        a_SwerveDrive->unsetHoldAngle();

        return true;
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