

#pragma once

// #include <frc/WPILib.h>
#include "SwerveDrive.h"
#include "Gyro.h"
#include <Prefs.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <units/math.h>
#include "Claw.h"
#include "TOF.h"



const std::string onePieceAMP = "One Amp";
const std::string twoPieceAMP = "Two Amp";
const std::string BlueMiddleOneNote = "Blue Middle One Note";
const std::string BlueMiddleTwoNote = "Blue Middle Two Note";
const std::string BlueRightOneNote = "Blue Right One Note";
const std::string BlueRightTwoNote = "Blue Right Two Note";
const std::string RedDropAndGoLeft = "Red Drop and Go Left";
const std::string RedChargeStationLeft = "Red Charge Station Left";
const std::string RedDropAndGoMiddle = "Red Drop and Go Middle";
const std::string RedChargeStationMiddle = "Red Charge Station Middle";
const std::string RedDropAndGoRight = "Red Drop and Go Right";
const std::string RedChargeStationRight = "Red Charge Station Right";
const std::string RobotDoNothing = "Sit Still";
const std::string LeftTwoPiece = "Left 2 Piece";
const std::string RightTwoPiece = "Right 2 Piece";
const std::string kAutoModeDefault = RobotDoNothing;

enum AutoState0 { // Encoders
    kBlueAutoIdle0 = 0,
    kBlueGo0,
    kBlueStartShooter0,
    kBlueShoot0,
    kBlueDriveAway0

};

enum AutoState1 { // Encoders
    kBlueAutoIdle1,
    kBlueGo1,
    kBlueStartShooter1,
    kBlueShoot1,
    kBlueTurn1,
    kBlueGetPiece1,
    kBlueRotateBack1,
    kBlueGoToAmp1,
    kBlueRestartShooter1,
    kBlueShootAgain1
};

enum AutoState2 { // T.O.F and Encoders
    kBlueAutoIdle2,
    kBlueStartShooter2,
    kBlueShoot2,
    kBlueDriveBack2,
};

// states for 3 ball auto
enum AutoState3 {
    kBlueAutoIdle3,
    kBlueStartShooter3,
    kBlueShoot3,
    kBlueGetNote3,
    kBlueGoToSpeaker3,
    kBlueRestartShooter3,
    kBlueShootAgain3
};

enum AutoState4 {
    kBlueAutoIdle4,
    kBlueStartShooter4,
    kBlueShoot4,
    kBlueRotate4,
    kBlueDriveAway4
};

// states for 5 ball auto
enum AutoState5 {
    kBlueAutoIdle5,
    kBlueStartShooter5,
    kBlueShoot5,
    kBlueRotate5,
    kBlueGetPiece5,
    kBlueGoToSpeaker5,
    kBlueTurnBack5,
    kRestartShooter5,
    kShootAgain5
};

enum AutoState6 {
    kBlueAutoIdle6,
    kBlueStartShooter6,
    kBlueShoot6,
    kBlueGetNote6,
    kBlueGoToSpeaker6,
    kBlueRestartShooter6,
    kBlueShootAgain6,
    kBlueTurn6,
    kBlueGetThirdNote6,
    kBlueGoBackToSpeaker6,
    kBlueTurnBack6,
    kBluePrepShooter6,
    kBlueShootThirdNote6
};
 enum AutoState7{
    kRedAutoIdle7,
    kRedExtend7,
    kRedDrop7,
    kRedRetract7,
    kRedDriveAway7,
    kRedGoToStation7,
    kRedBalance7


 };

  enum AutoState8{
    kRedAutoIdle8,
    kRedExtend8,
    kRedDrop8,
    kRedRetract8,
    kRedDriveAway8,

  };

  enum AutoState9{
    kRedAutoIdle9,
    kRedExtend9,
    kRedDrop9,
    kRedRetract9,
    kRedDriveAway9,
    kRedGoToStation9,
    kRedBalance9
  };
  enum AutoState10{
    kRedAutoIdle10,
    kRedExtend10,
    kRedDrop10,
    kRedRetract10,
    kRedDriveAway10


  };
    enum AutoState11{
        kRedAutoIdle11,
        kRedExtend11,
        kRedDrop11,
        kRedRetract11,
        kRedDriveAway11,
        kRedGoToStation11,
        kRedBalance11
    };
    enum AutoState12{
        kIdle
    };
     enum AutoState13{
        kAutoIdle13,
        kExtend13,
        kDrop13,
        kRetract13,
        kDriveAway13,
        kTurn13,
        kPickUp13,
        kGoBack13,
        kTurnBack13,
        kGoToGrid13,
        kExtendAgain13,
        kPlace13,
        kRetractAgain13
    };
    enum AutoState14{
        kBlueAutoIdle14,
        kBlueExtend14,
        kBlueDrop14,
        kBlueRetract14,
        kBlueDriveAway14,
        kTurn14,
        kBluePickUp14,
        kGoBack14,
        kTurnBack14,
        kGoToGrid14,
        kExtendAgain14,
        kPlace14,
    };


class Autonomous {
    public:
        void StartAuto(const std::string autoMode);
        void PeriodicAuto(const std::string periodicAutoMode);

    void DecidePath();
    Autonomous(Gyro *Gyro, SwerveDrive *SwerveDrive, Claw *Claw, TOF *tof);

   // const char *GetCurrentPath();



    void StartAuto();
    void PeriodicAuto();

    void oneAMP();         // Blue Drop and Go Left AutoState0
    void PeriodiconeAMP(); // Periodic Blue Drop and Go Left AutoState0

    void twoAMP();         // Blue Charge Station Left AutoState1
    void PeriodictwoAMP(); // Periodic Blue Charge Station Left AutoState1

    void BMOneNote();         // Blue Drop and Go Middle AutoState2
    void PeriodicBMOneNote(); // Periodic Blue Drop and Go Middle AutoState2

    void BMTwoNote();         // Blue Charge Station Middle AutoState3
    void PeriodicBMTwoNote(); // Periodic Blue Charge Station Middle AutoState3

    void BROneNote();         // Blue Drop and Go Right AutoState4
    void PeriodicBROneNote(); // Periodic Blue Drop and Go Right AutoState4

    void BRTwoNote();         // Blue Charge Station Right AutoState5
    void PeriodicBRTwoNote(); // Periodic Blue Charge Station Right AutoState5

    void RDGL();         // Red Drop and Go Left AutoState6
    void PeriodicRDGL(); // Periodic Red Drop and Go Left AutoState6

    void RCSL();         // Red Charge Station Left AutoState7
    void PeriodicRCSL(); // Periodic Red Charge Station Left AutoState7

    void RDGM();         // Red Drop and Go Middle AutoState8
    void PeriodicRDGM(); // Periodic Red Drop and Go Middle AutoState8

    void RCSM();         // Red Charge Station Middle AutoState9
    void PeriodicRCSM(); // Periodic Red Charge Station Middle AutoState9

    void RDGR();         // Red Drop and Go Right AutoState10
    void PeriodicRDGR(); // Periodic Red Drop and Go Right AutoState10

    void RCSR();         // Red Charge Station Right AutoState11
    void PeriodicRCSR(); // Periodic Red Charge Station Right AutoState11

    void DoNothing();
    void PeriodicDoNothing();

    void LeftPiece2();
    void LeftPeriodicPiece2();

    void RightPiece2();
    void RightPeriodicPiece2();

    // ------------------Sub-Routines-------------------------//

    void StopSwerves(); // IDLE

    // Timer System
    // Note: you MUST have a separate case to start the timer, though WaitForTime handles stopping & resetting
    void StartTimer();
    bool WaitForTime(double time); // Wait for specified time in seconds

    static double gettime_d();

    // Drives in direction at speed for distance. If going straight backwards, set angle to 180, not dist as a negative
    bool DriveDirection(double dist, double angle, double speed, bool fieldOriented);

    bool TurnToAngle(float angle, bool positive); // turns to a specific angle
    bool Balance(float direction);

private:
    Gyro *a_Gyro;
    Claw *a_Claw;
    SwerveDrive *a_SwerveDrive;

    TOF *a_TOF;


    AutoState0 a_AutoState0;
    AutoState1 a_AutoState1;
    AutoState2 a_AutoState2;
    AutoState3 a_AutoState3;
    AutoState4 a_AutoState4;
    AutoState5 a_AutoState5;
    AutoState6 a_AutoState6;
    AutoState7 a_AutoState7;
    AutoState8 a_AutoState8;
    AutoState9 a_AutoState9;
    AutoState10 a_AutoState10;
    AutoState11 a_AutoState11;
    AutoState12 a_AutoState12;
    AutoState13 a_AutoState13;
    AutoState14 a_AutoState14;



    std::string a_AutoSelected;
    std::string a_PeriodicAutoSelected;
    float drivestart{0.0};

    double distance = 0;
    double speed = .15;

    // Used to measure time duration in Autonomous states
    double state_time = 0.0;

    // used for waitForTime method
    double waitTimeStart{0.0};

    // TEMP
    double autoStartTime{0.0};
    // TEMP
    double autoScale{1.0};

    bool startedClimb{false};
    float startTime{0.0};
};