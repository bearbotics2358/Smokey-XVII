

#pragma once

// #include <frc/WPILib.h>
#include "SwerveDrive.h"
#include "Gyro.h"
#include <Prefs.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <units/math.h>
#include <NoteHandler.h>

const std::string RobotDoNothing = "Sit Still";
const std::string firstNote = "Note 1";
const std::string secondNote = "Note 2";
const std::string thirdNote = "Note 3";
const std::string fourthNote = "Note 4";
const std::string fifthNote = "Note 5";
const std::string sixthNote = "Note 6";
const std::string seventhNote = "Note 7";
const std::string eighthNote = "Note 8";


enum AutoState0 { // Do Nothing
    kAutoIdle0

};

enum AutoState1 { // Encoders
    kAutoIdle1,
    kRotateToShoot1,
    kShootFirstNote1,
    kGoToSecondNote1,
    kGoToSpeaker1,
    kShootSecondNote1
};

enum AutoState2 { // T.O.F and Encoders
    kAutoIdle2,
    kShootFirstNote2,
    kGoToSecondNote2,
    kGoToSpeaker2,
    kShootSecondNote2
    };

// states for 3 ball auto
enum AutoState3 {
    kAutoIdle3,
    kRotateToShoot3,
    kShootFirstNote3,
    kGoToSecondNote3,
    kGoToSpeaker3,
    kShootSecondNote3
    };

enum AutoState4 {
    kAutoIdle4,
    kShootFirstNote4,
    kGoToSecondNote4,
    kGoToSpeaker4,
    kShootSecondNote4,
    kGoToThirdNote4,
    kGoToSpeakerAgain4,
    kShootThirdNote4,
    kGoToFourthNote4,
    kGoToSpeakerThirdTime4,
    kShootFourthNote4
};

// states for 5 ball auto
enum AutoState5 {
    kAutoIdle5,
    kRotateToShoot5,
    kShootFirstNote5,
    kGoToSecondNote5,
    kGoToSpeaker5,
    kShootSecondNote5,
    kGoToThirdNote5,
    kGoToSpeakerAgain5,
    kShootThirdNote5
};

enum AutoState6 {
    kAutoIdle6,
    kRotateToShoot6,
    kShootFirstNote6,
    kGoToSecondNote6,
    kGoToSpeaker6,
    kShootSecondNote6,
    kGoToThirdNote6,
    kGoToSpeakerAgain6,
    kShootThirdNote6
};

 enum AutoState7{
    kAutoIdle7,
    kRotateToShoot7,
    kShootFirstNote7,
    kGoToSecondNote7,
    kGoToSpeaker7,
    kShootSecondNote7,
    kGoToThirdNote7,
    kGoToSpeakerAgain7,
    kShootThirdNote7
};

  enum AutoState8{
    kAutoIdle8,
    kRotateToShoot8,
    kShootFirstNote8,
    kGoToSecondNote8,
    kGoToSpeaker8,
    kShootSecondNote8,
    kGoToThirdNote8,
    kGoToSpeakerAgain8,
    kShootThirdNote8

  };

class Autonomous {
    public:
        void StartAuto(const std::string autoMode);
        void PeriodicAuto(const std::string periodicAutoMode);

    void DecidePath();
    Autonomous(Gyro *Gyro, SwerveDrive *SwerveDrive, NoteHandler *NoteHandler);

   // const char *GetCurrentPath();



    void StartAuto();
    void PeriodicAuto();

    void DoNothing();         // Blue Drop and Go Left AutoState0
    void PeriodicDoNothing(); // Periodic Blue Drop and Go Left AutoState0

    void NoteOne();         // Blue Charge Station Left AutoState1
    void PeriodicNoteOne(); // Periodic Blue Charge Station Left AutoState1

    void NoteTwo();         // Blue Drop and Go Middle AutoState2
    void PeriodicNoteTwo(); // Periodic Blue Drop and Go Middle AutoState2

    void NoteThree();         // Blue Charge Station Middle AutoState3
    void PeriodicNoteThree(); // Periodic Blue Charge Station Middle AutoState3

    void NoteFour();         // Blue Drop and Go Right AutoState4
    void PeriodicNoteFour(); // Periodic Blue Drop and Go Right AutoState4

    void NoteFive();         // Blue Charge Station Right AutoState5
    void PeriodicNoteFive(); // Periodic Blue Charge Station Right AutoState5

    void NoteSix();         // Red Drop and Go Left AutoState6
    void PeriodicNoteSix(); // Periodic Red Drop and Go Left AutoState6

    void NoteSeven();         // Red Charge Station Left AutoState7
    void PeriodicNoteSeven(); // Periodic Red Charge Station Left AutoState7

    void NoteEight();         // Red Drop and Go Middle AutoState8
    void PeriodicNoteEight(); // Periodic Red Drop and Go Middle AutoState8

    // ------------------Sub-Routines-------------------------//

    void StopSwerves(); // IDLE

    // Timer System
    // Note: you MUST have a separate case to start the timer, though WaitForTime handles stopping & resetting
    void StartTimer();
    bool WaitForTime(double time); // Wait for specified time in seconds

    // Drives in direction at speed for distance. If going straight backwards, set angle to 180, not dist as a negative
    bool DriveDirection(double dist, double angle, double speed, bool fieldOriented);

    bool TurnToAngle(float angle, bool positive); // turns to a specific angle
    bool Balance(float direction);

private:
    Gyro *a_Gyro;
    SwerveDrive *a_SwerveDrive;
    NoteHandler *a_NoteHandler;

    AutoState0 a_AutoState0;
    AutoState1 a_AutoState1;
    AutoState2 a_AutoState2;
    AutoState3 a_AutoState3;
    AutoState4 a_AutoState4;
    AutoState5 a_AutoState5;
    AutoState6 a_AutoState6;
    AutoState7 a_AutoState7;
    AutoState8 a_AutoState8;



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

    frc::PIDController autoDrivePID;
};