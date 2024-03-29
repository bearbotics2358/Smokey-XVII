#pragma once

#include <Shooter.h>
#include <Collector.h>
#include <InterpolatingMap.h>
#include <InterpolationValues.h>
#include "Climber.h"
#include "AmpTrap.h"

enum AmpLoadState { // Encoders
            IDLE = 0,
            LOADING,
            HOLDING,
            SCORE,
            TOAMP,
            AWAYFROMAMP,
            DONE,
};
enum ClimbingState { // Encoders
            CLIMBIDLE = 0,
            BRINGCLIMBERSUP,
            TRANSFERNOTE,
            CHECKBEAM,
            EXTENSION,
            CLIMB,
            MOVEARM,
            TRAP,
            HITNOTE,
            DONECLIMBING,
            RESET
};

class NoteHandler {
    public:
        NoteHandler();

        bool released = false;

        // Shooter Stuff
        double getShooterAngle();
        void startShooter(double rpm, double angle);
        void stopShooter();

        // Collecter Stuff
        void startCollector(double speed);
        void stopCollector();
        void runCollectorBack();

        // Indexer Stuff
        void indexToShoot();
        void indexToAmp();
        void indexToCollect();
        void stopIndexer();

        // Miscellaneous
        void stopAll();
        void stopCollection();
        void collectNote(double speed, bool doNotIgnoreBeamBreak);
        void shootNote(double speed);
        
        void dispenseNote();
        bool beamBroken();

        void UpdateSensors();
        void updateDashboard();
        bool shootToAmpMode = false;
        // Interpolation
        InterpolationValues interpolate(double x);
        void insertToInterpolatingMap(double x, InterpolationValues value);

        bool armToPose(double angle);

        void setRotPID(double p, double i, double d);

        void shootToAmp(bool transferButtonState, bool intoAmpButtonState, bool toDefaultPositionButtonState, bool shooterButtonState, bool driverShootNote, bool collectorButton);
        void climbControl(bool climbButton, bool finishClimbButton, bool resetButton);

        void feedToAmp(double speed);

        void runArmRoller(double rps);

        bool moveShooterToAngle(double angle);

        double getClimberPosition();
        void manualClimberUp();
        void manualClimberDown();
        void setClimberPosition();
        void stopClimber();
        void pidClimb();

        void setExtensionPosition();

        void runExtension(double position);

        bool ampBeamBreak();

        bool noteShot = false;

        double state_time = 0.0;

        AmpLoadState currentAmpLoadState;
        ClimbingState currentClimbState;

        bool transferToAmp();
    private:
        Shooter a_Shooter;
        Collector a_Collector;
        InterpolatingMap map;
        Climber a_Climber;
        AmpTrap a_AmpTrap;

        
        

};