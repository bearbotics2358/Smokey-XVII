#pragma once

#include <Shooter.h>
#include <Collector.h>

class NoteHandler {
    public:
        NoteHandler(Shooter *Shooter, Collector *Collector);

        // Shooter Stuff
        double getShooterAngle();
        void setShooterAngleToDefault();
        void startShooter(double rpm, double angle);
        void stopShooter();
        
        // Collecter Stuff
        void startCollector(double speed);
        void stopCollector();
        void runCollectorBack();

        // Indexer Stuff
        void indexToShoot();
        void indexToAmp();
        void stopIndexer();

        // Miscellaneous
        void stopAll();
        void stopCollection();
        void collectNote(double speed, bool doNotIgnoreBeamBreak);
        void dispenseNote();
        bool beamBroken();

    private:
        Shooter *a_Shooter;
        Collector *a_Collector;
};