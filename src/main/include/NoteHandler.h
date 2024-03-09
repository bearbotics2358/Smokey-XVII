#pragma once

#include <Shooter.h>
#include <Collector.h>
#include <InterpolatingMap.h>
#include <InterpolationValues.h>

class NoteHandler {
    public:
        NoteHandler();

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

        // Interpolation
        InterpolationValues interpolate(double x);
        void insertToInterpolatingMap(double x, InterpolationValues value);

    private:
        Shooter a_Shooter;
        Collector a_Collector;
        InterpolatingMap map;
};