#include <NoteHandler.h>

// Maybe no : after ()
NoteHandler::NoteHandler(Shooter *Shooter, Collector *Collector): 
a_Collector(Collector),
a_Shooter(Shooter)
{
 // NWOTE HWANDLWER
}

// Shooter stuff
double NoteHandler::getShooterAngle() {
    return a_Shooter->GetShooterAngle();
}

void NoteHandler::setShooterAngleToDefault() {
    a_Shooter->setShooterAngle();
}

void NoteHandler::startShooter(double rpm, double angle) {
    a_Shooter->moveToAngle(angle);
    //a_Shooter->setSpeed(rpm);
}

void NoteHandler::stopShooter() {
    a_Shooter->stopShooter();
}

// Collecter stuff
void NoteHandler::startCollector(double speed) {
    a_Collector->startCollector(speed);
}

void NoteHandler::stopCollector() {
    a_Collector->stopCollector();
}

void NoteHandler::runCollectorBack() {
    a_Collector->runCollectorback();
}

// Indexer stuff
void NoteHandler::indexToShoot() {
    a_Collector->indexToShoot();
}

void NoteHandler::indexToAmp() {
    a_Collector->indexToAmp();
}

void NoteHandler::stopIndexer() {
    a_Collector->stopIndexer();
}

// Miscellaneous
void NoteHandler::stopCollection() {
    stopCollector();
    stopIndexer();
}

void NoteHandler::stopAll() {
    stopCollection();
    a_Shooter->stopShooter();
}


void NoteHandler::collectNote(double speed, bool doNotIgnoreBeamBreak) {
    if (doNotIgnoreBeamBreak && beamBroken()) {
        stopCollection();
        return;
    }
    startCollector(speed);
    indexToShoot();
}

void NoteHandler::dispenseNote() {
    a_Collector->indexToAmp();
    runCollectorBack();
}

bool NoteHandler::beamBroken() {
    return a_Collector->beamBreak.beamBroken();
}
