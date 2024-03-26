#include <NoteHandler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "misc.h"


// Maybe no : after ()
NoteHandler::NoteHandler(): 
a_Collector(COLLECTOR_MOTOR_ID, INDEXER_MOTOR_ID),
a_Shooter(SHOOTER_LEFT_MOTOR_ID, SHOOTER_RIGHT_MOTOR_ID, PIVOT_MOTOR_ID, SHOOTER_LIMIT_SWITCH_PORT),
a_Climber(CLIMBER_MOTOR_ID, TOP_LIMIT_SWITCH_PORT),
a_AmpTrap(ROLLER_ID, ARM_PIVOT_MOTOR_ID, EXTENSION_ID),
currentAmpLoadState(IDLE),
currentClimbState(CLIMBIDLE)
{
 // NWOTE HWANDLWER
}

// Shooter stuff
double NoteHandler::getShooterAngle() {
    return a_Shooter.GetShooterAngle();
}

void NoteHandler::setShooterAngleToDefault() {
    a_Shooter.setShooterAngle();
}

void NoteHandler::startShooter(double rpm, double angle) {
    a_Shooter.moveToAngle(angle);
    a_Shooter.setSpeed(rpm);
}

void NoteHandler::stopShooter() {
    a_Shooter.stopShooter();
}

// Collecter stuff
void NoteHandler::startCollector(double speed) {
    a_Collector.startCollector(speed);
}

void NoteHandler::stopCollector() {
    a_Collector.stopCollector();
}

void NoteHandler::runCollectorBack() {
    a_Collector.runCollectorback();
}

// Indexer stuff
void NoteHandler::indexToShoot() {
    a_Collector.indexToShoot();
}

void NoteHandler::indexToAmp() {
    a_Collector.indexToAmp();
}
void NoteHandler::indexToCollect(){
    a_Collector.indexToCollect();
}
void NoteHandler::stopIndexer() {
    a_Collector.stopIndexer();
}

// Miscellaneous
void NoteHandler::stopCollection() {
    stopCollector();
    stopIndexer();
}

void NoteHandler::stopAll() {
    stopCollection();
    a_Shooter.stopShooter();
}

void NoteHandler::collectNote(double speed, bool doNotIgnoreBeamBreak) {
    if (doNotIgnoreBeamBreak && beamBroken()) {
        stopCollection();
        return;
    }

    startCollector(speed);
    frc::SmartDashboard::PutString("got to indexToCollect()", "YES");
    indexToCollect();
}
void NoteHandler::shootNote(double speed){
    startCollector(speed);
    indexToShoot();
}
void NoteHandler::feedToAmp(double speed){
    startCollector(speed);
    indexToAmp();
}
void NoteHandler::dispenseNote() {
    indexToAmp();
    runCollectorBack();
}

bool NoteHandler::beamBroken() {
    return a_Collector.beamBreak.beamBroken();
}

// Interpolation
InterpolationValues NoteHandler::interpolate(double x) {
    return map[x];
}

void NoteHandler::insertToInterpolatingMap(double x, InterpolationValues value) {
    map.insert(x, value);
}
void NoteHandler::updateDashboard(){
    frc::SmartDashboard::PutBoolean("AmpTrap BeamBreak", a_AmpTrap.beamBroken());
    frc::SmartDashboard::PutBoolean("Indexer BeamBreak", a_Collector.beamBroken());
    frc::SmartDashboard::PutNumber("Arm Angle", a_AmpTrap.GetArmAngle());
    frc::SmartDashboard::PutNumber("Extension Position", a_AmpTrap.GetExtensionPosition());
}
bool NoteHandler::armToPose(double angle){
    return a_AmpTrap.moveToPosition(angle);
}
void NoteHandler::setRotPID(double p, double i, double d){
    a_AmpTrap.setPID(p, i, d);
}
void NoteHandler::shootToAmp(bool transferButtonState, bool intoAmpButtonState, bool toDefaultPositionButtonState, bool shooterButtonState, bool driverShootNote, bool collectorButton) {
    switch(currentAmpLoadState){
        case IDLE:
            released = false;
            a_AmpTrap.stopRoller();
            if (shooterButtonState) {
                double rpm = 3500;
                double angle = 40.0;
                startShooter(rpm, angle);
            } 
            else {
                stopShooter();
                moveShooterToAngle(0.0);
            }
            if (collectorButton) {
                collectNote(-0.4, true);
            } 
            else if (driverShootNote) {
        // give note to shooter
                shootNote(-.65);
            } 
            else {
                stopCollection();
            }
           
            //a_AmpTrap.moveToPosition(7.5);
            a_AmpTrap.stopArm();
            a_AmpTrap.extendExtender(.03);
            if(transferButtonState == true){
                state_time = misc::gettime_d();
                currentAmpLoadState = LOADING;
            }
            break;

        case LOADING:
            if(!transferButtonState){
                state_time = misc::gettime_d();
                currentAmpLoadState = IDLE;
            }
            if(transferToAmp()) {
                state_time = misc::gettime_d();
                currentAmpLoadState = HOLDING;
            }
           break;
        case HOLDING:
            stopShooter();
            moveShooterToAngle(0.0);
            if(intoAmpButtonState){
                if(a_AmpTrap.extendExtender(2.88) && armToPose(125.0)){
                        runArmRoller();
                        state_time = misc::gettime_d();
                        currentAmpLoadState = SCORE;
                }
            }
                break;
        case SCORE:
            armToPose(125.0);
            if(misc::gettime_d() > state_time + 0.5){
                            state_time = misc::gettime_d();
                            a_AmpTrap.stopRoller();
                            currentAmpLoadState = AWAYFROMAMP;
            }
            break;
        case AWAYFROMAMP:
            a_AmpTrap.moveToPosition(245.0);
            if(toDefaultPositionButtonState){
                if(a_AmpTrap.moveToPosition(7.5)){
                    state_time = misc::gettime_d();
                    currentAmpLoadState = DONE;
                }

            }
            break;
        case DONE:
            stopShooter();
            a_AmpTrap.stopRoller();
            moveShooterToAngle(10.0);
            if(!transferButtonState){
                state_time = misc::gettime_d();
                currentAmpLoadState = IDLE;
            }
            break;
    }
}
void NoteHandler::climbControl(bool climbButton){
    frc::SmartDashboard::PutNumber("Current Climb State", currentClimbState);
    switch(currentClimbState){
        case CLIMBIDLE:
            if(climbButton){
                printf("in CLIMBIDLE\n");
                state_time = misc::gettime_d();
                currentClimbState = BRINGCLIMBERSUP;
            }
            break;
        case BRINGCLIMBERSUP:
            if(a_Climber.extendClimnber(4.0)){
                printf("in BRINGCLIMBERSUP\n");
                state_time = misc::gettime_d();
                currentClimbState = TRANSFERNOTE;
            }
            break;
        case TRANSFERNOTE:
            if(transferToAmp()){
                state_time = misc::gettime_d();
                currentClimbState = EXTENSION;
            }
            break;
        case EXTENSION:
            // 2.88 in what units?
            if(a_AmpTrap.extendExtender(15.75)){
                state_time = misc::gettime_d();
                currentClimbState = CLIMB;
            }
            break;
        case CLIMB:
            // Climnber???
            a_AmpTrap.extendExtender(15.75);
            if(a_Climber.extendClimnber(18.0)){
                state_time = misc::gettime_d();
                currentClimbState = MOVEARM;
            }
            break;
        case MOVEARM:
            a_AmpTrap.extendExtender(15.75);
            if(a_AmpTrap.moveToPosition(240.0)){
                state_time = misc::gettime_d();
                currentClimbState = TRAP;
            }
            break;
        case TRAP:
            a_AmpTrap.extendExtender(15.75);
            a_AmpTrap.moveToPosition(240.0);
            runArmRoller();
            if(misc::gettime_d() > state_time + 0.5){
                state_time = misc::gettime_d();
                currentClimbState = HITNOTE;
            }
            break;
        case HITNOTE:
            a_AmpTrap.moveToPosition(163.0);
            if(misc::gettime_d() > state_time + 1.0 && a_AmpTrap.moveToPosition(240.0)) {
                state_time = misc::gettime_d();
                currentClimbState = DONECLIMBING;
            }
            break;
        case DONECLIMBING:
            a_AmpTrap.stopRoller();
            stopAll();
            a_AmpTrap.stopArm();
            a_AmpTrap.stopExtension();
            a_Climber.stopClimber();
            break;
    }
}
void NoteHandler::runArmRoller(){
    a_AmpTrap.runRoller();
}
double NoteHandler::getClimberPosition(){
    return a_Climber.GetClimberPosition();
}
void NoteHandler::manualClimberUp(){
    a_Climber.runClimberUp();
}
void NoteHandler::manualClimberDown(){
    a_Climber.runClimberDown();
}
void NoteHandler::setClimberPosition(){
    a_Climber.setPosition();
}
void NoteHandler::stopClimber(){
    a_Climber.stopClimber();
}
void NoteHandler::pidClimb(){
    a_Climber.extendClimnber(18.0);
}
bool NoteHandler::moveShooterToAngle(double angle){
    return a_Shooter.moveToAngle(0.0);
}
void NoteHandler::setExtensionPosition(){
    a_AmpTrap.setPosition();
}
void NoteHandler::runExtension(double position){
    a_AmpTrap.extendExtender(position);
}

bool NoteHandler::transferToAmp() {
    a_AmpTrap.runRoller();
    a_Shooter.setSpeed(750);
    if (a_AmpTrap.moveToPosition(245.0) && a_Shooter.moveToAngle(54.0)) {
        
        shootNote(-.2);
        if (a_AmpTrap.beamBroken()) {
            state_time = misc::gettime_d();
            shootToAmpMode = true;
        }
        if (shootToAmpMode) {
            if (!a_AmpTrap.beamBroken()) {
                a_AmpTrap.stopRoller();
                stopAll();
                a_Shooter.stopShooter();
                shootToAmpMode = false;
                return true;
            }
        }
        
        
    }
    return false;
}