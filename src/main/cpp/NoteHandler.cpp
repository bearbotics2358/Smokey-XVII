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

void NoteHandler::UpdateSensors() {
    a_Shooter.UpdateSensors();
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
void NoteHandler::shootToAmp(bool transferButtonState, bool intoAmpButtonState, bool toDefaultPositionButtonState, bool shooterButtonState, bool driverShootNote, bool collectorButton, bool hooksUp, bool finishClimb, bool ejectCollector, bool ejectFromAmp) {
    frc::SmartDashboard::PutNumber("Current Amp State", currentAmpLoadState);
    switch(currentAmpLoadState){
        case IDLE:
            released = false;
            if (shooterButtonState) {
                double rpm = 2500;
                double angle = 42.5;
                startShooter(rpm, angle);
            } 
            else {
                stopShooter();
                moveShooterToAngle(0.0);
            }

            if (collectorButton) {
                collectNote(-0.4, true);
            }
            else if(ejectCollector){
                dispenseNote();
            } 
            else if (driverShootNote) {
        // give note to shooter
                shootNote(-.65);
            } 
            else {
                stopCollection();
            }
           
            //a_AmpTrap.moveToPosition(7.5);
        
            a_AmpTrap.stopRoller();
            a_AmpTrap.stopArm();
            a_AmpTrap.extendExtender(.03);
            
            if(transferButtonState == true){
                state_time = misc::gettime_d();
                currentAmpLoadState = LOADING;
            }
            else if (hooksUp){
                state_time = misc::gettime_d();
                currentAmpLoadState = CLIMBTRAP;
            }
            
            break;

        case LOADING:
            if(ejectFromAmp){
                if(armToPose(150.0)){
                    state_time = misc::gettime_d();
                    currentAmpLoadState = RESETAMP;
                }
            }
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
            if(ejectFromAmp){
                if(armToPose(150.0)){
                    state_time = misc::gettime_d();
                    currentAmpLoadState = RESETAMP;
                }
            }
            stopShooter();
            moveShooterToAngle(0.0);
            if(intoAmpButtonState){
                if(a_AmpTrap.extendExtender(3.0) && armToPose(135.0)){
                        runArmRoller(-45);
                        state_time = misc::gettime_d();
                        currentAmpLoadState = SCORE;
                }
            }
                break;
        case SCORE:
            if(ejectFromAmp){
                if(armToPose(150.0)){
                    state_time = misc::gettime_d();
                    currentAmpLoadState = RESETAMP;
                }
            }
            armToPose(135.0);
            if(misc::gettime_d() > state_time + 0.5){
                            state_time = misc::gettime_d();
                            a_AmpTrap.stopRoller();
                            currentAmpLoadState = AWAYFROMAMP;
            }
            break;
        case AWAYFROMAMP:
            if(ejectFromAmp){
                if(armToPose(150.0)){
                    state_time = misc::gettime_d();
                    currentAmpLoadState = RESETAMP;
                }
            }
            if(toDefaultPositionButtonState){
                if(a_AmpTrap.moveToPosition(7.5)){
                    state_time = misc::gettime_d();
                    currentAmpLoadState = DONE;
                }
            }
            else{
                a_AmpTrap.moveToPosition(245.0);
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
        case CLIMBTRAP:
            climbControl(hooksUp, finishClimb);
            break;
        case RESETAMP:
            runArmRoller(-45);
            if(misc::gettime_d() > 1.0 + state_time){
                a_AmpTrap.stopRoller();
                if(armToPose(7.5)){
                    state_time = misc::gettime_d();
                    currentAmpLoadState = IDLE;
                }
            }
            break;
    }
}
void NoteHandler::climbControl(bool climbButton, bool finishClimbButton){
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
            if(a_Climber.extendClimnber(2.5)){
                printf("in BRINGCLIMBERSUP\n");
                if(finishClimbButton){
                    state_time = misc::gettime_d();
                    currentClimbState = TRANSFERNOTE;
                }
            }
            break;
        case TRANSFERNOTE:
            if(a_Climber.extendClimnber(4.0)){
                if(transferToAmp()){
                    state_time = misc::gettime_d();
                    currentClimbState = EXTENSION;
                }
            }
            break;
        case EXTENSION:
            // 2.88 in what units?
            if(moveShooterToAngle(1.0)){
                if(a_AmpTrap.extendExtender(15.4)){
                    state_time = misc::gettime_d();
                    currentClimbState = CLIMB;
                }
            }
            break;
        case CLIMB:
            // Climnber???
            a_AmpTrap.extendExtender(15.4);
            if(a_Climber.extendClimnber(18.0)){
                state_time = misc::gettime_d();
                currentClimbState = MOVEARM;
            }
            break;
        case MOVEARM:
            a_Climber.extendClimnber(18.0);
            a_AmpTrap.extendExtender(15.4);
            if(a_AmpTrap.trapMoveToPosition(217.0)){//200
                state_time = misc::gettime_d();
                currentClimbState = TRAP;
            }
            break;
        case TRAP:
            a_Climber.extendClimnber(18.0);
            a_AmpTrap.extendExtender(15.4);
            //a_AmpTrap.moveToPosition(207.0);
            runArmRoller(-40);
            if(misc::gettime_d() > state_time + 0.40){//.5
                state_time = misc::gettime_d();
                currentClimbState = HITNOTE;
            }
            break;
        case HITNOTE:
            a_Climber.extendClimnber(18.0);
            a_AmpTrap.moveToPosition(155.0);
            if(misc::gettime_d() > state_time + 1.0 && a_AmpTrap.moveToPosition(240.0)) {
                state_time = misc::gettime_d();
                currentClimbState = TAP;
            }
            break;
        case TAP:
            a_Climber.extendClimnber(18.0);
            frc::SmartDashboard::PutString("Got to tap", "YES");
            a_AmpTrap.moveToPosition(163.0);
            if(misc::gettime_d() > state_time + .5 && a_AmpTrap.moveToPosition(175.0)) {
                frc::SmartDashboard::PutString("Got into if", "YES");
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
        case RESET:
            if(a_Climber.extendClimnber(4.0)){
                if(a_AmpTrap.extendExtender(1.0)){
                    a_AmpTrap.stopRoller();
                    stopAll();
                    a_AmpTrap.stopArm();
                    a_AmpTrap.stopExtension();
                    a_Climber.stopClimber();
                }
            }
    }
}
void NoteHandler::runArmRoller(double rps){
    a_AmpTrap.runRoller(rps);
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
bool NoteHandler::ampBeamBreak(){
    return a_AmpTrap.beamBroken();
}

bool NoteHandler::transferToAmp() {
    a_AmpTrap.runRoller(-15);
    a_Shooter.setSpeed(700);
    if (a_AmpTrap.moveToPosition(245.0) && a_Shooter.moveToAngle(55.0)) {
        
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