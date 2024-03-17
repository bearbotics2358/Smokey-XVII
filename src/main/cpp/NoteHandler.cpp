#include <NoteHandler.h>
#include <frc/smartdashboard/SmartDashboard.h>


// Maybe no : after ()
NoteHandler::NoteHandler(): 
a_Collector(COLLECTOR_MOTOR_ID, INDEXER_MOTOR_ID),
a_Shooter(SHOOTER_LEFT_MOTOR_ID, SHOOTER_RIGHT_MOTOR_ID, PIVOT_MOTOR_ID, SHOOTER_LIMIT_SWITCH_PORT),
a_Climber(CLIMBER_MOTOR_ID, TOP_LIMIT_SWITCH_PORT),
a_AmpTrap(ROLLER_ID, ARM_PIVOT_MOTOR_ID, EXTENSION_ID),
currentAmpLoadState(IDLE)
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
}
bool NoteHandler::armToPose(double angle){
    return a_AmpTrap.moveToPosition(angle);
}
void NoteHandler::setRotPID(double p, double i, double d){
    a_AmpTrap.setPID(p, i, d);
}
void NoteHandler::shootToAmp(bool transferButtonState, bool intoAmpButtonState, bool toDefaultPositionButtonState, bool shooterButtonState) {
    switch(currentAmpLoadState){
        case IDLE:
            released = false;
            if (shooterButtonState) {
            double rpm = 3500;
            double angle = 35.0;
            startShooter(rpm, angle);
            } 
            else {
                stopShooter();
                moveShooterToAngle(0.0);
            }
            // if (a_Gamepad.GetRawButton(3)) {
            //     double rpm = 3500;
            //     double angle = 29.0;
            //     a_NoteHandler.startShooter(rpm, angle);
            // } 
            // else {
            //     a_NoteHandler.stopShooter();
            //     a_NoteHandler.moveShooterToAngle(0.0);
            // }
            a_AmpTrap.moveToPosition(15.0);
            if(transferButtonState == true){
                currentAmpLoadState = LOADING;
            }
            break;

        case LOADING:
            if(!transferButtonState){
                currentAmpLoadState = IDLE;
            }
            a_AmpTrap.runRoller();
            a_Shooter.setSpeed(600);
            if(a_Shooter.moveToAngle(53.0) && a_AmpTrap.moveToPosition(237.0)){
                feedToAmp(-.2);

                if(a_AmpTrap.beamBroken()){
                    shootToAmpMode = true;
                }
                if(shootToAmpMode){
                    if(!a_AmpTrap.beamBroken()){
                        a_AmpTrap.stopRoller();
                        stopAll();
                        shootToAmpMode = false;
                        currentAmpLoadState = HOLDING;
                    }
                }
            }
                break;
            case HOLDING:
                stopShooter();
                moveShooterToAngle(0.0);
                if(intoAmpButtonState){
                    if(armToPose(154.0)){
                        runArmRoller();
                        currentAmpLoadState = DONE;
                    }
                }
                if(toDefaultPositionButtonState){
                    if(armToPose(10.0)){
                        currentAmpLoadState = TOAMP;
                    }
                }
                break;
            case TOAMP:
                stopShooter();
                moveShooterToAngle(0.0);
                if(intoAmpButtonState){
                    if(armToPose(154.0)){
                        runArmRoller();
                        currentAmpLoadState = DONE;
                    }
                }
                break;
            case DONE:
                stopShooter();
                moveShooterToAngle(0.0);
                if(!transferButtonState){
                    currentAmpLoadState = IDLE;
                }
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
    a_Climber.extendClimnber();
}
void NoteHandler::moveShooterToAngle(double angle){
    a_Shooter.moveToAngle(0.0);
}