#include "Gyro.h"
#include "misc.h"
#include <frc/interfaces/Gyro.h>
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * Constructor.
 *
 * @pa_ram deviceID The CAN Device ID
 */
Gyro::Gyro(int deviceID):
a_PigeonIMU(deviceID) {
    // uint8_t Buff[256];
    lastUpdate = 0;
    Init();
}

Gyro::~Gyro() {} //did nothing in original JrimmyGyro, but it's essential for building

void Gyro::WaitForValues() {
    double start = frc::Timer::GetFPGATimestamp().value();
    double now = start;
    while (now - start > 0.1) {now = frc::Timer::GetFPGATimestamp().value();}
    //the old logic basically checked for a result or it auto-expired after 0.1 seconds. 0.1 seconds is very short, so it just waits 0.1 seconds now
}


void Gyro::Init() {
    lastUpdate = 0;
    Cal();
}

void Gyro::Cal() {

}

void Gyro::Update() {
    frc::SmartDashboard::PutNumber("gyro angle clamped: ", getAngleClamped());

    // Posts a boolean to SmartDashboard to allow a big red or green square to indicate if the robot
    // is aligned with 1 of the 3 angles for the stage chain
    frc::SmartDashboard::PutBoolean("Chain Aligned", IsAlignedWithChain());
}

double Gyro::getAngle() const {
    return getYaw();
}

double Gyro::getAngleClamped() const {
    return misc::clampDegrees(getYaw());
}
double Gyro::getYaw() const {
    return a_PigeonIMU.GetYaw();
}
double Gyro::getPitch() const {
    return a_PigeonIMU.GetPitch();
}
void Gyro::setYaw(double angle){
    a_PigeonIMU.SetYaw(angle);
}
double Gyro::getAbsoluteCompassHeading () const{
    return a_PigeonIMU.GetAbsoluteCompassHeading();
}

void Gyro::Zero(double offsetAngle) { //takes offsetAngle, defaults to zero if none provided. CCW is +
    a_PigeonIMU.SetYaw(0);
}

/**
 * 2024 Crescendo function to return a simple boolean if the robot is aligned with
 * 1 of the 3 angles to hang on the chain for the stage. This cannot tell if the robot
 * is in the right location on the field. It is purely for lining up the angle.
*/
bool Gyro::IsAlignedWithChain() {
    static const double kToleranceDegrees = 4.0;
    static const std::array<double, 3> kChainAngleDegrees = {
        0.0,
        60.0,
        300.0
    };

    double current_angle = getAngleClamped();

    // If the angle of the robot is near 360, shift it down so the simple
    // subtraction and fabs check will handle the tolerance check correctly
    if (current_angle + kToleranceDegrees >= 360) {
        current_angle -= 360.0;
    }

    for (double chain_angle : kChainAngleDegrees) {
        if (fabs(current_angle - chain_angle) < kToleranceDegrees) {
            return true;
        }
    }

    return false;
}