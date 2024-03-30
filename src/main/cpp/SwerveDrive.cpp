#include "SwerveDrive.h"

#include "misc.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

SwerveDrive::SwerveDrive(SwerveModule& flModule, SwerveModule& frModule, SwerveModule& blModule, SwerveModule& brModule, Gyro& gyro):
flModule(flModule),
frModule(frModule),
blModule(blModule),
brModule(brModule),
a_gyro(gyro),
turnAnglePid(0.014, 0.0, 0.0),
crabAnglePid(1.5, 0.0, 0.01),
a_odometry{a_kinematics, frc::Rotation2d(units::radian_t(a_gyro.getAngleClamped()*((2*M_PI)/360.0))), getModulePositions()},
xProfiledPid(.5, .2, 0.0, linearConstraints),
yProfiledPid(.5, .2, 0.0, linearConstraints),
rotProfiledPid(.5, 0.1, 0.0, rotationalConstraints),
poseEstimator{
    a_kinematics, 
    frc::Rotation2d(units::radian_t(a_gyro.getAngleClamped()*((2*M_PI)/360.0))),
    getModulePositions(),
    frc::Pose2d(units::meter_t(0.0), units::meter_t(0.0), units::radian_t(0.0))
}
{
    xProfiledPid.SetTolerance(units::meter_t(.1));
    yProfiledPid.SetTolerance(units::meter_t(.1));
    rotProfiledPid.SetTolerance(units::radian_t(.087));

    rotProfiledPid.EnableContinuousInput(0_rad, 6.28_rad);

    turnAnglePid.EnableContinuousInput(0.0, 360.0);
    crabAnglePid.EnableContinuousInput(0.0, 360.0);
}

void SwerveDrive::crabUpdate(float x, float y, bool fieldOriented) {
    float gyroDegrees = a_gyro.getAngleClamped();

    if (!crab) {
        holdAngle = gyroDegrees;
        crab = true;
    }


    swerveUpdateInner(x, y, -1*crabCalcZ(holdAngle, gyroDegrees), gyroDegrees, fieldOriented);


}

void SwerveDrive::swerveUpdate(float x, float y, float z, bool fieldOriented) {
    crab = false;
    swerveUpdateInner(x, y, z, a_gyro.getAngleClamped(), fieldOriented);
}

void SwerveDrive::stop() {
    flModule.setDrivePercent(0.0);
    frModule.setDrivePercent(0.0);
    blModule.setDrivePercent(0.0);
    brModule.setDrivePercent(0.0);

    flModule.setSteerPercent(0.0);
    frModule.setSteerPercent(0.0);
    blModule.setSteerPercent(0.0);
    brModule.setSteerPercent(0.0);
}

void SwerveDrive::brakeOnStop() {
    flModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    frModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    blModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    brModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void SwerveDrive::coastOnStop() {
    flModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    frModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    blModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    brModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void SwerveDrive::setHoldAngle(float degrees) {
    crab = true;
    holdAngle = degrees;
}

void SwerveDrive::unsetHoldAngle() {
    crab = false;
}

void SwerveDrive::resetDrive() {
    flModule.resetDriveEncoder();
    frModule.resetDriveEncoder();
    blModule.resetDriveEncoder();
    brModule.resetDriveEncoder();

    // flModule.resetSteerEncoder();
    // frModule.resetSteerEncoder();
    // blModule.resetSteerEncoder();
    // brModule.resetSteerEncoder();
}

void SwerveDrive::driveDistance(float distMeters, float directionDegrees) {
    flModule.steerToAng(directionDegrees);
    frModule.steerToAng(directionDegrees);
    blModule.steerToAng(directionDegrees);
    brModule.steerToAng(directionDegrees);

    flModule.goToPosition(distMeters);
    frModule.goToPosition(distMeters);
    blModule.goToPosition(distMeters);
    brModule.goToPosition(distMeters);
}

void SwerveDrive::driveDirection(float percent, float directionDegrees) {
    // if (flModule.adjustAngle(directionDegrees)) {
    //     flModule.setDrivePercent(-percent);
    // } else {
    //     flModule.setDrivePercent(percent);
    // }

    // if (frModule.adjustAngle(directionDegrees)) {
    //     frModule.setDrivePercent(-percent);
    // } else {
    //     frModule.setDrivePercent(percent);
    // }

    // if (blModule.adjustAngle(directionDegrees)) {
    //     blModule.setDrivePercent(-percent);
    // } else {
    //     blModule.setDrivePercent(percent);
    // }

    // if (brModule.adjustAngle(directionDegrees)) {
    //     brModule.setDrivePercent(-percent);
    // } else {
    //     brModule.setDrivePercent(percent);
    // }
    flModule.steerToAng(directionDegrees);
    frModule.steerToAng(directionDegrees);
    blModule.steerToAng(directionDegrees);
    brModule.steerToAng(directionDegrees);

    flModule.setDrivePercent(percent);
    frModule.setDrivePercent(percent);
    blModule.setDrivePercent(percent);
    brModule.setDrivePercent(percent);
}
//exists solely for autonomous balance
void SwerveDrive::driveDirectionVelocity(float speed, float directionDegrees) {
    flModule.steerToAng(directionDegrees);
    frModule.steerToAng(directionDegrees);
    blModule.steerToAng(directionDegrees);
    brModule.steerToAng(directionDegrees);

    flModule.setDriveSpeed(speed);
    frModule.setDriveSpeed(speed);
    blModule.setDriveSpeed(speed);
    brModule.setDriveSpeed(speed);
}

float SwerveDrive::getAvgDistance() {
  //  return (fabs(flModule.getDistance()) + fabs(frModule.getDistance()) + fabs(blModule.getDistance()) + fabs(brModule.getDistance())) / 4.0;
  return (fabs(flModule.getDistance()) + fabs(frModule.getDistance()) + fabs(blModule.getDistance()) + fabs(brModule.getDistance())) / 4.0;
}
float SwerveDrive::getAvgVelocity() {
  //  return (fabs(flModule.getDistance()) + fabs(frModule.getDistance()) + fabs(blModule.getDistance()) + fabs(brModule.getDistance())) / 4.0;
  return (fabs(flModule.getVelocity()) + fabs(frModule.getVelocity()) + fabs(blModule.getVelocity()) + fabs(brModule.getVelocity())) / 4.0;
}
bool SwerveDrive::turnToAngle(float angle, bool positive_speed) {
        float gyroDegrees = a_gyro.getAngleClamped();
    // calculates a speed we need to go based off our current sensor and target position
            turnAnglePid.SetSetpoint(angle);
            float speed = turnCalcZ(angle, gyroDegrees);
            flModule.steerToAng(135);
            frModule.steerToAng(45);
            blModule.steerToAng(225);
            brModule.steerToAng(315);

            // - speed works, and speed does not because pid has clockwise as going up from zero, while the gyro thinks going clockwise as down from 360, so we need the opposite of one of them
            flModule.setDrivePercent(-speed);
            frModule.setDrivePercent(-speed);
            blModule.setDrivePercent(-speed);
            brModule.setDrivePercent(-speed);
        if((turnAnglePid.AtSetpoint())){
            return true;
        }
        else{
           
            return false;
        }
       

    // if(positive_speed){
    //      flModule.steerToAng(135);
    //      frModule.steerToAng(45);
    //      blModule.steerToAng(225);
    //      brModule.steerToAng(315);

    //      flModule.setDrivePercent(.15);
    //      frModule.setDrivePercent(.15);
    //      blModule.setDrivePercent(.15);
    //      brModule.setDrivePercent(.15);
    // }
    // else{
    //     flModule.steerToAng(135);
    //     frModule.steerToAng(45);
    //     blModule.steerToAng(225);
    //     brModule.steerToAng(315);

    //     flModule.setDrivePercent(-.15);
    //     frModule.setDrivePercent(-.15);
    //     blModule.setDrivePercent(-.15);
    //     brModule.setDrivePercent(-.15);
    // }
    

}


void SwerveDrive::goToTheDon(float speed, float direction, float distance, bool fieldOriented, bool stop_on_completion) {
   if (getAvgDistance() <= distance) {
        float radians = direction * M_PI / 180.0;

        float x = speed * sin(radians);
        float y = speed * cos(radians);

        crabUpdate(x, y, fieldOriented);
   } else {
     if(stop_on_completion) {
           stop();
    }
    }
}

void SwerveDrive::swerveUpdateInner(float x, float y, float z, float gyroDegrees, bool fieldOriented) {
    // Makes joystick inputs field oriented
    if (fieldOriented) {
        float gyroRadians = gyroDegrees * M_PI / 180;
        float temp = y * cos(gyroRadians) + x * sin(gyroRadians);
        x = -y * sin(gyroRadians) + x * cos(gyroRadians);
        y = temp;
    }

    float r = sqrt((DRIVE_LENGTH * DRIVE_LENGTH) + (DRIVE_WIDTH * DRIVE_WIDTH)); // radius of the drive base

    float a = x - z * (DRIVE_LENGTH / r); // temp variables to simplify math
    float b = x + z * (DRIVE_LENGTH / r);
    float c = y - z * (DRIVE_WIDTH / r);
    float d = y + z * (DRIVE_WIDTH / r);

    float flSpeed = sqrt(b * b + c * c);
    float frSpeed = sqrt(b * b + d * d);
    float blSpeed = sqrt(a * a + d * d);
    float brSpeed = sqrt(a * a + c * c);

    float flAngle = atan2(b, c) * 180 / M_PI; // calculates wheel angles and converts to radians
    float frAngle = atan2(b, d) * 180 / M_PI;
    float blAngle = atan2(a, c) * 180 / M_PI;
    float brAngle = atan2(a, d) * 180 / M_PI;

    if (flAngle < 0) {
        flAngle = flAngle + 360;
    }

    if (frAngle < 0) {
        frAngle = frAngle + 360;
    }

    if (blAngle < 0) {
        blAngle = blAngle + 360;
    }

    if (brAngle < 0) {
        brAngle = brAngle + 360;
    }

    float max = std::max(std::max(frSpeed, flSpeed), std::max(brSpeed, blSpeed)); // find max speed value

    // scale inputs respectively so no speed is greater than 1
    if (max > 1) {
        flSpeed /= max;
        frSpeed /= max;
        blSpeed /= max;
        brSpeed /= max;
    }

    float scalar = 1; // scalar to adjust if speed is too high
    flSpeed *= scalar;
    frSpeed *= scalar;
    blSpeed *= scalar;
    brSpeed *= scalar;

    float currentFL = flModule.getAngle();
    float currentFR = frModule.getAngle();
    float currentBR = brModule.getAngle();
    float currentBL = blModule.getAngle();

    float deadzoneCheck = sqrt(x * x + y * y);

    /*if (deadzoneCheck < 0.15 && fabs(z) < 0.01) {
        flSpeed = 0;
        frSpeed = 0;
        blSpeed = 0;
        brSpeed = 0;

        flAngle = currentFL;
        frAngle = currentFR;
        blAngle = currentBL;
        brAngle = currentBR;
    }*/

    // update speeds and angles
    // frc::SmartDashboard::PutNumber("fl output", flSpeed);-
    // frc::SmartDashboard::PutNumber("fr output", frSpeed);
    // frc::SmartDashboard::PutNumber("bl output", blSpeed);
    // frc::SmartDashboard::PutNumber("br output", brSpeed);
    if (flModule.adjustAngle(flAngle)) {
        flModule.setDrivePercent(-flSpeed);
    } else {
        flModule.setDrivePercent(flSpeed);
    }

    if (frModule.adjustAngle(frAngle)) {
        frModule.setDrivePercent(-frSpeed);
    } else {
        frModule.setDrivePercent(frSpeed);
    }

    if (blModule.adjustAngle(blAngle)) {
        blModule.setDrivePercent(-blSpeed);
    } else {
        blModule.setDrivePercent(blSpeed);
    }

    if (brModule.adjustAngle(brAngle)) {
        brModule.setDrivePercent(-brSpeed);
    } else {
        brModule.setDrivePercent(brSpeed);
    }
}

float SwerveDrive::crabCalcZ(float angle, float gyroDegrees) {
    return std::clamp(crabAnglePid.Calculate(gyroDegrees, angle) / 270.0, -0.5, 0.5);
}

float SwerveDrive::turnCalcZ(float angle, float gyroDegrees) {
    return std::clamp(turnAnglePid.Calculate(gyroDegrees, angle), -0.2, 0.2);
}

bool SwerveDrive::odometryGoToPose(double xDesired, double yDesired, double rotDesired) {
    odometryGoToPose(frc::Pose2d(units::meter_t(xDesired), units::meter_t(yDesired), frc::Rotation2d(units::radian_t(rotDesired))));
}

bool SwerveDrive::odometryGoToPose(frc::Pose2d desired) {
        double xPose = a_odometry.GetPose().X().value();
        double yPose = a_odometry.GetPose().Y().value();
        double rotPose = a_odometry.GetPose().Rotation().Radians().value();

        xProfiledPid.SetGoal(desired.X());
        yProfiledPid.SetGoal(desired.Y());
        rotProfiledPid.SetGoal(desired.Rotation().Radians());
        double xSpeed = std::clamp (xProfiledPid.Calculate(units::meter_t(xPose), desired.X()), -.25, .25);
        double ySpeed = std::clamp (yProfiledPid.Calculate(units::meter_t(yPose), desired.Y()), -.25, .25);
        double rotSpeed = std::clamp (rotProfiledPid.Calculate(units::radian_t(rotPose), desired.Rotation().Radians()), -.25, .25);
        frc::SmartDashboard::PutNumber("xSpeed", xSpeed);
        frc::SmartDashboard::PutNumber("ySpeed", ySpeed);
        frc::SmartDashboard::PutNumber("rotSpeed", rotSpeed);
        swerveUpdate(ySpeed, xSpeed, -rotSpeed, true);
        //swerveUpdate(0.0, 0.0, -rotSpeed, true);
        //return (xProfiledPid.AtGoal() && yProfiledPid.AtGoal() && rotProfiledPid.AtGoal());

        if (fabs(xPose - desired.X().value()) < .1
            && fabs(yPose - desired.Y().value()) < .1
            && fabs(rotPose - desired.Rotation().Radians().value()) < .087) {

            stop();
            return true;
        }
        else{
            swerveUpdate(ySpeed, xSpeed, -rotSpeed, true);
            return false;
        }
}
void SwerveDrive::updateOdometry(){
     a_odometry.Update(getGyroAngle(), getModulePositions());
   
    std::optional<photon::EstimatedRobotPose> pose = vision.estimate_position();
    if (pose) {
        frc::Pose2d p = (*pose).estimatedPose.ToPose2d();
        units::second_t t = (*pose).timestamp;
        poseEstimator.AddVisionMeasurement(p, t);
        poseEstimator.UpdateWithTime(t, getGyroAngle(), getModulePositions());
    } else {
        poseEstimator.Update(getGyroAngle(), getModulePositions());
    }
}
frc::Pose2d SwerveDrive::getPose(){
    return a_odometry.GetPose();
}
double SwerveDrive::getXPose(){
    return getPose().X().value();
}
double SwerveDrive::getYPose(){
    return getPose().Y().value();
}
double SwerveDrive::getRotPose(){
    return getPose().Rotation().Degrees().value();
}
void SwerveDrive::zeroPose(frc::Pose2d pose){
    a_odometry.ResetPosition(getGyroAngle(), getModulePositions(), pose);
}

frc::Pose2d SwerveDrive::getPoseEstimatorPose(){
    return poseEstimator.GetEstimatedPosition();
}

double SwerveDrive::getPoseEstimatorX(){
    return getPoseEstimatorPose().X().value();
}

double SwerveDrive::getPoseEstimatorY(){
    return getPoseEstimatorPose().Y().value();
}

double SwerveDrive::getPoseEstimatorRot(){
    return getPoseEstimatorPose().Rotation().Degrees().value();
}

frc::Rotation2d SwerveDrive::getGyroAngle() {
    return frc::Rotation2d(units::degree_t(a_gyro.getAngleClamped()));
}

wpi::array<frc::SwerveModulePosition, 4U> SwerveDrive::getModulePositions() {
    return {flModule.GetPosition(), frModule.GetPosition(), blModule.GetPosition(), brModule.GetPosition()};
}