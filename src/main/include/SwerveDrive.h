#pragma once

#include <frc/controller/PIDController.h>

#include "Gyro.h"
#include "Prefs.h"
#include "SwerveModule.h"
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <Vision.h>

class SwerveDrive // Class to handle the kinematics of Swerve Drive
{
    public:
        SwerveDrive(SwerveModule& flModule, SwerveModule& frModule, SwerveModule& blModule, SwerveModule& brModule, Gyro& gyro);

        // TODO: change the signs of x, because the positive is left thing is weird
        // TODO: use meters/second vector for crabUpdate and swerveUpdate, instead of x and y going from 0 to 1
        // for crab drive update and swerve drive update, +y is forward, -y is backward, x is left, and -x is right
        // crab drive is like swerve drive update, except it maintains a constant turn angle
        void crabUpdate(float x, float y, bool fieldOriented = true);
        void swerveUpdate(float x, float y, float z, bool fieldOriented);
        /*
            x = x axis on joystick
            y = y axis on joystick
            z = z axis on joystick (rotation)
            gyroDegrees = sensor
            fieldOriented = if true, translation movement is relative to the field
            if false, translational movement is relative to the front of the robot,
            and it is affected by the robot's current turn angle
        */

        // stops the robot from moving, just coasts
        void stop();

        // configures the swerve drive to brake when stopped
        void brakeOnStop();

        // configures the swerve drive to coast when stopped
        void coastOnStop();

        // sets the hold angle used by crab drive update
        void setHoldAngle(float degrees);

        // unsets the hold angle, so the next call to crabUpdate will set the hold angle to the reading from the gyro
        void unsetHoldAngle();

        // resets steering and driving encoders
        void resetDrive();

        // dist in meters and angle 0-360
        void driveDistance(float distMeters, float directionDegrees);

        /** Drives at a given percent speed in a given direction.
         *  @param percent percent output of drive motors, from -1 to 1.
         *  @param directionDegrees direction to drive in degress from 0 to 360.
         */
        void driveDirection(float percent, float directionDegrees);
        void driveDirectionVelocity(float speed, float directionDegrees);

        // returns the average of the total distance of the drive encoders in all 4 modules in meters
        float getAvgDistance();
        float getAvgVelocity();
       
        // angle is in degrees
        bool turnToAngle(float angle, bool positive_speed);

        // drives at a given speed (units uknown), in a given direction in degrees, for a given distance in meters
        // if another call to goToTheDon() will follow this call, suggest setting stop_on_completion false so that
        // the 'bot will not slam on the brakes once the first run is successful
        void goToTheDon(float speed, float direction, float distance, bool fieldOriented = true, bool stop_on_completion = true);

        // This version of odometryGoToPose is kept for backwards compatibility. If all usage is converted to
        // use the one with the Pose2d parameter, this one can be removed.
        bool odometryGoToPose(double xDesired, double yDesired, double rotDesired);

        bool odometryGoToPose(frc::Pose2d desiredPose);

        void updateOdometry();

        frc::Pose2d getPose();
        double getXPose();
        double getYPose();
        double getRotPose();
        void zeroPose(frc::Pose2d pose);

        // pose estimator
        frc::Pose2d getPoseEstimatorPose();
        double getPoseEstimatorX();
        double getPoseEstimatorY();
        double getPoseEstimatorRot();

        frc::Rotation2d getGyroAngle();
        wpi::array<frc::SwerveModulePosition, 4U> getModulePositions();

    private:
        // called by both crabUpdate and swerveUpdata
        // does the bulk of the swerve drive work
        // x and y are translation, z is rotation
        void swerveUpdateInner(float x, float y, float z, float gyroDegrees, bool fieldOriented);

        // uses the crab pid to calulate the required z drive to get to the specified angle
        float crabCalcZ(float angle, float gyroDegrees);

        // uses the turn pid to calculate the required z drive
        float turnCalcZ(float angle, float gyroDegrees);

        SwerveModule& flModule;
        SwerveModule& frModule;
        SwerveModule& blModule;
        SwerveModule& brModule;
        Gyro& a_gyro;

        // pid when using turn to angle
        frc::PIDController turnAnglePid;

        // pid when using crabUpdate
        frc::PIDController crabAnglePid;

        // if we're in crab drive mode
        bool crab;
        // angle to hold in crab drive mode
        float holdAngle;

        // last position of each drive wheel in meters
        float flLastPos { 0.0 };
        float frLastPos { 0.0 };
        float blLastPos { 0.0 };
        float brLastPos { 0.0 };

        constexpr static float DRIVE_LENGTH = 13.0026;
        constexpr static float DRIVE_WIDTH = 29.4878;

        // for goToPosition, when the distance to the target position is within this amount, say that we are done (assuming angle is also close enough)
        constexpr static float GO_TO_DIST_DONE = 0.2;
        // for goToPosition, when the angle difference from the target angle is within this amount, say that we are done (assuming distance is also close enough)
        constexpr static float GO_TO_ANGLE_DONE = 5.0;

       // 36 in 29.5 in
        frc::Translation2d a_FLLocation{+0.11_m, +0.375_m};
        frc::Translation2d a_FRLocation{+0.2_m, -0.375_m};
        frc::Translation2d a_BLLocation{-0.11_m, +0.375_m};
        frc::Translation2d a_BRLocation{-0.2_m, -0.375_m};


        frc::SwerveDriveKinematics<4> a_kinematics{a_FLLocation, a_FRLocation, a_BLLocation, a_BRLocation};

        frc::Rotation2d Rotation2d;
        frc::SwerveDriveOdometry<4> a_odometry;
        
        frc::TrapezoidProfile<units::meters>::Constraints linearConstraints{units::meters_per_second_t(1.0), units::meters_per_second_squared_t(1.0)};
        frc::ProfiledPIDController<units::meters> xProfiledPid;
        frc::ProfiledPIDController<units::meters> yProfiledPid;

        frc::TrapezoidProfile<units::radian>::Constraints rotationalConstraints{units::radians_per_second_t(2.0), units::radians_per_second_squared_t(2.0)};
        frc::ProfiledPIDController<units::radian> rotProfiledPid;

        // Pose Estimator
        frc::SwerveDrivePoseEstimator<4> poseEstimator;
        Vision vision;
};
