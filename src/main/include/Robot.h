
#pragma once

#include "Autonomous.h"
#include "CanHandler.h"
#include "CompressorController.h"
#include "Gyro.h"
#include "LimitSwitch.h"
#include "SwerveDrive.h" // Swerve kinematics
#include "SwerveModule.h" // Swerve modules
#include "photonlib/photon.h"
#include <frc/Joystick.h> // Joystick
#include <frc/TimedRobot.h> // "Timed Robot" template
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>
#include "BeamBreak.h"
#include "Shooter.h"
#include <frc/smartdashboard/SendableChooser.h>
#include "Collector.h"
#include "BeamBreak.h"
#include "LED_DIO.h"
#include <frc/controller/PIDController.h>



enum class DriveBackState {
    Inactive,
    Start,
    Active,
};

class Robot : public frc::TimedRobot {
    public:
        Robot();
        void RobotInit();
        void RobotPeriodic();

        void DisabledInit();
        void DisabledPeriodic();

        // called whenever the robot transitions from disabled to either autonomous, teleop, or test
        // this means it is basically called whenever the robot is enabled
        // is not called when the robot moves between autonomous, teleop, or test
        void EnabledInit();
        // called during autonomous, teleop, and test periodic
        void EnabledPeriodic();

        void AutonomousInit();
        void AutonomousPeriodic();
        void DecidePath();
        void TeleopInit();
        void TeleopPeriodic();

        void TestInit();
        void TestPeriodic();

        // void SetTargetType(target_type_enum target);



    private:
        int armStage;
        bool isHighPistonDone;
        // keeps track of when to call enabled init
        bool a_doEnabledInit { true };
        frc::SendableChooser<std::string> m_AutoModeSelector;

        Gyro a_Gyro;

        SwerveModule a_FLModule;
        SwerveModule a_FRModule;
        SwerveModule a_BLModule;
        SwerveModule a_BRModule;
        SwerveDrive a_SwerveDrive;


        // speed multiplier for driver controls for the swerve
        bool a_slowSpeed { false };
        double xlast = 0.0;
        double ylast = 0.0;
        double zlast = 0.0;
        double a = 1.0;
        double xnew = 0;
        double ynew = 0;
        double znew = 0;
        double pvaluesteer = 2.7;
        double ivaluesteer = 0.3;
        double dvaluesteer = 0.0;

        Autonomous a_Autonomous;
        Shooter a_Shooter;
        Collector a_Collector;

        frc::XboxController a_DriverXboxController; // 3D flightstick (Logitech Attack 3?)
        frc::XboxController a_OperatorXboxController;

        //CompressorController a_CompressorController;


        //LED_DIO a_LED;

        double state_time;
        double piston_time;
        bool catchBegin = false;

        // stuff that autonomous needs

        double pvaluedrive;
        double pChange;
        double iChange;
        double dChange;

        units::meter_t newXComponent;

        //--------------photonvision-------------//
        const units::meter_t CAMERA_HEIGHT = 24_in;
        const units::meter_t TARGET_HEIGHT = 5_ft;

        // Angle between horizontal and the camera.
        const units::radian_t CAMERA_PITCH = 0_deg;

        // How far from the target we want to be
        const units::meter_t GOAL_RANGE_METERS = 3_ft;

        // PID constants should be tuned per robot
        const double LINEAR_P = 0.1;
        const double LINEAR_D = 0.0;
        frc2::PIDController forwardController{LINEAR_P, 0.0, LINEAR_D};
        const double ANGULAR_P = 0.1;
        const double ANGULAR_D = 0.0;
        frc2::PIDController turnController{ANGULAR_P, 0.0, ANGULAR_D};

        photonlib::PhotonCamera a_camera{"limelight1"}; //name of camera

        enum target_type_enum target_type = target_type_enum::CONE;


};
