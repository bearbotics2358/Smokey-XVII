
#include "Robot.h"
#include "Autonomous.h"
#include "Prefs.h"
#include "buttons.h"
#include "misc.h"
#include "Gyro.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <stdio.h>
#include <frc/interfaces/Gyro.h>
#include "Claw.h"
#include <frc/XboxController.h>
#include <cmath>
#include "Collector.h"
#include "BeamBreak.h"

/*~~ hi :) ~~ */
Robot::Robot():
a_Gyro(GYRO_ID),
a_Claw(ARM_MOTOR, SHUTTLE_MOTOR, PISTON_PUSH_SOLENOID_MODULE, PISTON_PULL_SOLENOID_MODULE, CLAW_OPEN_SOLENOID_MODULE, CLAW_CLOSE_SOLENOID_MODULE, /*CONE_PRESSURE_SOLENOID, CUBE_PRESSURE_SOLENOID,*/ CANCODER_ID_ARM, LIMIT_SWITCH), //Get the IDs for the arms solenoids
a_FLModule(misc::GetFLDrive(), misc::GetFLSteer(), misc::GetFLCANCoder()),
a_FRModule(misc::GetFRDrive(), misc::GetFRSteer(), misc::GetFRCANCoder()),
a_BLModule(misc::GetBLDrive(), misc::GetBLSteer(), misc::GetBLCANCoder()),
a_BRModule(misc::GetBRDrive(), misc::GetBRSteer(), misc::GetBRCANCoder()),
a_SwerveDrive(a_FLModule, a_FRModule, a_BLModule, a_BRModule, a_Gyro),
a_TOF(),
a_Autonomous(&a_Gyro, &a_SwerveDrive, &a_Claw, &a_TOF),
a_DriverXboxController(DRIVER_PORT),
a_OperatorXboxController(OPERATOR_PORT),
a_CompressorController(),
a_LED(ARDUINO_DIO_PIN),
rotPid(.5, 0, 0)
// NEEDED A PORT, THIS IS PROBABLY WRONG, PLEASE FIX IT LATER
//  handler("169.254.179.144", "1185", "data"),
//  handler("raspberrypi.local", 1883, "PI/CV/SHOOT/DATA"),
//  a_canHandler(CanHandler::layout2022()),
{
    /*if (!handler.ready()) {
        // do something if handler failed to connect
    }*/

    armStage = 1;
    clawClosed = false;

    a_FLModule.setDrivePID(0.001, 0, 0);
    a_FLModule.setSteerPID(STEERING_PVALUE, .3, 0);
    

    a_FRModule.setDrivePID(pvaluedrive, 0, 0);

   a_FRModule.setSteerPID(STEERING_PVALUE, .3, 0);

    a_BLModule.setDrivePID(0.001, 0, 0);
    a_BLModule.setSteerPID(STEERING_PVALUE, .3, 0);

    a_BRModule.setDrivePID(0.001, 0, 0);
    a_BRModule.setSteerPID(STEERING_PVALUE, .3, 0);

    a_SwerveDrive.brakeOnStop();
}

void Robot::RobotInit() {
    frc::SmartDashboard::init();

#ifndef COMP_BOT
    // using #ifndef here to indicate when this is the practice bot
    frc::SmartDashboard::PutString("Bot type", "PRACTICE BOT");
#endif

    a_Gyro.Init();
    a_Gyro.Zero();

    m_AutoModeSelector.SetDefaultOption(RobotDoNothing, RobotDoNothing);
    m_AutoModeSelector.AddOption(onePieceAMP, onePieceAMP);
    m_AutoModeSelector.AddOption(twoPieceAMP, twoPieceAMP);
    m_AutoModeSelector.AddOption(BlueMiddleOneNote, BlueMiddleOneNote);
    m_AutoModeSelector.AddOption( BlueMiddleTwoNote,  BlueMiddleTwoNote);
    m_AutoModeSelector.AddOption(BlueRightOneNote, BlueRightOneNote);
    m_AutoModeSelector.AddOption(BlueRightTwoNote, BlueRightTwoNote);
    m_AutoModeSelector.AddOption(RedDropAndGoLeft, RedDropAndGoLeft);
    m_AutoModeSelector.AddOption(RedChargeStationLeft, RedChargeStationLeft);
    m_AutoModeSelector.AddOption(RedDropAndGoMiddle, RedDropAndGoMiddle);
    m_AutoModeSelector.AddOption(RedChargeStationMiddle, RedChargeStationMiddle);
    m_AutoModeSelector.AddOption(RedDropAndGoRight, RedDropAndGoRight);
    m_AutoModeSelector.AddOption(RedChargeStationRight, RedChargeStationRight);
    m_AutoModeSelector.AddOption(LeftTwoPiece, LeftTwoPiece);
    m_AutoModeSelector.AddOption(RightTwoPiece, RightTwoPiece);
    frc::SmartDashboard::PutData("Auto Modes", &m_AutoModeSelector);

    a_LED.Init();

    SetTargetType(target_type_enum::CONE);
}

void Robot::RobotPeriodic() {
    a_Gyro.Update();
    a_Claw.updateDashboard();
    a_LED.Update();
    a_TOF.Update();
    a_Claw.UpdateShuttleEncoder(); //automatically sets the shuttle's encoder to 0 if hitting the limit switch

//testing code block for PID tuning

    // if(a_DriverXboxController.GetRawButton(3)) {
    //     a_FRModule.steerToAng(120);
    //     a_FLModule.steerToAng(120);
    //     a_BRModule.steerToAng(120);
    //     a_BLModule.steerToAng(120);
    // }
    // else {
    //     a_FRModule.steerToAng(150);
    //     a_FLModule.steerToAng(150);
    //     a_BRModule.steerToAng(150);
    //     a_BLModule.steerToAng(150);
    // }
    frc::SmartDashboard::PutNumber("Distance", a_SwerveDrive.getAvgDistance());
}

void Robot::DisabledInit() {
    a_doEnabledInit = true;
    a_SwerveDrive.resetDrive();
}
void Robot::EnabledInit(){}

void Robot::EnabledPeriodic() {
    a_CompressorController.update();
}
void Robot::DisabledPeriodic(){}


void Robot::AutonomousInit() {
    SetTargetType(target_type_enum::CONE);

    if (a_doEnabledInit) {
        EnabledInit();
        a_doEnabledInit = false;
    }

    a_SwerveDrive.unsetHoldAngle();
    a_Gyro.Zero();
    std::string SelectedRoute = m_AutoModeSelector.GetSelected(); //assigns value frm smart dashboard to a string variable

    a_Autonomous.StartAuto(SelectedRoute); //starts auto from selected route

}

void Robot::AutonomousPeriodic() {
    std::string SelectedRoute = m_AutoModeSelector.GetSelected(); //assigns value frm smart dashboard to a string variable
    a_Autonomous.PeriodicAuto(SelectedRoute);
    EnabledPeriodic();
}

void Robot::TeleopInit() {
    SetTargetType(target_type_enum::CONE);

    //a_Gyro.setYaw(180 + a_Gyro.getYaw());

    if (a_doEnabledInit) {
        EnabledInit();
        a_doEnabledInit = false;
    }

    // pChange = 0;
    // iChange = 0;
    // dChange = 0;

}

// main loop
void Robot::TeleopPeriodic() {
   
    // EnabledPeriodic();

   
    
    
    /* =-=-=-=-=-=-=-=-=-=-= Claw Controls =-=-=-=-=-=-=-=-=-=-= */
    // if (catchBegin || (a_TOF.GetTargetRangeIndicator() == target_range_enum::TARGET_IN_RANGE && a_DriverXboxController.GetRightTriggerAxis() > 0.5 && clawClosed == false)) {
    //     a_Claw.ClawClose();
    //     if(!catchBegin) {
    //         state_time = Autonomous::gettime_d();
    //         catchBegin = true;
    //     }
    //     if(Autonomous::gettime_d() > state_time + 0.5) {
    //         armStage = 1;
    //         clawClosed = true;
    //         catchBegin = false;
    //     }
    // }

    // if (a_DriverXboxController.GetYButton()){
    //     armStage = 1;
    // } else if (a_DriverXboxController.GetBButton()) {
    //     armStage = 2;
    // } else if (a_OperatorXboxController.GetLeftBumperPressed()) {
    //     armStage = 3;
    // } else if (a_OperatorXboxController.GetRightBumperPressed()) {
    //     armStage = 4;
    // } else if (a_DriverXboxController.GetAButton()) {
    //     armStage = 6;
    // }

    // switch (armStage) {
    //     case 1:
    //         a_Claw.TransformClaw(125, -15, false); // transport
    //         break;
    //     case 2:
    //         a_Claw.TransformClaw(10, -15, false); // arm down pointing downwards from the back
    //         break;
    //     case 3:
    //         a_Claw.TransformClaw(190, 500, false); // arm at the top, piston off
    //         break;
    //     case 4:
    //         isHighPistonDone = false;
    //         piston_time = Autonomous::gettime_d();
    //         armStage = 5;
    //         break;
    //     case 5:
    //         if (!isHighPistonDone){
    //             bool pistonDone = a_Claw.TransformClaw(160, 500, true);
    //             isHighPistonDone = pistonDone && (Autonomous::gettime_d() > piston_time + 3);
    //         } else {
    //             a_Claw.TransformClaw(185, 500, true); // arm at the top, piston on
    //         }
    //         break;
    //     case 6: {
    //         bool transformDone = a_Claw.TransformClaw(160, 640, false);
    //         if (transformDone){
    //             armStage = 7;
    //         }
    //         break;
    //     }
    //     case 7:
    //         a_Claw.TransformClaw(300, 640, false); //set point 290 640
    //         break;
    //     default:
    //         a_Claw.TransformClaw(130, -15, false); // transport as default state
    //         break;
    // }

    // claw open/close controls
    // if(a_DriverXboxController.GetRightBumper()) {
    //     a_Claw.ClawOpen();
    //     clawClosed = false;
    // } else if (a_DriverXboxController.GetLeftBumper()) {
    //     a_Claw.ClawClose();
    //     clawClosed = true;
    // }

    /* =-=-=-=-=-=-=-=-=-=-= Swerve Controls =-=-=-=-=-=-=-=-=-=-= */

    // dpad up for full speed,
    // down for half speed
    if (a_DriverXboxController.GetLeftTriggerAxis() > .5) {
        a_slowSpeed = true;
    } else  {
        a_slowSpeed = false;
    }

    float multiplier = 1.0;
    if (a_slowSpeed) {
        multiplier = 0.2;
    }

    float x = a_DriverXboxController.GetLeftX();
    float y = a_DriverXboxController.GetLeftY();
    float z = a_DriverXboxController.GetRightX();

    if (fabs(x) < 0.10) {
        x = 0;
    }
    if (fabs(y) < 0.10) {
        y = 0;
    }
    if (fabs(z) < 0.10) {
        z = 0;
    }

    bool inDeadzone = (sqrt(x * x + y * y) < JOYSTICK_DEADZONE) && (fabs(z) < JOYSTICK_DEADZONE); // Checks joystick deadzones
    
    if(a_DriverXboxController.GetLeftBumperPressed()){
        a+=.1;
    }
    else if(a_DriverXboxController.GetRightBumperPressed()){
        a-=.1;
    }
    frc::SmartDashboard::PutNumber("a", a);
    x = (1-a)*xlast + a*x;
    y = (1-a)*ylast + a*y;
    z = (1-a)*zlast + a*z;
    // scale by multiplier for slow mode, do this after deadzone check
    x *= multiplier;
    y *= multiplier;
    z *= multiplier;

    // turn field oriented mode off if the trigger is pressed for more than 0.25 (GetRightTriggerAxis ranges from 0 to 1)

    bool fieldOreo = true;
    if(a_DriverXboxController.GetPOV() == 0 && fieldOreo == true)
    {
        fieldOreo = false;
    }
    else if(a_DriverXboxController.GetPOV() == 0 && fieldOreo == false){
        fieldOreo = true;
    }

    frc::SmartDashboard::PutBoolean("field oriented: ", fieldOreo);
    
    // calibrate gyro
    if (a_DriverXboxController.GetPOV() == 180) {
        a_Gyro.Cal();
        a_Gyro.Zero();
    }

    xlast = x;
    ylast = y;
    zlast = z;
    frc::SmartDashboard::PutNumber("x", x);
    frc::SmartDashboard::PutNumber("y", y);
    frc::SmartDashboard::PutNumber("z", z);

    photonlib::PhotonPipelineResult result = a_camera.GetLatestResult();
    if (!inDeadzone) {
        a_SwerveDrive.swerveUpdate(x, y, z, fieldOreo);
    } 
    else if(a_DriverXboxController.GetRightTriggerAxis() > 0.5) {
         
               
      
        if (result.HasTargets()) {
            photonlib::PhotonTrackedTarget target = result.GetBestTarget();
            double target_Yaw = target.GetYaw();
            double goToYaw = a_Gyro.getAngleClamped() + target_Yaw;
            frc::SmartDashboard::PutNumber("Yaw", goToYaw);
            a_SwerveDrive.turnToAngle(goToYaw, true);
        }
        // if((turnAnglePid.AtSetpoint())){
        //     a_SwerveDrive.stop();
        // }

        // frc::Transform3d bestCameraToTarget = target.GetBestCameraToTarget();
        // double X = bestCameraToTarget.X().value();
        // double Y = bestCameraToTarget.Y().value();
        // double R = half_of_range(X, Y);
        // double theta = calculate_shooting_angle(R);
        // double velocity = velocity_needed_to_reach_height(theta);
        // double rpm = velocity_to_rpm(velocity);
        // frc::SmartDashboard::PutNumber("X: ", X);
        // frc::SmartDashboard::PutNumber("Y: ", Y);
        // frc::SmartDashboard::PutNumber("Range: ", R);
        // frc::SmartDashboard::PutNumber("Angle: ", theta);
        // frc::SmartDashboard::PutNumber("Velocity: ", velocity);
        // frc::SmartDashboard::PutNumber("RPM: ", rpm);
    }
    else {         
        a_SwerveDrive.stop();
    } 
}

void Robot::TestInit() {
    TeleopInit();
}


void Robot::TestPeriodic() {
   TeleopPeriodic();
    // if(a_DriverXboxController.GetLeftBumperPressed()){
    //     pvaluesteer+=.1;
    // }
    // else if(a_DriverXboxController.GetRightBumperPressed()){
    //     pvaluesteer-=.1;
    // }
    // else if(a_DriverXboxController.GetAButtonPressed()){
    //     ivaluesteer+=.1;
    // }
    // else if(a_DriverXboxController.GetBButtonPressed()){
    //     ivaluesteer-=.1;
    // }
    // else if(a_DriverXboxController.GetXButtonPressed()){
    //     dvaluesteer+=.1;
    // }
    // else if(a_DriverXboxController.GetYButtonPressed()){
    //     dvaluesteer-=.1;
    // }
    // a_FLModule.setSteerPID(pvaluesteer, ivaluesteer, dvaluesteer);
    
    // a_FRModule.setSteerPID(pvaluesteer, ivaluesteer, dvaluesteer);

    // a_BLModule.setSteerPID(pvaluesteer, ivaluesteer, dvaluesteer);

    // a_BRModule.setSteerPID(pvaluesteer, ivaluesteer, dvaluesteer);

    // frc::SmartDashboard::PutNumber("pvaluesteer", pvaluesteer);
    // frc::SmartDashboard::PutNumber("ivaluesteer", ivaluesteer);
    // frc::SmartDashboard::PutNumber("dvaluesteer", dvaluesteer);

     // if(a_DriverXboxController.GetRightTriggerAxis() > 0.25) {
    //     a_FRModule.steerToAng(0);
    //     a_FLModule.steerToAng(0);
    //     a_BRModule.steerToAng(0);
    //     a_BLModule.steerToAng(0);
    // } 
    // else {
    //     a_FRModule.steerToAng(45);
    //     a_FLModule.steerToAng(45);
    //     a_BRModule.steerToAng(45);
    //     a_BLModule.steerToAng(45);
    // }
}

void Robot::SetTargetType(target_type_enum target) {
    target_type = target;
    if(target_type == target_type_enum::CONE) {
        // Set target type to CONE
        a_LED.SetTargetType(target_type_enum::CONE);
        a_TOF.SetTargetType(target_type_enum::CONE);
        //a_Claw.ConePressure();
    } else if(target_type == target_type_enum::CUBE) {
        // Set target type to CUBE
        a_LED.SetTargetType(target_type_enum::CUBE);
        a_TOF.SetTargetType(target_type_enum::CUBE);
        //a_Claw.CubePressure();

    }
}


int main() { return frc::StartRobot<Robot>(); } // Initiate main loop

// Double values are assumed to be meters. theta is in degrees
double Robot::velocity_needed_to_reach_height(double theta) {
    return sqrt(
        (2 * SPEAKER_HEIGHT_CENTER * ACCELERATION_DUE_TO_GRAVITY) /
        (pow(sin(theta), 2))
    ); 
}

double Robot::half_of_range(double x, double y) {
    return sqrt(x * x + y * y);
}

double Robot::calculate_shooting_angle(double range) {
    return atan2(SPEAKER_HEIGHT_CENTER, range) * (180.0 / M_PI);
}

double Robot::velocity_to_rpm(double velocity) {
    return (30 * velocity) / (M_PI * RADIUS_OF_MOTOR);
}