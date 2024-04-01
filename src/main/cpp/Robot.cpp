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
#include <frc/XboxController.h>
#include "Collector.h"
#include "BeamBreak.h"
#include "SwerveDrive.h"
#include "LED.h"
#include <frc/GenericHID.h>
#include "LimelightHelpers.h"


/*~~ hi :) ~~ */
Robot::Robot():
a_Gyro(GYRO_ID),
a_FLModule(misc::GetFLDrive(), misc::GetFLSteer(), misc::GetFLCANCoder()),
a_FRModule(misc::GetFRDrive(), misc::GetFRSteer(), misc::GetFRCANCoder()),
a_BLModule(misc::GetBLDrive(), misc::GetBLSteer(), misc::GetBLCANCoder()),
a_BRModule(misc::GetBRDrive(), misc::GetBRSteer(), misc::GetBRCANCoder()),
a_SwerveDrive(a_FLModule, a_FRModule, a_BLModule, a_BRModule, a_Gyro),
a_DriverXboxController(DRIVER_PORT),
a_OperatorXboxController(OPERATOR_PORT),
a_Gamepad(4),
a_NoteHandler(),
//a_CompressorController(),
//a_LED(ARDUINO_DIO_PIN),
// a_Shooter(SHOOTER_RIGHT_MOTOR_ID, SHOOTER_LEFT_MOTOR_ID, PIVOT_MOTOR_ID, LIMIT_SWITCH),
a_Autonomous(&a_Gyro, &a_SwerveDrive, &a_NoteHandler)
// NEEDED A PORT, THIS IS PROBABLY WRONG, PLEASE FIX IT LATER
//  handler("169.254.179.144", "1185", "data"),
//  handler("raspberrypi.local", 1883, "PI/CV/SHOOT/DATA"),
//  a_canHandler(CanHandler::layout2022()),
{
    /*if (!handler.ready()) {
        // do something if handler failed to connect
    }*/

    pvaluedrive = .037;
    a_FLModule.setDrivePID(pvaluedrive, 0, 0);
    a_FLModule.setSteerPID(pvaluesteer, ivaluesteer, dvaluesteer);


    a_FRModule.setDrivePID(pvaluedrive, 0, 0);

   a_FRModule.setSteerPID(pvaluesteer, ivaluesteer, dvaluesteer);

    a_BLModule.setDrivePID(pvaluedrive, 0, 0);
    a_BLModule.setSteerPID(1.6, ivaluesteer, dvaluesteer);

    a_BRModule.setDrivePID(pvaluedrive, 0, 0);
    a_BRModule.setSteerPID(pvaluesteer, ivaluesteer, dvaluesteer);

    a_NoteHandler.setRotPID(rotP, rotI, rotD);

    a_SwerveDrive.brakeOnStop();
}

void Robot::RobotInit() {
    a_NoteHandler.setExtensionPosition();
    a_NoteHandler.setClimberPosition();
    aprilTagFieldLayout.SetOrigin(frc::Pose3d(units::meter_t(-0.038), units::meter_t(5.55), units::meter_t(1.45), frc::Rotation3d(units::radian_t(0.0), units::radian_t(90.0), units::radian_t(0.0))));
    frc::SmartDashboard::init();

#ifndef COMP_BOT
    // using #ifndef here to indicate when this is the practice bot
    frc::SmartDashboard::PutString("Bot type", "PRACTICE BOT");
#endif

    a_Gyro.Init();
    a_Gyro.Zero();

    
    m_AutoModeSelector.SetDefaultOption(RobotDoNothing, RobotDoNothing);
    m_AutoModeSelector.AddOption(RobotDoNothing, RobotDoNothing);
    m_AutoModeSelector.AddOption(firstNote, firstNote);
    m_AutoModeSelector.AddOption(secondNote, secondNote);
    m_AutoModeSelector.AddOption(thirdNote,  thirdNote);
    m_AutoModeSelector.AddOption(fourthNote, fourthNote);
    m_AutoModeSelector.AddOption(fifthNote, fifthNote);
    m_AutoModeSelector.AddOption(sixthNote, sixthNote);
    m_AutoModeSelector.AddOption(seventhNote, seventhNote);
    m_AutoModeSelector.AddOption(eighthNote, eighthNote);
    frc::SmartDashboard::PutData("Auto Modes", &m_AutoModeSelector);

    // a_LED.Init();
    // a_LED.SetAngleToNote(0.3);

    //a_LED.SetTargetType(LED_STAGE_enum::WHITE);
    //InterpolationValues value = {22.5, 3500};
    a_NoteHandler.insertToInterpolatingMap(2.546859, {22.5, 4000});
    a_NoteHandler.insertToInterpolatingMap(4.212965, {9.5, 4000});

}

void Robot::RobotPeriodic() {
    //a_LED.Update();

    a_NoteHandler.UpdateSensors();

    
    // if(a_NoteHandler.beamBroken()){
    //      a_LED.SetNoteOnBoard();
    // } else {
    //      a_LED.SetMSGIdle();
    // }

    a_NoteHandler.updateDashboard();


    // photon::PhotonPipelineResult result = a_camera.GetLatestResult();
    // double Note_Offset = LimelightHelpers::getTX("limelight-notes");

    // if (result.HasTargets()) {
    //     frc::SmartDashboard::PutString("Has_AprilTags", "YES");
    // } else { 
    //     frc::SmartDashboard::PutString("Has_AprilTags", "NO");
    // }

    // frc::SmartDashboard::PutNumber("Note_Offset", Note_Offset);

    frc::SmartDashboard::PutNumber("Shooter Angle", a_NoteHandler.getShooterAngle());

    // frc::SmartDashboard::PutBoolean("BeamBreak", a_NoteHandler.beamBroken());

    a_Gyro.Update();

    //a_LED.Update();


    a_SwerveDrive.updateOdometry();
    //frc::SmartDashboard::PutNumber("Shooter Angle", a_Shooter.GetShooterAngle().value());


    frc::SmartDashboard::PutNumber("xPose", (a_SwerveDrive.getXPose()));
    frc::SmartDashboard::PutNumber("yPose", (a_SwerveDrive.getYPose()));
    frc::SmartDashboard::PutNumber("degreePose", (a_SwerveDrive.getRotPose()));

    // frc::SmartDashboard::PutNumber("FL radians", a_FLModule.getAngle());
    // frc::SmartDashboard::PutNumber("FR Radians", a_FRModule.getAngle());
    // frc::SmartDashboard::PutNumber("BL Radians", a_BLModule.getAngle());
    // frc::SmartDashboard::PutNumber("BR Radians", a_BRModule.getAngle());

    // frc::SmartDashboard::PutNumber("FL Distance", a_FLModule.getDistance());
    // frc::SmartDashboard::PutNumber("FR Distance", a_FRModule.getDistance());
    // frc::SmartDashboard::PutNumber("BL Distance", a_BLModule.getDistance());
    // frc::SmartDashboard::PutNumber("BR Distance", a_BRModule.getDistance());

    // frc::SmartDashboard::PutNumber("FL Velocity", a_FLModule.getVelocity());
    // frc::SmartDashboard::PutNumber("FR Velocity", a_FRModule.getVelocity());
    // frc::SmartDashboard::PutNumber("BL Velocity", a_BLModule.getVelocity());
    // frc::SmartDashboard::PutNumber("BR Velocity", a_BRModule.getVelocity());
    // frc::SmartDashboard::PutNumber("Button Count", a_Gamepad.GetButtonCount());


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
    // frc::SmartDashboard::PutNumber("Distance", a_SwerveDrive.getAvgDistance());
    // frc::SmartDashboard::PutNumber("Velocity", a_SwerveDrive.getAvgVelocity());


    frc::SmartDashboard::PutNumber("Climb Position", a_NoteHandler.getClimberPosition());
    
    //frc::SmartDashboard::PutNumber("Climb Position", a_NoteHandler.getClimberPosition());

    std::optional<photon::EstimatedRobotPose> pose = a_Vision.estimate_position();

    frc::Pose3d p = (*pose).estimatedPose;
    frc::SmartDashboard::PutNumber("Pose Estimator X", p.X().value());
    frc::SmartDashboard::PutNumber("Pose Estimator Y", p.Y().value());
    frc::SmartDashboard::PutNumber("Pose Estimator Z", p.Z().value());
    
    frc::Pose3d april_tag = a_Vision.get_april_tag_pose();
    frc::SmartDashboard::PutNumber("April Tag X", april_tag.X().value());
    frc::SmartDashboard::PutNumber("April Tag Y", april_tag.Y().value());
    frc::SmartDashboard::PutNumber("April Tag Z", april_tag.Z().value());

    frc::Pose2d pose_estimator_pose = a_SwerveDrive.getPoseEstimatorPose();
    frc::SmartDashboard::PutNumber("pose_estimator_pose X", pose_estimator_pose.X().value());
    frc::SmartDashboard::PutNumber("pose_estimator_pose Y", pose_estimator_pose.Y().value());
}

void Robot::DisabledInit() {
    a_doEnabledInit = true;
    a_SwerveDrive.resetDrive();
}
void Robot::EnabledInit(){}

void Robot::EnabledPeriodic() {
  //  a_CompressorController.update();
}
void Robot::DisabledPeriodic(){}


void Robot::AutonomousInit() {
    //a_SwerveDrive.zeroPose();
    if (a_doEnabledInit) {
        EnabledInit();
        a_doEnabledInit = false;
    }

    a_SwerveDrive.unsetHoldAngle();
    
    std::string SelectedRoute = m_AutoModeSelector.GetSelected(); //assigns value frm smart dashboard to a string variable

    a_Autonomous.StartAuto(SelectedRoute); //starts auto from selected route
    
}

void Robot::AutonomousPeriodic() {
    //a_NoteHandler.startShooter(3500.0, 30.0); // change angle later
    std::string SelectedRoute = m_AutoModeSelector.GetSelected(); //assigns value frm smart dashboard to a string variable
    a_Autonomous.PeriodicAuto(SelectedRoute);
    EnabledPeriodic();
    a_NoteHandler.startShooter(3500, 40.0);
}

void Robot::TeleopInit() {
   // SetTargetType(target_type_enum::CONE);

    //a_Gyro.setYaw(180 + a_Gyro.getYaw());
    a_NoteHandler.stopAll();

    if (a_doEnabledInit) {
        EnabledInit();
        a_doEnabledInit = false;
    }
    a_Gyro.Zero(0.0);

    // pChange = 0;
    // iChange = 0;
    // dChange = 0;

}

// main loop
void Robot::TeleopPeriodic() {
    
    //a_Shooter.moveToAngle(20.0);
    // frc::SmartDashboard::PutNumber("desired angle", pivotAngle);
    // a_Shooter.moveToAngle(pivotAngle);
    // EnabledPeriodic();

    photon::PhotonPipelineResult result = a_camera.GetLatestResult();
    double goalYaw;
    // drive done
    // slow mode done
    //set shooter done
    //shoot done
    //collector done
    //transfer to amp done
    //score in amp done
    //set arm to starting position
    //run climber?

    /* =-=-=-=-=-=-=-=-=-=-= Shooter Controls =-=-=-=-=-=-=-=-=-=-= */
    // getting shooter up to speeed
    // if (a_OperatorXboxController.GetRightTriggerAxis() > .75) {
    //     // if (result.HasTargets()) {
        //     std::span<const photon::PhotonTrackedTarget> targets = result.GetTargets();
        //     for (photon::PhotonTrackedTarget target : targets) {
        //         int id = target.GetFiducialId();
        //         if (id == 4 || id == 7) {
        //             // shoot
        //             goalYaw = a_Gyro.getAngleClamped() - target.GetYaw();
        //         } else if (id == 3 || id == 8) {
        //             // shoot
        //         }
        //     }
        // } else {
        //     goalYaw = 0.0;
        // }
    //     double rpm = 3500;
    //     double angle = 35.0;
    //     a_NoteHandler.startShooter(rpm, angle);
    // } 
    // else {
    //     a_NoteHandler.stopShooter();
    //     a_NoteHandler.moveShooterToAngle(0.0);
    // }
    /* =-=-=-=-=-=-=-=-=-=-= Collector/Indexer Controls =-=-=-=-=-=-=-=-=-=-= */

    // start collector
    //a_NoteHandler.shootToAmp(a_DriverXboxController.GetRightTriggerAxis() > .75, a_DriverXboxController.GetAButton(), a_DriverXboxController.GetLeftBumper(), a_OperatorXboxController.GetRightTriggerAxis() > .75);
    // if(a_DriverXboxController.GetAButton()) {
        // if(a_NoteHandler.armToPose(154.0)){
        //     a_NoteHandler.runArmRoller();
        // }
    

    
    /* Amp Control*/
    
    /* =-=-=-=-=-=-=-=-=-=-= Swerve Controls =-=-=-=-=-=-=-=-=-=-= */

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

    // if(a_DriverXboxController.GetLeftBumperPressed()){
    //     a+=.1;
    // }
    // else if(a_DriverXboxController.GetRightBumperPressed()){
    //     a-=.1;
    // }
    //frc::SmartDashboard::PutNumber("a", a);
    x = (1-a)*xlast + a*x;
    y = (1-a)*ylast + a*y;
    z = (1-a)*zlast + a*z;
    // scale by multiplier for slow mode, do this after deadzone check
    x *= multiplier;
    y *= multiplier;
    z *= multiplier;

    // turn field oriented mode off if the trigger is pressed for more than 0.25 (GetRightTriggerAxis ranges from 0 to 1)

    bool fieldOreo = true;
    // if(a_DriverXboxController.GetPOV() == 0 && fieldOreo == true)
    // {
    //     fieldOreo = false;
    // }
    // else if(a_DriverXboxController.GetPOV() == 0 && fieldOreo == false){
    //     fieldOreo = true;
    // }

    // frc::SmartDashboard::PutBoolean("field oriented: ", fieldOreo);

    // calibrate gyro
    if (a_DriverXboxController.GetPOV() == 180) {
        a_Gyro.Cal();
        a_Gyro.Zero();
    }

    xlast = x;
    ylast = y;
    zlast = z;
    // frc::SmartDashboard::PutNumber("x", x);
    // frc::SmartDashboard::PutNumber("y", y);
    // frc::SmartDashboard::PutNumber("z", z);




    // if(a_DriverXboxController.GetAButton()){
    //     a_NoteHandler.runArmRoller(-8);
    // }
    // else if(a_DriverXboxController.GetBButton()){
    //     a_NoteHandler.runArmRoller(8);
    // }
    // else{
    //     a_NoteHandler.runArmRoller(0);
    // }
    
    a_NoteHandler.shootToAmp(
        a_DriverXboxController.GetRightTriggerAxis() > .75, //transfer to amp
        a_DriverXboxController.GetAButton(), //shoot into amp
        a_DriverXboxController.GetLeftBumper(), //bring to default amp
        a_Gamepad.GetRawButton(3), //run shooter
        a_DriverXboxController.GetRightBumper(), //shoot note
        a_Gamepad.GetRawButton(1),//run collector
        a_DriverXboxController.GetBButton(),// hooks up
        a_DriverXboxController.GetXButton());//finishclimb
                                            // eject from amp
                                            //eject from collector

    // if(a_DriverXboxController.GetRightTriggerAxis() > .5){
    //     a_SwerveDrive.odometryGoToPose(1.0, 1.0, M_PI/2);
    // }
    // else 
    if (!inDeadzone) {
        a_SwerveDrive.swerveUpdate(x, y, z, fieldOreo);
    }   
    //else if(a_DriverXboxController.GetRightBumper()) //{

        //  if (result.HasTargets()) {
        //      photon::PhotonTrackedTarget target = result.GetBestTarget();
        //      double target_Yaw = target.GetYaw();
        //      double goToYaw = a_Gyro.getAngleClamped() - target_Yaw;
        //      frc::SmartDashboard::PutNumber("GoalYaw", goToYaw);
        //      a_SwerveDrive.turnToAngle(goToYaw, true);
        //  }
        // }
    else {
        a_SwerveDrive.stop();
    }

    if(a_DriverXboxController.GetLeftBumperPressed()){
        a_SwerveDrive.zeroPose(frc::Pose2d(units::meter_t(0.0), units::meter_t(0.0), units::degree_t(0.0)));
    }
   
    // if (result.HasTargets()) {
    //     frc::SmartDashboard::PutString("HAS_TARGETS", "YES");
    // } else {
    //     frc::SmartDashboard::PutString("HAS_TARGETS", "NO");
    // }


    // if (result.HasTargets()) {
    //     photon::PhotonTrackedTarget target = result.GetBestTarget();
    //     frc::Transform3d bestCameraToTarget = target.GetBestCameraToTarget();

    //     double x_vision = bestCameraToTarget.X().value();
    //     double y_vision = bestCameraToTarget.Y().value();

    //     frc::SmartDashboard::PutNumber("PhotonLib Range", sqrt(x_vision * x_vision + y_vision * y_vision));
    // }
}

void Robot::TestInit() {
    TeleopInit();
    
}


void Robot::TestPeriodic() {
    if(a_DriverXboxController.GetBButton()){
        a_NoteHandler.manualClimberDown();
    }
    else if(a_DriverXboxController.GetXButton()){
        a_NoteHandler.manualClimberUp();
    }
    else if(a_DriverXboxController.GetYButton()){
        a_NoteHandler.pidClimb();
    }
    else{
        a_NoteHandler.stopClimber();
    }
    //a_NoteHandler.climbControl(a_DriverXboxController.GetBButton(), a_DriverXboxController.GetXButton());
    // if(a_DriverXboxController.GetLeftBumperPressed()){
    //     rotP+=.001;
    // }
    // else if(a_DriverXboxController.GetRightBumperPressed()){
    //     rotP-=.001;
    // }
    // else if(a_DriverXboxController.GetAButtonPressed()){
    //     rotI+=.001;
    // }
    // else if(a_DriverXboxController.GetBButtonPressed()){
    //     rotI-=.001;
    // }
    // else if(a_DriverXboxController.GetXButtonPressed()){
    //     rotD+=.0001;
    // }
    // else if(a_DriverXboxController.GetYButtonPressed()){
    //     rotD-=.0001;
    // }
    
    
    // a_FLModule.setSteerPID(pvaluesteer, ivaluesteer, dvaluesteer);

    // a_FRModule.setSteerPID(pvaluesteer, ivaluesteer, dvaluesteer);
    
    // a_BLModule.setSteerPID(1.6, ivaluesteer, dvaluesteer);

    // a_BRModule.setSteerPID(pvaluesteer, ivaluesteer, dvaluesteer);

    // frc::SmartDashboard::PutNumber("pvaluesteer", rotP);
    // frc::SmartDashboard::PutNumber("ivaluesteer", rotI);
    // frc::SmartDashboard::PutNumber("dvaluesteer", rotD);

    //  if(a_DriverXboxController.GetRightTriggerAxis() > 0.25) {
    //     a_NoteHandler.runExtension(15.75);
    //     a_NoteHandler.armToPose(150.0);
    //     a_FRModule.steerToAng(0);
    //     a_FLModule.steerToAng(0);
    //     a_BRModule.steerToAng(0);
    //     a_BLModule.steerToAng(0);
    // }
    // else {
    //     a_NoteHandler.runExtension(1.0);
    //     a_NoteHandler.armToPose(245.0);
    //     a_FRModule.steerToAng(45);
    //     a_FLModule.steerToAng(45);
    //     a_BRModule.steerToAng(45);
    //     a_BLModule.steerToAng(45);
    //}
    
    // a_NoteHandler.shootToAmp(a_DriverXboxController.GetRightTriggerAxis() > .75);
     
    // if(a_DriverXboxController.GetAButton()){
    //     if(a_NoteHandler.armToPose(154.0)){
    //         //frc::SmartDashboard::PutString("through if?", "YES");
    //         a_NoteHandler.runArmRoller();
    //     }
    //     else{
    //        // frc::SmartDashboard::PutString("through if?", "NO");
    //     }
    // }
    //frc::SmartDashboard::PutNumber("Current State", a_NoteHandler.currentAmpLoadState);
    // frc::SmartDashboard::PutBoolean("B button state", a_DriverXboxController.GetBButton());
    // if(a_DriverXboxController.GetYButton()){
    //     a_NoteHandler.manualClimberDown();
    // }
    // else if(a_DriverXboxController.GetXButton()){
    //     a_NoteHandler.manualClimberUp();
    // }
    

}

//void Robot::SetTargetType(LED_STAGE_enum target) {
    //target_type = target;
    // if(target_type == target_type_enum::CONE) {
    //     // Set target type to CONE
    //    a_LED.SetTargetType(target_type_enum::CONE);
        
    // } else if(target_type == target_type_enum::CUBE) {
        // Set target type to CUBE
       // a_LED.SetTargetType(target_type_enum::CUBE);
        //a_Claw.CubePressure();

    //}
    //a_LED.SetTargetType(LED_STAGE_enum target);
//}

int main() { return frc::StartRobot<Robot>(); } // Initiate main loop
