// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/Trigger.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <math.h>
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/Auton/Autons.h"
#include "COMETS3357/Configs/ControllerMap.h"
#include "COMETS3357/TimerSubsystem.h"
#include "COMETS3357/GyroSubsystem.h"
#include "COMETS3357/Subsystems/Vision/LimelightSubsystem.h"
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include "Subsystems/VisionSystemSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/IndexerSubsytem.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/LEDsSubsystem.h"
#include "Commands/IntakeIndexerCommand.h"
#include "Commands/ShooterCommand.h"
#include "COMETS3357/LookupTable.h"
#include "commands/LegAvoidanceCommand.h"
#include "commands/SetPointCommand.h"
#include "Subsystems/AutonResetGyroButtonSubsystem.h"
#include "Commands/AmpExtend.h"
#include "Commands/AmpRetract.h"
#include "Commands/ClimbReset.h"
#include "Commands/Climb.h"
#include "Commands/IntakeIndexerAutonCommand.h"
#include "COMETS3357/Auton/AutonPathCommand.h"
#include "Subsystems/AmpSubsystem.h"






/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  void Periodic();


  // Subsystems
  COMETS3357::TimerSubsystem timer{};
  COMETS3357::GyroSubsystem gyro{};
  COMETS3357::LimelightSubsystem limelight{};
  COMETS3357::SwerveSubsystem swerve{"Swerve", &gyro};

  VisionSystemSubsystem visionSystem{&swerve, &gyro};
  IntakeSubsystem intake {}; 
  IndexerSubsystem indexer {}; 
  ShooterSubsystem shooter {&swerve, &gyro};
  ElevatorSubsystem elevator {};
  AmpSubsystem amp {};
  LEDsSubsystem led {&indexer}; 


  // Commands
  IntakeIndexerCommand intakeIndexer {&indexer}; 
  IntakeIndexerAutonCommand intakeIndexerAuton{&indexer};
  ShooterCommand shooterCommand {&shooter, &indexer, &swerve};
  SetPointCommand subWooferSetpoint{&shooter, &indexer, &swerve, 57, 2000};
  SetPointCommand podiumSetPoint{&shooter, &indexer, &swerve, 38.5, 2000};
  SetPointCommand ampSetPoint{&shooter, &indexer, &swerve, 34, 2000};
  AmpExtendCommand ampExtend{&amp, &shooter};
  AmpRetractCommand ampRetract{&amp, &shooter};
  ClimbResetCommand climbReset{&elevator};
  ClimbCommand climb{&elevator, &shooter};
  LegAvoidanceCommand legAvoidance{&swerve};
  AutonGyroResetSubsystem gyroResetButton{&gyro, &led};
  //AutonPathCommand ampAlign{&swerve, 100, 100, frc::Pose2d::Pose2d{frc::Translation2d(units::meter_t{0}, units::meter_t{0}), frc::Rotation2d{0,0}}};
  

  // Instant Commands
  frc2::InstantCommand avoid{[this](){legAvoidance.Schedule();}, {&swerve}}; // test purposes
  frc2::InstantCommand ejectIndexer{[this](){indexer.SetVelocity("IndexerEjectSpeed");}, {&indexer}}; 
  frc2::InstantCommand stopIntake{[this](){intake.SetPercent(0); indexer.SetPercent(0);}, {&intake, &indexer}}; 
  frc2::InstantCommand startIntake{[this](){intake.SetPercent("IntakeSpeed"); intakeIndexer.Schedule();}, {&intake}}; 
  frc2::InstantCommand startIntakeAuto{[this](){intake.SetPercent("IntakeSpeed"); }, {&intake}}; 
  frc2::InstantCommand ejectIntake{[this](){intake.SetPercent("EjectSpeed"); indexer.SetVelocity("IndexerEjectSpeed");}, {&intake}}; 
  frc2::InstantCommand stopIndex{[this](){indexer.SetVelocity(0);}, {&indexer}}; 
  frc2::InstantCommand stopShoot{[this](){shooter.SetVelocityKickerWheel(0); shooter.SetVelocityFlyWheel(0); indexer.SetPercent(0); shooter.SetPositionPivot(40);}, {&shooter}}; 
  frc2::InstantCommand zeroGyro{[this](){gyro.ZeroGyro(); led.gyroZero = true; frc::SmartDashboard::PutBoolean("IsGyroZeroed", led.gyroZero);}, {&gyro}};
  frc2::InstantCommand shoot{[this](){indexer.SetPercent(1);}, {&indexer}};
  frc2::InstantCommand stopTurningTowardsSpeaker{[this](){ shooter.stopTurnToTarget();}, {}};
  frc2::InstantCommand turnTowardsSpeaker{[this](){shooter.startTurnToTarget();}, {}};
  frc2::InstantCommand angleOffsetPositive{[this](){shooter.offset += .5;}, {&shooter}}; 
  frc2::InstantCommand angleOffsetNegative{[this](){shooter.offset -= .5;}, {&shooter}}; 
  frc2::InstantCommand autoSubwooferSetpoint{[this](){shooter.SetPositionPivot(56); shooter.SetVelocityKickerWheel(2000); shooter.SetVelocityFlyWheel(-2000);}, {}};
  frc2::InstantCommand climbRetract{[this](){elevator.SetPosition(0); }, {&elevator}};
  frc2::InstantCommand humanPlayerSignalOn{[this](){led.hpSignal = true;}, {&led}}; 
  frc2::InstantCommand humanPlayerSignalOff{[this](){led.hpSignal = false;}, {&led}}; 
  frc2::InstantCommand ampSignalOn{[this](){led.ampSignal = true;}, {&led}}; 
  frc2::InstantCommand ampSignalOff{[this](){led.ampSignal = false;}, {&led}};
  frc2::InstantCommand pivotRelative{[this](){shooter.pivotInRelative = true; shooter.Pivot.changeRunMode(COMETS3357::SparkMaxPositionRunMode::POSITION_SPARK_MAX_RELATIVE);}, {}};

  // Autonomous Commands
  frc2::InstantCommand piece4AutoSetpoint{[this](){shooter.SetPositionPivot(40), shooter.SetVelocityKickerWheel(2000); shooter.SetVelocityFlyWheel(-2000);}, {}};
  frc2::InstantCommand piece4AutoSetpoint2{[this](){shooter.SetPositionPivot(40), shooter.SetVelocityKickerWheel(2000); shooter.SetVelocityFlyWheel(-2000);}, {}};
  frc2::InstantCommand piece4AutoSetpoint3{[this](){shooter.SetPositionPivot(40.5), shooter.SetVelocityKickerWheel(2000); shooter.SetVelocityFlyWheel(-2000);}, {}};
  frc2::InstantCommand piece4AutoSetpoint4{[this](){shooter.SetPositionPivot(28), shooter.SetVelocityKickerWheel(3000); shooter.SetVelocityFlyWheel(-3000);}, {}};
  frc2::InstantCommand midPiece4AutoSetpoint{[this](){shooter.SetPositionPivot(29), shooter.SetVelocityKickerWheel(2500); shooter.SetVelocityFlyWheel(-2500);}, {}};
  AutonPathCommand ampAlignBlue{&swerve, 0.5, .5, frc::Pose2d{frc::Translation2d(units::meter_t{1.85}, units::meter_t{7.65}), frc::Rotation2d{units::radian_t{-1.57}}}, true, 0, 0};
  AutonPathCommand ampAlignRed{&swerve, 0.5, .5, frc::Pose2d{frc::Translation2d(units::meter_t{14.68}, units::meter_t{7.65}), frc::Rotation2d{units::radian_t{-1.57}}}, true, 0, 0};
  frc2::InstantCommand ampStart{[this](){
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
      ampAlignRed.Schedule(); 
    }
    else
    {
      ampAlignBlue.Schedule(); 
    }
  }, {}};
  frc2::InstantCommand ampCancel{[this](){
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
      ampAlignRed.Cancel(); 
      //swerve.Drive(units::velocity::meters_per_second_t{0}, units::velocity::meters_per_second_t{0}, units::angular_velocity::radians_per_second_t{0}, true, true, 0);
      //swerve.Drive(units::velocity::meters_per_second_t{0}, units::velocity::meters_per_second_t{0}, 0, 0, true, true);
      swerve.DriveXRotate(units::velocity::meters_per_second_t{0}, units::velocity::meters_per_second_t{0}, units::angular_velocity::radians_per_second_t{0}); 
    }
    else
    {
      ampAlignBlue.Cancel(); 
      //swerve.Drive(units::velocity::meters_per_second_t{0}, units::velocity::meters_per_second_t{0}, units::angular_velocity::radians_per_second_t{0}, true, true, 0je j);
      //swerve.Drive(units::velocity::meters_per_second_t{0}, units::velocity::meters_per_second_t{0}, 0, 0, true, true);
      swerve.DriveXRotate(units::velocity::meters_per_second_t{0}, units::velocity::meters_per_second_t{0}, units::angular_velocity::radians_per_second_t{0}); 
    }
  }, {}};
    

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> buttonActionMap 
  {
    {"ZeroGyro", std::make_shared<frc2::InstantCommand>(zeroGyro)},
    {"EjectIntake", std::make_shared<frc2::InstantCommand>(ejectIntake)},
    {"StartIntake", std::make_shared<frc2::InstantCommand>(startIntake)},
    {"StopIntake", std::make_shared<frc2::InstantCommand>(stopIntake)},
    {"StartShoot", std::make_shared<ShooterCommand>(shooterCommand)},
    {"StopShoot", std::make_shared<frc2::InstantCommand>(stopShoot)},
    {"Shoot", std::make_shared<frc2::InstantCommand>(shoot)},
    {"TurnTowardsSpeaker", std::make_shared<frc2::InstantCommand>(turnTowardsSpeaker)},
    {"StopTurnTowardsSpeaker", std::make_shared<frc2::InstantCommand>(stopTurningTowardsSpeaker)},
    {"SubWooferSetpoint", std::make_shared<SetPointCommand>(subWooferSetpoint)},
    {"PodiumSetpoint", std::make_shared<SetPointCommand>(podiumSetPoint)},
    {"AmpSetpoint", std::make_shared<SetPointCommand>(ampSetPoint)},
    {"angleOffsetPositive", std::make_shared<frc2::InstantCommand>(angleOffsetPositive)},
    {"angleOffsetNegative", std::make_shared<frc2::InstantCommand>(angleOffsetNegative)},
    {"AmpExtend", std::make_shared<frc2::SequentialCommandGroup>(ampExtend)},
    {"AmpRetract", std::make_shared<frc2::SequentialCommandGroup>(ampRetract)},
    {"Climb", std::make_shared<ClimbCommand>(climb)},
    {"ClimbRetract", std::make_shared<frc2::InstantCommand>(climbRetract)},
    {"ClimbReset", std::make_shared<ClimbResetCommand>(climbReset)},
    {"humanPlayerSignalOn", std::make_shared<frc2::InstantCommand>(humanPlayerSignalOn)},
    {"humanPlayerSignalOff", std::make_shared<frc2::InstantCommand>(humanPlayerSignalOff)},
    {"ampSignalOn", std::make_shared<frc2::InstantCommand>(ampSignalOn)},
    {"ampSignalOff", std::make_shared<frc2::InstantCommand>(ampSignalOff)},
    {"PivotToRelative", std::make_shared<frc2::InstantCommand>(pivotRelative)},
    {"ampStart", std::make_shared<frc2::InstantCommand>(ampStart)},
    {"ampCancel", std::make_shared<frc2::InstantCommand>(ampCancel)}
    
  };


  std::unordered_map<std::string, std::tuple<std::function<void(double, double, double, double)>, frc2::Subsystem*, COMETS3357::Controller::JoystickCommandMode>> joystickActionMap
  {
   {"SwerveDefaultCommand", {[this](auto leftX, auto leftY, auto rightX, auto rightY){swerve.DriveCornerTurning(-units::meters_per_second_t{leftY}, -units::meters_per_second_t{leftX}, -units::radians_per_second_t{rightX});}, &swerve, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"SwerveDefaultCommandDirectional", {[this](auto leftX, auto leftY, auto rightX, auto rightY){swerve.DriveXRotate(-units::meters_per_second_t{leftY}, -units::meters_per_second_t{leftX}, -units::radians_per_second_t{rightX});}, &swerve, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualIntake", {[this](auto leftX, auto leftY, auto rightX, auto rightY){intake.SetPercent(leftY);}, &intake, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualShoot", {[this](auto leftX, auto leftY, auto rightX, auto rightY){shooter.SetPercentKickerWheel(leftY); shooter.SetPercentFlyWheel(leftX);}, &shooter, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualIndexer", {[this](auto leftX, auto leftY, auto rightX, auto rightY){indexer.SetPercent(rightY);}, &shooter, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualPivot", {[this](auto leftX, auto leftY, auto rightX, auto rightY){shooter.SetPercentPivot(leftY * -0.2); frc::SmartDashboard::PutNumber("PIVOT ANGLE", shooter.GetPivotRelativePosition()); frc::SmartDashboard::PutNumber("PIVOT ANGLE Absolute", shooter.GetPivotAbsolutePosition());}, &shooter, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualElevator", {[this](auto leftX, auto leftAuto, auto rightX, auto rightY){elevator.SetPercent(rightY * 0.5);}, &elevator, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"CentricDriveCommand", {[this](auto leftX, auto leftY, auto rightX, auto rightY){swerve.CentricDrive(-units::meters_per_second_t{leftY}, -units::meters_per_second_t{leftX}, -units::radians_per_second_t{rightX});}, &swerve, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
  };

  std::vector<std::pair<std::string, std::shared_ptr<frc2::Command>>> autonActionMap
  {
    {"TurnTowardsSpeaker", std::make_shared<frc2::InstantCommand>(turnTowardsSpeaker)},
    {"StopTurnTowardsSpeaker", std::make_shared<frc2::InstantCommand>(stopTurningTowardsSpeaker)},
    {"Intake", std::make_shared<frc2::InstantCommand>(startIntakeAuto)},
    {"IntakeIndexer", std::make_shared<IntakeIndexerAutonCommand>(intakeIndexerAuton)},
    {"StartShoot", std::make_shared<ShooterCommand>(shooterCommand)},
    {"4PieceSetpoint", std::make_shared<frc2::InstantCommand>(piece4AutoSetpoint)},
    {"4PieceSetpoint2", std::make_shared<frc2::InstantCommand>(piece4AutoSetpoint2)},
    {"4PieceSetpoint3", std::make_shared<frc2::InstantCommand>(piece4AutoSetpoint3)},
    {"4PieceSetpoint41", std::make_shared<frc2::InstantCommand>(piece4AutoSetpoint4)},
    {"4PieceSetpoint42", std::make_shared<frc2::InstantCommand>(piece4AutoSetpoint4)},
    {"MidPiece4AutoSetpoint", std::make_shared<frc2::InstantCommand>(midPiece4AutoSetpoint)},
    {"1PieceAutoSetpoint", std::make_shared<frc2::InstantCommand>(autoSubwooferSetpoint)},
    {"PodiumSetpoint", std::make_shared<SetPointCommand>(podiumSetPoint)},
    {"AmpSetpoint", std::make_shared<SetPointCommand>(ampSetPoint)},  
    {"StopShoot", std::make_shared<frc2::InstantCommand>(stopShoot)},
    {"Shoot", std::make_shared<frc2::InstantCommand>(shoot)},
    {"StopIntake", std::make_shared<frc2::InstantCommand>(stopIntake)},

  };

  COMETS3357::ControllerMap controllerMap{buttonActionMap, joystickActionMap, "CompControllerMap" };
  COMETS3357::Autons autos{&swerve, autonActionMap};

  void ConfigureBindings();
};
