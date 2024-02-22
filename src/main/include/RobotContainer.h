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

#include "Commands/IntakeIndexerCommand.h"
#include "Commands/ShooterCommand.h"

#include "COMETS3357/LookupTable.h"

#include "commands/LegAvoidanceCommand.h"
#include "commands/SetPointCommand.h"
#include "COMETS3357/Auton/AutonPathCommand.h"



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


  //Subsystems
  COMETS3357::TimerSubsystem timer{};
  COMETS3357::GyroSubsystem gyro{};
  COMETS3357::LimelightSubsystem limelight{};
  COMETS3357::SwerveSubsystem swerve{"Swerve", &gyro};

  VisionSystemSubsystem visionSystem{&swerve, &gyro};
  IntakeSubsystem intake {}; 
  IndexerSubsystem indexer {}; 
  ShooterSubsystem shooter {&swerve, &gyro};
  ElevatorSubsystem elevator {};

  IntakeIndexerCommand intakeIndexer {&indexer}; 
  ShooterCommand shooterCommand {&shooter, &indexer, &swerve};

  SetPointCommand subWooferSetpoint{&shooter, &indexer, &swerve, 56, 2000};
  SetPointCommand podiumSetPoint{&shooter, &indexer, &swerve, 38.5, 2000};
  SetPointCommand ampSetPoint{&shooter, &indexer, &swerve, 24, 2000};
  SetPointCommand midFieldSetpoint{&shooter, &indexer, &swerve, 18, 3500};

  // Instance command
  LegAvoidanceCommand legAvoidance{&swerve};

  frc2::InstantCommand avoid{[this](){legAvoidance.Schedule();}, {&swerve}}; // test purposes

  frc2::InstantCommand ejectIndexer{[this](){indexer.SetVelocity("IndexerEjectSpeed");}, {&indexer}}; 

  frc2::InstantCommand stopIntake{[this](){intake.SetPercent(0); indexer.SetPercent(0);}, {&intake, &indexer}}; 

  frc2::InstantCommand startIntake{[this](){intake.SetPercent("IntakeSpeed"); intakeIndexer.Schedule();}, {&intake}}; 
  frc2::InstantCommand slowIntake{[this](){intake.SetPercent(-0.3); }, {&intake}}; 

  frc2::InstantCommand ejectIntake{[this](){intake.SetPercent("EjectSpeed"); indexer.SetVelocity("IndexerEjectSpeed"); shooter.SetVelocityKickerWheel(-3000); shooter.SetVelocityKickerWheel(3000);}, {&intake}}; 

  frc2::InstantCommand stopIndex{[this](){indexer.SetVelocity(0);}, {&indexer}}; 

  frc2::InstantCommand stopShoot{[this](){shooter.SetVelocityKickerWheel(0); shooter.SetVelocityFlyWheel(0); indexer.SetPercent(0);}, {&shooter}}; // shooter.SetPositionPivot(35);}, {&shooter}}; 

  frc2::InstantCommand zeroGyro{[this](){gyro.ZeroGyro();}, {&gyro}};
  frc2::InstantCommand shoot{[this](){indexer.SetPercent(1);}, {&indexer}};

  frc2::InstantCommand ampRampUp{[this](){shooter.SetVelocityKickerWheel(1200); shooter.SetVelocityFlyWheel(-1200);}, {&shooter}};


  frc2::InstantCommand stopTurningTowardsSpeaker{[this](){shooter.stopTurnToTarget();}, {}};
  frc2::InstantCommand turnTowardsSpeaker{[this](){shooter.startTurnToTarget();}, {}};

  frc2::InstantCommand angleOffsetPositive{[this](){shooter.offset += .25;}, {&shooter}}; 
  frc2::InstantCommand angleOffsetNegative{[this](){shooter.offset -= .25;}, {&shooter}}; 
  // frc2::SequentialCommandGroup autoSubwooferShoot{subWooferSetpoint, frc2::WaitCommand{2_s}, shoot, frc2::WaitCommand{0.5_s}, stopShoot};
  frc2::InstantCommand autoSubwooferSetpoint{[this](){shooter.SetPositionPivot(37); shooter.SetVelocityKickerWheel(2000); shooter.SetVelocityFlyWheel(-2000);}, {}};






  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> buttonActionMap 
  {
      {"ZeroGyro", std::make_shared<frc2::InstantCommand>(zeroGyro)},
      {"AVOIDLEG", std::make_shared<frc2::InstantCommand>(avoid)}, // test purposes
      {"EjectIntake", std::make_shared<frc2::InstantCommand>(ejectIntake)},
      {"StartIntake", std::make_shared<frc2::InstantCommand>(startIntake)},
      {"StopIntake", std::make_shared<frc2::InstantCommand>(stopIntake)},
      {"StartShoot", std::make_shared<ShooterCommand>(shooterCommand)},
      {"StopShoot", std::make_shared<frc2::InstantCommand>(stopShoot)},
      {"Shoot", std::make_shared<frc2::InstantCommand>(shoot)},
      {"TurnTowardsSpeaker", std::make_shared<frc2::InstantCommand>(turnTowardsSpeaker)},
      {"StopTurnTowardsSpeaker", std::make_shared<frc2::InstantCommand>(stopTurningTowardsSpeaker)},
      {"SubWooferSetpoint", std::make_shared<SetPointCommand>(subWooferSetpoint)},
      {"MidFieldSetpoint", std::make_shared<SetPointCommand>(midFieldSetpoint)},
      
      {"PodiumSetpoint", std::make_shared<SetPointCommand>(podiumSetPoint)},
      {"AmpSetpoint", std::make_shared<SetPointCommand>(ampSetPoint)},
      {"AmpRampUp", std::make_shared<frc2::InstantCommand>(ampRampUp)},
      {"angleOffsetPositive", std::make_shared<frc2::InstantCommand>(angleOffsetPositive)},
      {"angleOffsetNegative", std::make_shared<frc2::InstantCommand>(angleOffsetNegative)}
  };


  std::unordered_map<std::string, std::tuple<std::function<void(double, double, double, double)>, frc2::Subsystem*, COMETS3357::Controller::JoystickCommandMode>> joystickActionMap
  {
   {"SwerveDefaultCommand", {[this](auto leftX, auto leftY, auto rightX, auto rightY){swerve.DriveCornerTurning(-units::meters_per_second_t{leftY}, -units::meters_per_second_t{leftX}, -units::radians_per_second_t{rightX});}, &swerve, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"SwerveDefaultCommandDirectional", {[this](auto leftX, auto leftY, auto rightX, auto rightY){swerve.DriveXRotate(-units::meters_per_second_t{leftY}, -units::meters_per_second_t{leftX}, -units::radians_per_second_t{rightX});}, &swerve, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualIntake", {[this](auto leftX, auto leftY, auto rightX, auto rightY){intake.SetPercent(leftY);}, &intake, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualShoot", {[this](auto leftX, auto leftY, auto rightX, auto rightY){shooter.SetPercentKickerWheel(leftY); shooter.SetPercentFlyWheel(leftX);}, &shooter, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualIndexer", {[this](auto leftX, auto leftY, auto rightX, auto rightY){indexer.SetPercent(rightY);}, &shooter, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualPivot", {[this](auto leftX, auto leftY, auto rightX, auto rightY){shooter.SetPercentPivot(leftY * -0.2); frc::SmartDashboard::PutNumber("PIVOT ANGLE", shooter.GetPivotRelativePosition()); frc::SmartDashboard::PutNumber("PIVOT ANGLE Absolute", shooter.GetPivotAbsolutePosition());}, &shooter, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualElevator", {[this](auto leftX, auto leftAuto, auto rightX, auto rightY){elevator.SetPercent(rightY * 0.5);}, &elevator, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}}
  };

  frc2::InstantCommand print1{[this](){frc::SmartDashboard::PutNumber("Test", 1);}, {}};
  frc2::InstantCommand print2{[this](){frc::SmartDashboard::PutNumber("Test", 2);}, {}};

std::vector<std::pair<std::string, std::shared_ptr<frc2::Command>>> autonActionMap
  {
    {"Intake", std::make_shared<frc2::InstantCommand>(startIntake)},
    {"idk", std::make_shared<frc2::InstantCommand>(autoSubwooferSetpoint)},
    {"PodiumSetpoint", std::make_shared<SetPointCommand>(podiumSetPoint)},
    {"AmpSetpoint", std::make_shared<SetPointCommand>(ampSetPoint)},  
    {"StopShoot", std::make_shared<frc2::InstantCommand>(stopShoot)},
    {"Shoot", std::make_shared<frc2::InstantCommand>(shoot)},
    {"StopIntake", std::make_shared<frc2::InstantCommand>(stopIntake)},
    {"SlowIntake", std::make_shared<frc2::InstantCommand>(slowIntake)},
    {"print1", std::make_shared<frc2::InstantCommand>(print1)},
    {"print2", std::make_shared<frc2::InstantCommand>(print2)}
  };

  COMETS3357::ControllerMap controllerMap{buttonActionMap, joystickActionMap, "CompControllerMap" };
  COMETS3357::Autons autos{&swerve, autonActionMap};

  void ConfigureBindings();
};
