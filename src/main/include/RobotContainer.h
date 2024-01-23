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

#include "Commands/IntakeIndexerCommand.h"
#include "Commands/ShooterCommand.h"

#include "COMETS3357/LookupTable.h"


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
  COMETS3357::SwerveSubsystem swerve{"Swerve"};

  COMETS3357::LookupTable table{"idk"};

  // VisionSystemSubsystem visionSystem{&swerve};
  IntakeSubsystem intake {}; 
  IndexerSubsystem indexer {}; 
  ShooterSubsystem shooter {};

  IntakeIndexerCommand intakeIndexer {&indexer}; 
  ShooterCommand shooterCommand {&shooter, &indexer, &swerve};

  // Instance command

  frc2::InstantCommand shootIndexer{[this](){indexer.SetVelocity("IndexerShootSpeed");}, {&indexer}}; 
  frc2::InstantCommand ejectIndexer{[this](){indexer.SetVelocity("IndexerEjectSpeed");}, {&indexer}}; 

  frc2::InstantCommand stopIntake{[this](){intake.SetPercent(0);}, {&intake}}; 

  frc2::InstantCommand startIntake{[this](){intake.SetPercent("IntakeSpeed"); intakeIndexer.Schedule();}, {&intake}}; 

  frc2::InstantCommand ejectIntake{[this](){intake.SetPercent("EjectSpeed");}, {&intake}}; 

  frc2::InstantCommand stopIndex{[this](){indexer.SetVelocity(0);}, {&indexer}}; 

  frc2::InstantCommand stopShoot{[this](){shooter.SetVelocityKickerWheel(0); indexer.SetPercent(0);}, {&shooter}}; 

  frc2::InstantCommand zeroGyro{[this](){gyro.ZeroGyro();}, {&gyro}};

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> buttonActionMap 
  {
      {"ZeroGyro", std::make_shared<frc2::InstantCommand>(zeroGyro)},
      {"EjectIntake", std::make_shared<frc2::InstantCommand>(ejectIntake)},
      {"StopIntake", std::make_shared<frc2::InstantCommand>(stopIntake)},
      {"StartShoot", std::make_shared<ShooterCommand>(shooterCommand)},
      {"StopShoot", std::make_shared<frc2::InstantCommand>(stopShoot)}
  };


  std::unordered_map<std::string, std::tuple<std::function<void(double, double, double, double)>, frc2::Subsystem*, COMETS3357::Controller::JoystickCommandMode>> joystickActionMap
  {
   {"SwerveDefaultCommand", {[this](auto leftX, auto leftY, auto rightX, auto rightY){swerve.DriveCornerTurning(-units::meters_per_second_t{leftY}, -units::meters_per_second_t{leftX}, -units::radians_per_second_t{rightX});}, &swerve, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"SwerveDefaultCommandDirectional", {[this](auto leftX, auto leftY, auto rightX, auto rightY){swerve.DriveXRotate(-units::meters_per_second_t{leftY}, -units::meters_per_second_t{leftX}, -units::radians_per_second_t{rightX});}, &swerve, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualIntake", {[this](auto leftX, auto leftY, auto rightX, auto rightY){intake.SetPercent(leftY);}, &intake, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"KickerWheel", {[this](auto leftX, auto leftY, auto rightX, auto rightY){shooter.SetPercentKickerWheel(leftY); shooter.SetPercentKickerWheel(rightY);}, &shooter, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}},
   {"ManualIndexer", {[this](auto leftX, auto leftY, auto rightX, auto rightY){indexer.SetPercent(rightY);}, &shooter, COMETS3357::Controller::JoystickCommandMode::JOYSTICK_DEADZONE_COMMAND}}
  };

  std::vector<std::pair<std::string, std::shared_ptr<frc2::Command>>> autonActionMap
  {

  };

  // COMETS3357::ControllerMap controllerMap{buttonActionMap, joystickActionMap, "CompControllerMap", };
  COMETS3357::Autons autos{&swerve, autonActionMap};

  void ConfigureBindings();
};
