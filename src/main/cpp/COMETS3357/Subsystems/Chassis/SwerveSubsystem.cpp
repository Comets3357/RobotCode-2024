// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include "COMETS3357/utils/SwerveUtils.h"

#include <frc/smartdashboard/SmartDashboard.h>
using namespace COMETS3357;

SwerveSubsystem::SwerveSubsystem(std::string configFileName, COMETS3357::GyroSubsystem* gyro)
    : configuration{ConfigFiles::getInstance().GetConfigFiles().swerveConfigs[configFileName]},
      COMETS3357::Subsystem("SwerveSubsystem"),
      gyroSubsystemData{GetSubsystemData("GyroSubsystem")},
        gyroSubsystem{gyro},
      m_frontLeft{configuration.frontLeftModule},
      m_rearLeft{configuration.backLeftModule},
      m_frontRight{configuration.frontRightModule},
      m_rearRight{configuration.backRightModule},
      m_magLimiter{configuration.magnitudeSlewRate / 1_s},
      m_rotLimiter{configuration.rotationalSlewRate / 1_s},
      kDriveKinematics{
      frc::Translation2d{configuration.wheelBase / 2,
                         configuration.trackWidth / 2}, // front right
      frc::Translation2d{configuration.wheelBase / 2,
                         -configuration.trackWidth / 2}, // back right
      frc::Translation2d{-configuration.wheelBase / 2,
                         configuration.trackWidth / 2}, // front left
      frc::Translation2d{-configuration.wheelBase / 2,
                         -configuration.trackWidth / 2}}, // back left

      kDriveKinematicsFrontLeft{
      frc::Translation2d{configuration.wheelBase,
                         units::length::meter_t{0}}, // front right
      frc::Translation2d{configuration.wheelBase,
                         -configuration.trackWidth}, // back right
      frc::Translation2d{units::length::meter_t{0},
                         units::length::meter_t{0}}, // front left
      frc::Translation2d{units::length::meter_t{0},
                         -configuration.trackWidth}}, // back left

      kDriveKinematicsFrontRight{
      frc::Translation2d{units::length::meter_t{0},
                         units::length::meter_t{0}}, // front right
      frc::Translation2d{units::length::meter_t{0},
                         -configuration.trackWidth}, // back right
      frc::Translation2d{-configuration.wheelBase,
                         units::length::meter_t{0}}, // front left
      frc::Translation2d{-configuration.wheelBase,
                         -configuration.trackWidth}}, // back left

      kDriveKinematicsBackLeft{
      frc::Translation2d{configuration.wheelBase,
                         configuration.trackWidth}, // front right
      frc::Translation2d{configuration.wheelBase,
                         units::length::meter_t{0}}, // back right
      frc::Translation2d{units::length::meter_t{0},
                         configuration.trackWidth}, // front left
      frc::Translation2d{units::length::meter_t{0},
                         units::length::meter_t{0}}}, // back left

      kDriveKinematicsBackRight{
      frc::Translation2d{units::length::meter_t{0},
                         configuration.trackWidth}, // front right
      frc::Translation2d{units::length::meter_t{0},
                         units::length::meter_t{0}}, // back right
      frc::Translation2d{-configuration.wheelBase,
                         configuration.trackWidth}, // front left
      frc::Translation2d{-configuration.wheelBase,
                         units::length::meter_t{0}}}, // back left

      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::radian_t{gyroSubsystemData->GetEntry("angle").GetDouble(0)}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}},
      m_odometry2{kDriveKinematics,
                 frc::Rotation2d(units::radian_t{gyroSubsystemData->GetEntry("angle").GetDouble(0)}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}}
{

    currentKinematic = &kDriveKinematics;
}

void SwerveSubsystem::Initialize()
{
  
}

frc::Pose2d SwerveSubsystem::GetPose2()
{
  return m_odometry2.GetPose();
}

void SwerveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.

  m_frontLeft.Periodic();
  m_frontRight.Periodic();
  m_rearLeft.Periodic();
  m_rearRight.Periodic();

    //frc::SmartDashboard::PutNumber("SWERVEX", (double)m_odometry.GetEstimatedPosition().X());
  //frc::SmartDashboard::PutNumber("SWERVEY", (double)m_odometry.GetEstimatedPosition().Y());



  m_odometry.Update(frc::Rotation2d(units::radian_t{gyroSubsystemData->GetEntry("angle").GetDouble(0)}),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
  m_odometry2.Update(frc::Rotation2d(units::radian_t{gyroSubsystemData->GetEntry("angle").GetDouble(0)}),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});

  //frc::SmartDashboard::PutData("Fielsd2 real", &m_field);

  m_field.SetRobotPose(m_odometry2.GetPose());

  
}

void SwerveSubsystem::Drive(units::meters_per_second_t xSpeed,
              units::meters_per_second_t ySpeed, double directionX, double directionY,
              bool fieldRelative, bool rateLimit)
{

  frc::SmartDashboard::PutNumber("Gyro Angle", gyroSubsystemData->GetEntry("angle").GetDouble(0));
  //frc::SmartDashboard::PutNumber("Angle Difference", gyroSubsystemData->GetEntry("angle").GetDouble(0) - lastAngle);


  double gyroAngle = gyroSubsystemData->GetEntry("angle").GetDouble(0);

  if (gyroAngle - lastAngle > 3.14159)
  {
    actualAngle += (gyroAngle - lastAngle) - (3.14159 * 2);
  }
  else if (gyroAngle - lastAngle < -3.14159)
  {
    actualAngle += (gyroAngle - lastAngle) + (3.14159 * 2);
  }
  else
  {
    actualAngle += gyroAngle - lastAngle;
  }

  //frc::SmartDashboard::PutNumber("ActualRotation", actualAngle);

  lastAngle = gyroAngle;

  

  double targetAngle = (atan2(directionX, directionY)) + (actualAngle - fmod(actualAngle, (3.14159 * 2)));

  if (targetAngle - actualAngle >= 3.14159)
  {
    targetAngle -= (3.14159 * 2);
  }
  else if (targetAngle - actualAngle <= -3.14159)
  {
    targetAngle += (3.14159 * 2);
  }

  double speed = sqrt(pow(directionX, 2) + pow(directionY, 2));
  // rotationController.SetIntegratorRange(-5, 5);



  units::radians_per_second_t rot = units::radians_per_second_t{std::clamp(rotationController.Calculate(actualAngle, targetAngle), -speed, speed)};

  //frc::SmartDashboard::PutNumber("ROTATION AMMOUNT", (double)rot);

  double xSpeedCommanded;
  double ySpeedCommanded;

  if (rateLimit) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    double inputTranslationMag =
        sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate =
          abs(configuration.directionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0;  // some high number that means the slew rate
                                  // is effectively instantaneous
    }

    double currentTime = wpi::Now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils::AngleDifference(inputTranslationDir,
                                                   m_currentTranslationDir);
    if (angleDif < 0.45 * std::numbers::pi) {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
    } else if (angleDif > 0.85 * std::numbers::pi) {
      if (m_currentTranslationMag >
          1e-4) {  // some small number to avoid floating-point errors with
                   // equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.Calculate(0.0);
      } else {
        m_currentTranslationDir =
            SwerveUtils::WrapAngle(m_currentTranslationDir + std::numbers::pi);
        m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
      }
    } else {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(0.0);
    }
    m_prevTime = currentTime;

    xSpeedCommanded = m_currentTranslationMag * cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * sin(m_currentTranslationDir);
    // m_currentRotation = m_rotLimiter.Calculate(rot.value());

  } else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    m_currentRotation = rot.value();
  }

  if (speed > 0.25)
  {
    m_currentRotation = rot.value();
  }
  else
  {
    m_currentRotation = 0;
  }
  

  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeedCommanded * configuration.maxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeedCommanded * configuration.maxSpeed;
  units::radians_per_second_t rotDelivered =
      m_currentRotation * configuration.maxTurnSpeed;

  auto states = currentKinematic->ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{gyroSubsystemData->GetEntry("angle").GetDouble(0)}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  currentKinematic->DesaturateWheelSpeeds(&states, configuration.maxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void SwerveSubsystem::CentricDrive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot) {
  double xSpeedCommanded;
  double ySpeedCommanded;

 bool fieldRelative = false;
 bool rateLimit = true; 



  

  if (rateLimit) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    double inputTranslationMag =
        sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate =
          abs(configuration.directionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0;  // some high number that means the slew rate
                                  // is effectively instantaneous
    }

    double currentTime = wpi::Now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils::AngleDifference(inputTranslationDir,
                                                   m_currentTranslationDir);
    if (angleDif < 0.45 * std::numbers::pi) {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
    } else if (angleDif > 0.85 * std::numbers::pi) {
      if (m_currentTranslationMag >
          1e-4) {  // some small number to avoid floating-point errors with
                   // equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.Calculate(0.0);
      } else {
        m_currentTranslationDir =
            SwerveUtils::WrapAngle(m_currentTranslationDir + std::numbers::pi);
        m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
      }
    } else {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(0.0);
    }
    m_prevTime = currentTime;

    xSpeedCommanded = m_currentTranslationMag * cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.Calculate(rot.value());

  } else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    m_currentRotation = rot.value();
  }

  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeedCommanded * configuration.maxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeedCommanded * configuration.maxSpeed;
  units::radians_per_second_t rotDelivered =
      m_currentRotation * configuration.maxTurnSpeed;

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{gyroSubsystemData->GetEntry("angle").GetDouble(0)}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  kDriveKinematics.DesaturateWheelSpeeds(&states, configuration.maxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

frc::ChassisSpeeds SwerveSubsystem::getSpeeds()
{
  return currentKinematic->ToChassisSpeeds({m_frontLeft.GetState(), m_frontRight.GetState(), m_rearLeft.GetState(), m_rearRight.GetState()});
}

void SwerveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, bool fieldRelative,
                           bool rateLimit,
                           frc::SwerveDriveKinematics<4>* kinematics) {
  double xSpeedCommanded;
  double ySpeedCommanded;



  

  if (rateLimit) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    double inputTranslationMag =
        sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate =
          abs(configuration.directionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0;  // some high number that means the slew rate
                                  // is effectively instantaneous
    }

    double currentTime = wpi::Now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils::AngleDifference(inputTranslationDir,
                                                   m_currentTranslationDir);
    if (angleDif < 0.45 * std::numbers::pi) {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
    } else if (angleDif > 0.85 * std::numbers::pi) {
      if (m_currentTranslationMag >
          1e-4) {  // some small number to avoid floating-point errors with
                   // equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.Calculate(0.0);
      } else {
        m_currentTranslationDir =
            SwerveUtils::WrapAngle(m_currentTranslationDir + std::numbers::pi);
        m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
      }
    } else {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(0.0);
    }
    m_prevTime = currentTime;

    xSpeedCommanded = m_currentTranslationMag * cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.Calculate(rot.value());

  } else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    m_currentRotation = rot.value();
  }

  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeedCommanded * configuration.maxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeedCommanded * configuration.maxSpeed;
  units::radians_per_second_t rotDelivered =
      m_currentRotation * configuration.maxTurnSpeed;

  auto states = kinematics->ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{gyroSubsystemData->GetEntry("angle").GetDouble(0)}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  kinematics->DesaturateWheelSpeeds(&states, configuration.maxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void SwerveSubsystem::SetX() {
  m_frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void SwerveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  currentKinematic->DesaturateWheelSpeeds(&desiredStates,
                                         configuration.maxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);

}

wpi::array<frc::SwerveModulePosition, 4U> SwerveSubsystem::GetPositions()
{
  return {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_rearLeft.GetPosition(), m_rearRight.GetPosition()};
}

void SwerveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

void SwerveSubsystem::SetChassisSpeed(frc::ChassisSpeeds chassisSpeed)
{
    wpi::array<frc::SwerveModuleState, 4> desiredStates = currentKinematic->ToSwerveModuleStates(chassisSpeed);

    SetModuleStates(desiredStates);

}

units::degree_t SwerveSubsystem::GetHeading() const {
  return frc::Rotation2d(units::radian_t{gyroSubsystemData->GetEntry("angle").GetDouble(0)}).Degrees();
}

void SwerveSubsystem::ZeroHeading() { }//m_gyro.Reset(); }

double SwerveSubsystem::GetTurnRate() { return -gyroSubsystemData->GetEntry("angleRate").GetDouble(0); }

frc::Pose2d SwerveSubsystem::GetPose() { return m_odometry.GetEstimatedPosition(); }

frc::Pose2d SwerveSubsystem::GetMovingPose(double time)
{
  
  frc::Pose2d robotPose = m_odometry.GetEstimatedPosition();
  double deltaTime = (double)wpi::math::MathSharedStore::GetTimestamp() - lastTime;
  frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(getSpeeds(), robotPose.Rotation());
  frc::Pose2d returnPose = frc::Pose2d{frc::Translation2d{robotPose.X() + units::meter_t{(((double)robotPose.X() - (double)lastDeltaPose.X())/deltaTime) * time},robotPose.Y() +  units::meter_t{(((double)robotPose.Y() - (double)lastDeltaPose.Y())/deltaTime) * time}}, frc::Rotation2d{units::radian_t{0}} };
  lastTime = (double)wpi::math::MathSharedStore::GetTimestamp();
  lastDeltaPose = m_odometry.GetEstimatedPosition();

  return frc::Pose2d{frc::Translation2d{robotPose.X() + units::meter_t{(double)fieldRelativeSpeeds.vx * time},robotPose.Y() +  units::meter_t{(double)fieldRelativeSpeeds.vy * time}}, frc::Rotation2d{units::radian_t{0}} };
}

void SwerveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
  m_odometry2.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

void SwerveSubsystem::DriveXRotate(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot)
{
  //frc::SmartDashboard::PutBoolean("controllingSwerveRotation", controllingSwerveRotation);
  if (!controllingSwerveRotation )
  {
    rot = overrideRotation;
  }
  // if (!controllingSwerveMovement)
  // {
  //   xSpeed = overrideXSpeed;
  //   ySpeed = overrideYSpeed;
  // }
  // if (addingSwerveRotation)
  // {
  //   rot += addingRot;
  // }
  // if (addingSwerveMovement)
  // {
  //   xSpeed += addingXSpeed;
  //   ySpeed += addingYSpeed;
  // }
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
  {
    ySpeed *= -1;
    xSpeed *= -1;
  }

  currentKinematic = &kDriveKinematics;
  Drive(xSpeed, ySpeed, rot, true, true, &kDriveKinematics);
  pickedCorner = false;
}

void SwerveSubsystem::DriveDirectionalRotate(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, double directionX, double directionY)
{
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
  {
    ySpeed *= -1;
    xSpeed *= -1;
  }
  
  currentKinematic = &kDriveKinematics;
  Drive(xSpeed, ySpeed, directionX, directionY, true, true);
  pickedCorner = false;
}

void SwerveSubsystem::DriveCornerTurning(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot)
{
  //frc::SmartDashboard::PutBoolean("controllingSwerveRotation", controllingSwerveRotation);
  if (!controllingSwerveRotation )
  {
    rot = overrideRotation;
  }
  // if (!controllingSwerveMovement)
  // {
  //   xSpeed = overrideXSpeed;
  //   ySpeed = overrideYSpeed;
  // }
  // if (addingSwerveRotation)
  // {
  //   rot += addingRot;
  // }
  // if (addingSwerveMovement)
  // {
  //   xSpeed += addingXSpeed;
  //   ySpeed += addingYSpeed;
  // }
  
  double angleOnDrivebase = atan2(ySpeed.value(), xSpeed.value()) - ((-gyroSubsystem->m_navx.GetAngle() * 3.14159 / 180) + gyroSubsystem->angleOffset);
    double angleXPortion = sin(angleOnDrivebase);
    double angleYPortion = cos(angleOnDrivebase);
    //frc::SmartDashboard::PutNumber("drivebase heading", angleOnDrivebase * 180 / 3.14159);
    if (angleXPortion <= 0 && angleYPortion >= 0)
    {
      currentKinematic = &kDriveKinematicsFrontLeft;
    }
    else if (angleXPortion >= 0 && angleYPortion >= 0)
    {
      currentKinematic = &kDriveKinematicsFrontRight;
    }
    else if (angleXPortion <= 0 && angleYPortion <= 0)
    {
      currentKinematic = &kDriveKinematicsBackLeft;
    }
    else if (angleXPortion >= 0 && angleYPortion <= 0)
    {
      currentKinematic = &kDriveKinematicsBackRight;
    }

      if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
  {
    ySpeed *= -1;
    xSpeed *= -1;
  }

  Drive(xSpeed, ySpeed, rot, true, true, currentKinematic);

  
}