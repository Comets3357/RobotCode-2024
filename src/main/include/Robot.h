// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include "RobotContainer.h"
#include <rev/CANSparkMax.h>
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/Configs/ConfigFiles.h"
#include "COMETS3357/Subsystems/SubsystemManager.h"
#include "COMETS3357/CometsRobot/CometsRobot.h"



class Robot : public COMETS3357::CometsRobot {
 public:

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:

  RobotContainer m_container;
};
