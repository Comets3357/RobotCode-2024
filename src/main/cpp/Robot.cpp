#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "COMETS3357/Configs/ConfigFiles.h"

void Robot::RobotInit() 
{
// ConfigFiles::getInstance().LoadConfigFiles("Comp");
//m_container.timerSubsystem.ResetAndBeginTimer();
COMETS3357::SubsystemManager::GetInstance().Init();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
  frc2::CommandScheduler::GetInstance().Run();
  m_container.Periodic();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {

  // for (auto motor : COMETS3357::ConfigFiles::getInstance().GetConfigFiles().sparkMaxVelocityConfigs)
  // {
  //   motor.second.motor->SetPercent(0);
  // }

  // for (auto motor : COMETS3357::ConfigFiles::getInstance().GetConfigFiles().sparkMaxPercentConfigs)
  // {
  //   motor.second.motor->SetPower(0);
  // }

  // for (auto motor : COMETS3357::ConfigFiles::getInstance().GetConfigFiles().sparkMaxVelocityConfigs)
  // {
  //   motor.second.motor->SetPercent(0);
  // }
}

void Robot::DisabledPeriodic() 
{

}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() 
{
  //m_autonomousCommand = m_container.GetAutonomousCommand();

  m_container.autos.AutonomousInit();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() 
{
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand) 
  {
    m_autonomousCommand->Cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() 
{
  frc2::CommandScheduler::GetInstance().Run();
}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif
