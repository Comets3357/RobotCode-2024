
#include "Commands/TurnToCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

TurnToCommand::TurnToCommand(COMETS3357::SwerveSubsystem* swerveSubsystem, COMETS3357::GyroSubsystem * gyroSubsystem) {
    swerve = swerveSubsystem; 
    gyro = gyroSubsystem;
    // AddRequirements({swerve}); 
}



void TurnToCommand::Initialize()
{
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        targetPos = frc::Translation2d{units::meter_t{16.579}, units::meter_t{5.547867999999999}};
    }
    else
    {
        targetPos = frc::Translation2d{units::meter_t{0}, units::meter_t{5.547867999999999}};
    }
}

void TurnToCommand::Execute()
{
   frc::Pose2d robotPosition = swerve->GetPose();
   units::meter_t deltaX = robotPosition.X() - targetPos.X();
   units::meter_t deltaY = robotPosition.Y() - targetPos.Y();

   double angle = atan2((double)deltaX, (double)deltaY);
   swerve->controllingSwerveRotation = false;
   turnToPID.SetP(1);
   swerve->overrideRotation = units::radians_per_second_t{turnToPID.Calculate(gyro->GetSubsystemData("GyroSubsystem")->GetEntry("angle").GetDouble(0), angle)};//rotationPLookup.GetValue(0);
}

bool TurnToCommand::IsFinished()
{
    return false;
}

void TurnToCommand::End(bool interrupted)
{
    swerve->controllingSwerveRotation = true;
    swerve->overrideRotation = units::radians_per_second_t{0};
}