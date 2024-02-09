
#include "Commands/ShooterCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

ShooterCommand::ShooterCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem) {
    shooterSubsystem = shooter;
    indexerSubsystem = indexer; 
    swerve = swerveSubsystem; 
    AddRequirements({shooter}); 
}

void ShooterCommand::Initialize()
{
    shooterSubsystem->SetVelocityFlyWheel("ShooterSpeed"); 
    shooterSubsystem->SetVelocityKickerWheel("ShooterSpeed"); 
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        targetPos = frc::Translation2d{units::meter_t{16.579}, units::meter_t{5.5}};
    }
    else
    {
        targetPos = frc::Translation2d{units::meter_t{0}, units::meter_t{5.5}};
    }
}

void ShooterCommand::Execute()
{
    frc::Pose2d pos = swerve->GetPose(); 
    double distance = sqrt(pow((double)(targetPos.X() - pos.X()), 2) + pow((double)(targetPos.Y() - pos.Y()), 2)) + 0.25; 
    frc::SmartDashboard::PutNumber("Distance From Target", distance);
    double shooterAngle = shooterSubsystem->angleLookup.GetValue(distance);
    double velocity = shooterSubsystem->velocityLookup.GetValue(distance);
    shooterSubsystem->Pivot.SetPosition(shooterAngle); 
    shooterSubsystem->SetVelocityKickerWheel(velocity);
    shooterSubsystem->SetVelocityFlyWheel(-velocity);

    frc::SmartDashboard::PutNumber("FLywheel Velocity", shooterSubsystem->GetVelocityFlyWheel());
    // if (shooterSubsystem->GetVelocityFlyWheel() < shooterSubsystem->FlyWheel.config.velocities["ShooterSpeed"] + 100)
    // {

    //         indexerSubsystem->SetPercent(0.35);
    //         // return true;
    // }
}

bool ShooterCommand::IsFinished()
{
    // if (shooterSubsystem->GetVelocityFlyWheel() < shooterSubsystem->FlyWheel.config.velocities["ShooterSpeed"] + 100)
    // {
    //         indexerSubsystem->SetPercent(0.35);
    //         return true;
    // }

    return false;
}

void ShooterCommand::End(bool interrupted)
{
    
}