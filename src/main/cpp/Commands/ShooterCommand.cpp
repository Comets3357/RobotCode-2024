#pragma once
#include "Commands/ShooterCommand.h"


ShooterCommand::ShooterCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem) {
    shooterSubsystem = shooter;
    indexerSubsystem = indexer; 
    swerve = swerveSubsystem; 
    AddRequirements({shooter, indexer, swerve}); 
}

void ShooterCommand::Initialize()
{
    shooterSubsystem->SetVelocityFlyWheel("ShooterSpeed"); 
    shooterSubsystem->SetVelocityKickerWheel("ShooterSpeed"); 
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        targetPos = frc::Translation2d{units::meter_t{0}, units::meter_t{5.5}};
    }
    else
    {
        targetPos = frc::Translation2d{units::meter_t{16.5}, units::meter_t{5.5}};
    }
}

void ShooterCommand::Execute()
{
    frc::Pose2d pos = swerve->GetPose(); 
    double distance = sqrt(pow((double)(targetPos.X() - pos.X()), 2) + pow((double)(targetPos.Y() - pos.Y()), 2)); 
    double angle = shooterSubsystem->Table.GetValue(distance); 
    shooterSubsystem->SetPositionPivot(angle); 
}

bool ShooterCommand::IsFinished()
{
    if (shooterSubsystem->GetVelocityKickerWheel() > shooterSubsystem->KickerWheel.config.velocities["ShooterSpeed"] - 20 && shooterSubsystem->GetVelocityFlyWheel() > shooterSubsystem->FlyWheel.config.velocities["ShooterSpeed"] - 20)
    {
            indexerSubsystem->SetVelocity("ShooterSpeed");
            return true;
    }

    return false;
}

void ShooterCommand::End(bool interrupted)
{
    
}