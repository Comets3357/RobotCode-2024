#include "Commands/SourceShootCommand.h"

SourceShootCommand::SourceShootCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem) {
    shooterSubsystem = shooter;
    indexerSubsystem = indexer; 
    swerve = swerveSubsystem; 
    AddRequirements({shooter}); 
    SetName("Shoot Command");
    
    cycle = 0; 
}

void SourceShootCommand::Initialize()
{
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
       // if (swerve->GetPose().X > )
    } else {

    }
}

void SourceShootCommand::Execute()
{
    if (cycle == 0) {

    } else if (cycle == 1){
        shooterSubsystem->Pivot.SetPosition(37, shooterSubsystem->offset);
        shooterSubsystem->SetVelocityFlyWheel(1000);
        shooterSubsystem->SetVelocityKickerWheel(1000); 
    } else if (cycle == 2){
       
    } else if (cycle == 3){

    }
    
}

bool SourceShootCommand::IsFinished()
{
    // if (shooterSubsystem->GetVelocityFlyWheel() < shooterSubsystem->FlyWheel.config.velocities["ShooterSpeed"] + 100)
    // {
    //         indexerSubsystem->SetPercent(0.35);
    //         return true;
    // }

    return false;
}

void SourceShootCommand::End(bool interrupted)
{
    
}