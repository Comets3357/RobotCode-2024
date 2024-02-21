
#include "Commands/AmpCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

AmpCommand::AmpCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem) {
    shooterSubsystem = shooter;
    indexerSubsystem = indexer; 
    swerve = swerveSubsystem; 
    AddRequirements({shooter}); 
}

void AmpCommand::Initialize()
{

}

void AmpCommand::Execute()
{

    shooterSubsystem->Pivot.SetPosition(110); 

    if (shooterSubsystem->Pivot.GetPosition() > 95)
    {
        indexerSubsystem->SetPercent(1);
    }

    frc::SmartDashboard::PutNumber("FLywheel Velocity", shooterSubsystem->GetVelocityFlyWheel());
    // if (shooterSubsystem->GetVelocityFlyWheel() < shooterSubsystem->FlyWheel.config.velocities["ShooterSpeed"] + 100)
    // {

    //         indexerSubsystem->SetPercent(0.35);
    //         // return true;
    // }
}

bool AmpCommand::IsFinished()
{
    // if (shooterSubsystem->GetVelocityFlyWheel() < shooterSubsystem->FlyWheel.config.velocities["ShooterSpeed"] + 100)
    // {
    //         indexerSubsystem->SetPercent(0.35);
    //         return true;
    // }

    return shooterSubsystem->Pivot.GetPosition() > 100;
}

void AmpCommand::End(bool interrupted)
{
    
}