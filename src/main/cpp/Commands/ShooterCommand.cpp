
#include "Commands/ShooterCommand.h"


ShooterCommand::ShooterCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer) {
    shooterSubsystem = shooter;
    indexerSubsystem = indexer; 
    AddRequirements({shooter, indexer}); 
}

void ShooterCommand::Initialize()
{
    shooterSubsystem->SetVelocityFlyWheel("ShooterSpeed"); 
    shooterSubsystem->SetVelocityKickerWheel("ShooterSpeed"); 

}

void ShooterCommand::Execute()
{
    if (shooterSubsystem->GetVelocityKickerWheel() > shooterSubsystem->KickerWheel.config.velocities["ShooterSpeed"] - 20 && shooterSubsystem->GetVelocityFlyWheel() > shooterSubsystem->FlyWheel.config.velocities["ShooterSpeed"] - 20)
    {
            indexerSubsystem->SetVelocity("ShooterSpeed");

    }

}

bool ShooterCommand::IsFinished()
{
    
}

void ShooterCommand::End(bool interrupted)
{
    shooterSubsystem->SetVelocityFlyWheel(0); 
}