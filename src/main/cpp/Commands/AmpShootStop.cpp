
#include "Commands/AmpShootStop.h"

AmpShootStopCommand::AmpShootStopCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator, IndexerSubsystem* indexer) {
    shooterSubsystem = shooter; 
    elevatorSubsystem = elevator;
    indexerSubsystem = indexer;
    AddRequirements({shooter, elevator, indexer}); 
}

void AmpShootStopCommand::Initialize()
{
    elevatorSubsystem->SetPosition(0);
    shooterSubsystem->SetVelocityFlyWheel(0);
    shooterSubsystem->SetVelocityKickerWheel(0);
}

void AmpShootStopCommand::Execute()
{
    if (elevatorSubsystem->elevatorMotor.GetPosition() > -15)
    {
        shooterSubsystem->SetPositionPivot(35);
        indexerSubsystem->SetPercent(0);
    }
}

bool AmpShootStopCommand::IsFinished()
{
    if (elevatorSubsystem->elevatorMotor.GetPosition() > -15)
    {
        shooterSubsystem->SetPositionPivot(35);
        indexerSubsystem->SetPercent(0);
        return true;
    }
    return false;
}

void AmpShootStopCommand::End(bool interrupted)
{
   
}