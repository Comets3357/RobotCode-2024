
#include "Commands/AmpShoot.h"

AmpShootCommand::AmpShootCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator) {
    shooterSubsystem = shooter; 
    elevatorSubsystem = elevator;
    AddRequirements({shooter, elevator}); 
}

void AmpShootCommand::Initialize()
{
    shooterSubsystem->SetPositionPivot(68);
    shooterSubsystem->SetVelocityFlyWheel(-825);
    shooterSubsystem->SetVelocityKickerWheel(825);
}

void AmpShootCommand::Execute()
{
    if (shooterSubsystem->Pivot.GetPosition() > 65)
    {
        elevatorSubsystem->SetPosition(-96);
    }
}

bool AmpShootCommand::IsFinished()
{
    if (shooterSubsystem->Pivot.GetPosition() > 65)
    {
        elevatorSubsystem->SetPosition(-96);
        return true;
    }
    return false;
}

void AmpShootCommand::End(bool interrupted)
{
   
}