
#include "Commands/AmpShootStop.h"

AmpShootStopCommand::AmpShootStopCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator) {
    shooterSubsystem = shooter; 
    elevatorSubsystem = elevator;
    AddRequirements({shooter, elevator}); 
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
    }
}

bool AmpShootStopCommand::IsFinished()
{
    if (elevatorSubsystem->elevatorMotor.GetPosition() > -15)
    {
        shooterSubsystem->SetPositionPivot(35);
        return true;
    }
    return false;
}

void AmpShootStopCommand::End(bool interrupted)
{
   
}