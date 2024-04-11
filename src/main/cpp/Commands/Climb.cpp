
#include "Commands/Climb.h"

ClimbCommand::ClimbCommand(ElevatorSubsystem* elevator, ShooterSubsystem* shooter) {
    elevatorSubsystem = elevator;
    shooterSubsystem = shooter;
    AddRequirements({elevator, shooter}); 
}

void ClimbCommand::Initialize()
{
    shooterSubsystem->SetPositionPivot(80);
}

void ClimbCommand::Execute()
{
    if (shooterSubsystem->Pivot.GetPosition() > 75)
    {
        elevatorSubsystem->SetPosition(-135);
    }
}

bool ClimbCommand::IsFinished()
{
    if (shooterSubsystem->Pivot.GetPosition() > 75)
    {
        elevatorSubsystem->SetPosition(-135);
        return true;
    }
    return false;
}

void ClimbCommand::End(bool interrupted)
{
   
}