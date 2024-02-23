
#include "Commands/ClimbReset.h"

ClimbResetCommand::ClimbResetCommand(ElevatorSubsystem* elevator) {
    elevatorSubsystem = elevator;
    AddRequirements({elevator}); 
}

void ClimbResetCommand::Initialize()
{
    elevatorSubsystem->elevatorMotor.SetPower(0.1);
}

void ClimbResetCommand::Execute()
{
    if (elevatorSubsystem->ElevatorLimit.Get())
    {
        elevatorSubsystem->SetPosition(0);
    }
}

bool ClimbResetCommand::IsFinished()
{
    if (elevatorSubsystem->ElevatorLimit.Get())
    {
        elevatorSubsystem->SetPosition(0);
        return true;
    }
    return false;
}

void ClimbResetCommand::End(bool interrupted)
{
   
}