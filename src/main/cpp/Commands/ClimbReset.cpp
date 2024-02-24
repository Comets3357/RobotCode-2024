
#include "Commands/ClimbReset.h"
#include <frc/smartdashboard/SmartDashboard.h>

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
    frc::SmartDashboard::PutBoolean("Limit reached", elevatorSubsystem->ElevatorLimit.Get());
}

bool ClimbResetCommand::IsFinished()
{
    frc::SmartDashboard::PutBoolean("Limit reached", elevatorSubsystem->ElevatorLimit.Get());
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