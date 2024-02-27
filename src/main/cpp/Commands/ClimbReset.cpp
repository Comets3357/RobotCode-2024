
#include "Commands/ClimbReset.h"
#include <frc/smartdashboard/SmartDashboard.h>

ClimbResetCommand::ClimbResetCommand(ElevatorSubsystem* elevator) {
    elevatorSubsystem = elevator;
    AddRequirements({elevator}); 
}

void ClimbResetCommand::Initialize()
{
    elevatorSubsystem->elevatorMotor.SetPower(0.25);
}

void ClimbResetCommand::Execute()
{
    //frc::SmartDashboard::PutBoolean("Limit reached", elevatorSubsystem->ElevatorLimit.Get());
    if (elevatorSubsystem->ElevatorLimit.Get())
    {
        elevatorSubsystem->elevatorMotor.SetRelativeEncoderPosition(0);
    }
}

bool ClimbResetCommand::IsFinished()
{
    //frc::SmartDashboard::PutBoolean("Limit reached", elevatorSubsystem->ElevatorLimit.Get());
    if (elevatorSubsystem->ElevatorLimit.Get())
    {
        elevatorSubsystem->elevatorMotor.SetRelativeEncoderPosition(0);
        return true;
    }
    return false;
}

void ClimbResetCommand::End(bool interrupted)
{
   
}