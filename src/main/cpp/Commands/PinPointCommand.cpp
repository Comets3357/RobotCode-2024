
#include "Commands/PinPointCommand.h"

PinPointCommand::PinPointCommand(IndexerSubsystem *FlyWheel, ShooterSubsystem *ShooterAngle) {
    flyWheel = FlyWheel; 
    shooterAngle = ShooterAngle;
    AddRequirements({FlyWheel},{ShooterAngle}); 
}

void PinPointCommand::Initialize()
{
    if (!flyWheel->IsDetected())
    {
        flyWheel->SetVelocity("flyWheelSpeed"); 
    }
    if (!shooterAngle->IsDetected())
    {
        shooterAngle->SetPosition("shooterAnglePosition"); 
    }
}

void PinPointCommand::Execute()
{

}

bool PinPointCommand::IsFinished()
{
    return flyWheel->IsDetected();
}

void PinPointCommand::End(bool interrupted)
{
    flyWheel->SetVelocity(0); 
    shooterAngle->SetPosition(45);
}