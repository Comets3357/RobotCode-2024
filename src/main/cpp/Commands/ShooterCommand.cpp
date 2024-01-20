
#include "Commands/ShooterCommand.h"

ShooterCommand::ShooterCommand(ShooterSubsystem *shooter) {
    ShooterSubsytem = shooter; 
    AddRequirements({shooter}); 
}

void ShooterCommand::Initialize()
{
    ShooterSubsytem->SetVelocityFlyWheel("ShooterSpeed"); 

}

void ShooterCommand::Execute()
{
    if (ShooterSubsytem->GetVelocityKickerWheel() > ShooterSubsytem->KickerWheel.config.velocities["ShooterSpeed"] - 20)
    {
        ShooterSubsytem->SetVelocityFlyWheel("ShooterSpeed");
    }
    if (ShooterSubsytem->GetVelocityFlyWheel() > ShooterSubsytem->FlyWheel.config.velocities["ShooterSpeed"] - 20)
    {
        ShooterSubsytem->SetVelocityFlyWheel("ShooterSpeed");
    }
}

bool ShooterCommand::IsFinished()
{
    
}

void ShooterCommand::End(bool interrupted)
{
    ShooterSubsytem->SetVelocityFlyWheel(0); 
}