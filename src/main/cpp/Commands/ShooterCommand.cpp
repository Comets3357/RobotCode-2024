
#include "Commands/ShooterCommand.h"

ShooterCommand::ShooterCommand(ShooterSubsystem *shooter) {
    ShooterSubsytem = shooter; 
    AddRequirements({shooter}); 
}

void ShooterCommand::Initialize()
{
    ShooterSubsytem->SetVelocityFlyWheel("ShooterSpeed"); 
    ShooterSubsytem->SetVelocityKickerWheel("ShooterSpeed"); 

}

void ShooterCommand::Execute()
{
    if (ShooterSubsytem->GetVelocityKickerWheel() > ShooterSubsytem->KickerWheel.config.velocities["ShooterSpeed"] - 20 && ShooterSubsytem->GetVelocityFlyWheel() > ShooterSubsytem->FlyWheel.config.velocities["ShooterSpeed"] - 20)
    {
        ShooterSubsytem->SetVelocityFlyWheel(1);
        ShooterSubsytem->SetVelocityKickerWheel(1);
        ShooterSubsytem->SetVelocityFlyWheel("ShooterSpeed");
        ShooterSubsytem->SetVelocityKickerWheel("ShooterSpeed"); 
        
    }
    else
    {
        ShooterSubsytem->SetVelocityFlyWheel(0);
        ShooterSubsytem->SetVelocityKickerWheel(0);
    }

}

bool ShooterCommand::IsFinished()
{
    
}

void ShooterCommand::End(bool interrupted)
{
    ShooterSubsytem->SetVelocityFlyWheel(0); 
}