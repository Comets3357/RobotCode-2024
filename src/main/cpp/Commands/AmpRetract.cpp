
#include "Commands/AmpRetract.h"

AmpRetractCommand::AmpRetractCommand(ShooterSubsystem* shooter, AmpSubsystem* amp) {
    shooterSubsystem = shooter; 
    ampSubsystem = amp;
    AddRequirements({shooter, amp}); 
}

void AmpRetractCommand::Initialize()
{
    ampSubsystem->SetPercent(-0.3);
    time = (double)wpi::math::MathSharedStore::GetTimestamp();
    shooterSubsystem->SetVelocityFlyWheel(0);
    shooterSubsystem->SetVelocityKickerWheel(0);
}

void AmpRetractCommand::Execute()
{
    if((double)wpi::math::MathSharedStore::GetTimestamp() > time + 0.5) {
        ampSubsystem->SetPercent(0);
        shooterSubsystem->SetPositionPivot(35);
    }
}

bool AmpRetractCommand::IsFinished()
{
    if((double)wpi::math::MathSharedStore::GetTimestamp() > time + 0.5) {
        ampSubsystem->SetPercent(0);
        shooterSubsystem->SetPositionPivot(35);
        return true;
    }
    return false;
}

void AmpRetractCommand::End(bool interrupted)
{
   
}