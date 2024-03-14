
#include "Commands/AmpExtend.h"

AmpExtendCommand::AmpExtendCommand(ShooterSubsystem* shooter, AmpSubsystem* amp) {
    shooterSubsystem = shooter; 
    ampSubsystem = amp;
    AddRequirements({shooter, amp}); 
}

void AmpExtendCommand::Initialize()
{
    shooterSubsystem->SetPositionPivot(50);
    shooterSubsystem->SetVelocityFlyWheel(-1400);
    shooterSubsystem->SetVelocityKickerWheel(1400);
}

void AmpExtendCommand::Execute()
{
    if (shooterSubsystem->Pivot.GetPosition() > 52 && !alreadySetIt)
    {
        ampSubsystem->SetPercent(0.3);
        time = (double)wpi::math::MathSharedStore::GetTimestamp();
        alreadySetIt = true;
    }
}

bool AmpExtendCommand::IsFinished()
{
    return (double)wpi::math::MathSharedStore::GetTimestamp() > time + 1;
}

void AmpExtendCommand::End(bool interrupted)
{
   ampSubsystem->SetPercent(0);
}