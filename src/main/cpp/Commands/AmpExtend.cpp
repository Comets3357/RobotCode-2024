
#include "Commands/AmpExtend.h"

AmpExtendCommand::AmpExtendCommand(ShooterSubsystem* shooter, AmpSubsystem* amp) {
    shooterSubsystem = shooter; 
    ampSubsystem = amp;
    AddRequirements({shooter, amp}); 
}

void AmpExtendCommand::Initialize()
{
    shooterSubsystem->SetPositionPivot(51);
    shooterSubsystem->SetVelocityFlyWheel(-1600);
    shooterSubsystem->SetVelocityKickerWheel(1600);
    alreadySetIt = false;
}

void AmpExtendCommand::Execute()
{
    if (shooterSubsystem->Pivot.GetPosition() > 49 && !alreadySetIt)
    {
        ampSubsystem->SetPercent(0.3);
        time = (double)wpi::math::MathSharedStore::GetTimestamp();
        alreadySetIt = true;
    }
}

bool AmpExtendCommand::IsFinished()
{
    return ((double)wpi::math::MathSharedStore::GetTimestamp() > time + 0.5) && alreadySetIt;
}

void AmpExtendCommand::End(bool interrupted)
{
   ampSubsystem->SetPercent(0);
}