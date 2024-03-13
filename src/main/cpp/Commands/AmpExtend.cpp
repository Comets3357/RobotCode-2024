
#include "Commands/AmpExtend.h"

AmpExtendCommand::AmpExtendCommand(AmpSubsystem* amp, ShooterSubsystem* shooter) {
    AddCommands(
        frc2::InstantCommand{[shooter](){shooter->SetPercentPivot(50);}, {shooter}},
        frc2::InstantCommand{[shooter](){shooter->SetVelocityFlyWheel(-1400);}, {shooter}},
        frc2::InstantCommand{[shooter](){shooter->SetVelocityKickerWheel(1400);}, {shooter}},
        frc2::WaitUntilCommand([shooter](){return shooter->GetPivotAbsolutePosition() > 47;}),
        frc2::InstantCommand{[amp](){amp->SetPercent(0.3);}, {amp}},
        frc2::WaitCommand{1_s},
        frc2::InstantCommand{[amp](){amp->SetPercent(0);}, {amp}}
    );
}