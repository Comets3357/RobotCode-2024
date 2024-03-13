
#include "Commands/AmpRetract.h"

AmpRetractCommand::AmpRetractCommand(AmpSubsystem* amp, ShooterSubsystem* shooter) {
    AddCommands(
        frc2::InstantCommand{[amp](){amp->SetPercent(-0.3);}, {amp}},
        frc2::InstantCommand{[shooter](){shooter->SetVelocityFlyWheel(0);}, {shooter}},
        frc2::InstantCommand{[shooter](){shooter->SetVelocityKickerWheel(0);}, {shooter}},
        frc2::WaitCommand{1_s},
        frc2::InstantCommand{[amp](){amp->SetPercent(0);}, {amp}},
        frc2::InstantCommand{[shooter](){shooter->SetPercentPivot(35);}, {shooter}}
    );
}