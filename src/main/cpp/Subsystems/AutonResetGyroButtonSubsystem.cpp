#include "Subsystems/AutonResetGyroButtonSubsystem.h"

AutonGyroResetSubsystem::AutonGyroResetSubsystem(frc2::InstantCommand* gyroReset) : COMETS3357::Subsystem("AutonGyroResetSubsystem") {
    resetCommand = gyroReset;
}

void AutonGyroResetSubsystem::Initialize()
{

}

void AutonGyroResetSubsystem::Periodic() {
    if (resetButton.Get())
    {
        resetCommand->Schedule();
    }
}