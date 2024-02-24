#include "Subsystems/AutonResetGyroButtonSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

AutonGyroResetSubsystem::AutonGyroResetSubsystem(frc2::InstantCommand* gyroReset) : COMETS3357::Subsystem("AutonGyroResetSubsystem") {
    resetCommand = gyroReset;
}

void AutonGyroResetSubsystem::Initialize()
{

}

void AutonGyroResetSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean("Button", resetButton.Get());
    // if (!resetButton.Get())
    // {
    //     resetCommand->Schedule();
    // }
}