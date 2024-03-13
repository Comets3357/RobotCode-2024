#include "Subsystems/AmpSubsystem.h"

AmpSubsystem::AmpSubsystem() : COMETS3357::Subsystem("AmpSubsystem") {

}

void AmpSubsystem::Initialize()
{
    // noteDetector.EnableLimitSwitch(true);
}

void AmpSubsystem::Periodic() {

}

void AmpSubsystem::SetPercent(double percent)
{
    AmpMotor.SetPower(percent);
}
