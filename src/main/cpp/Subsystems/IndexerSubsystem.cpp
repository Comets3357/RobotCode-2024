#include "Subsystems/IndexerSubsytem.h"
#include <frc/smartdashboard/SmartDashboard.h>
IndexerSubsystem::IndexerSubsystem() : COMETS3357::Subsystem("IndexerSubsystem") {

}

void IndexerSubsystem::Initialize()
{

}

void IndexerSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean("ISDETECTED", IsDetected());
}

void IndexerSubsystem::SetPercent(double percent)
{
    IndexerMotor.SetPercent(percent);
}

void IndexerSubsystem::SetVelocity(double velocity) {
    IndexerMotor.SetVelocity(velocity); 
}

void IndexerSubsystem::SetVelocity(std::string velocity) {
    IndexerMotor.SetVelocity(velocity); 
}

bool IndexerSubsystem::IsDetected()
{
    return noteDetector.Get(); 
}