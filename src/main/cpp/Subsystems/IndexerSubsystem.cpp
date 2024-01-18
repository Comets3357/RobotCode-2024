#include "Subsystems/IndexerSubsytem.h"

IndexerSubsystem::IndexerSubsystem() : COMETS3357::Subsystem("IndexerSubsystem") {

}

void IndexerSubsystem::Initialize()
{

}

void IndexerSubsystem::Periodic() {

}

void IndexerSubsystem::SetVelocity(double velocity) {
    IndexerMotor.SetVelocity(velocity); 
}

bool IndexerSubsystem::IsDetected()
{
    return noteDetector.Get(); 
}