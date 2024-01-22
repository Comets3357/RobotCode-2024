#include "Subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() : COMETS3357::Subsystem("IntakeSubsystem") {

}

void IntakeSubsystem::Initialize()
{

}

void IntakeSubsystem::Periodic() {

}

void IntakeSubsystem::SetPercent(double percent) {
    intakeMotor.SetPower(percent); 
}

void IntakeSubsystem::SetPercent(std::string percent) {
    intakeMotor.SetPower(percent); 
}