#include "Subsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() : COMETS3357::Subsystem("ElevatorSubsystem") {

}

void ElevatorSubsystem::Initialize()
{

}

void ElevatorSubsystem::Periodic() {

}

void ElevatorSubsystem::SetPercent(double percent) {
    elevatorMotor.SetPower(percent); 
}

void ElevatorSubsystem::SetPosition(double position)
{
    elevatorMotor.SetPosition(position);
}

void ElevatorSubsystem::SetPosition(std::string position)
{
    elevatorMotor.SetPosition(position);
}