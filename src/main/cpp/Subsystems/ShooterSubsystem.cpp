#include "Subsystems/ShooterSubsystem.h"

Shootersubsystem::ShooterSubsystem() : COMETS3357::Subsystem("ShooterSubsystem") {

}
    void ShooterSubsystem::Initialize()
    {

    }

void ShooterSubsystem::Periodic(){

}

void ShooterSubsystem::SetVelocityFlyWheel(double velocity)
{
    FlyMotor.SetVelocity(velocity);
}
void ShooterSubsystem::SetVelocityFlyWheel(std::string velocity)
{
    FlyMotor.SetVelocity(velocity);
}

void ShooterSubsystem::SetVelocityKickerWheel(double velocity)
{
    KickerMotor.SetVelocity(velocity);
}

void ShooterSubsystem::SetVelocityKickerWheel(std::string velocity)
{
    KickerMotor.SetVelocity(velocity);
}
