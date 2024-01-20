#include "Subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() : COMETS3357::Subsystem("ShooterSubsystem") {

}
    void ShooterSubsystem::Initialize()
    {

    }

void ShooterSubsystem::Periodic(){

}

void ShooterSubsystem::SetVelocityFlyWheel(double velocity)
{
    FlyWheel.SetVelocity(velocity);
}
void ShooterSubsystem::SetVelocityFlyWheel(std::string velocity)
{
    FlyWheel.SetVelocity(velocity);
}

void ShooterSubsystem::SetVelocityKickerWheel(double velocity)
{
    KickerWheel.SetVelocity(velocity);
}

void ShooterSubsystem::SetVelocityKickerWheel(std::string velocity)
{
    KickerWheel.SetVelocity(velocity);
}

// Manuel // 

void ShooterSubsystem::SetPercentFlyWheel(double percent) 
{
    FlyWheel.SetPercent(percent);
}

void ShooterSubsystem::SetPercentKickerWheel (double percent)
{
    KickerWheel.SetPercent(percent);
}

double ShooterSubsystem::GetVelocityKickerWheel()
{
    return KickerWheel.GetRelativeVelocity();
}

double ShooterSubsystem::GetVelocityFlyWheel()
{
    return FlyWheel.GetRelativeVelocity();
}

void ShooterSubsystem::SetPercentPivot(double percent) 
{
    Pivot.SetPosition(percent);
}

void ShooterSubsystem::SetPositionPivot(double position)
{
    Pivot.SetPosition(position); 
}

void ShooterSubsystem::SetPositionPivot(std::string position)
{
    Pivot.SetPosition(position); 
}


