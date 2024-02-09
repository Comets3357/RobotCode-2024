#include "Subsystems/ShooterSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem() : COMETS3357::Subsystem("ShooterSubsystem") {

}
    void ShooterSubsystem::Initialize()
    {

    }

void ShooterSubsystem::Periodic(){
    frc::SmartDashboard::PutNumber("Velocity wheels ", KickerWheel.GetRelativeVelocity());
    frc::SmartDashboard::PutNumber("PIvotAbso", GetPivotAbsolutePosition());
    // SetPositionPivot(45);

    frc::SmartDashboard::PutNumber("Pivot ANgle", Pivot.GetAbsolutePosition());
    
    Pivot.Periodic();

    frc::SmartDashboard::PutNumber("ANGLE LOOKUP", angleLookup.GetValue(2));
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

// Manual // 

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
    Pivot.SetPower(percent);
}

void ShooterSubsystem::SetPositionPivot(double position)
{
    Pivot.SetPosition(position); 
}

void ShooterSubsystem::SetPositionPivot(std::string position)
{
    Pivot.SetPosition(position); 
}

double ShooterSubsystem::GetPivotRelativePosition()
{
    return Pivot.GetRelativePosition();
}

double ShooterSubsystem::GetPivotAbsolutePosition()
{
    frc::SmartDashboard::PutNumber("ENCODER VALUE", pivotAbsoluteEncoder.GetOutput());
    return ((double)pivotAbsoluteEncoder.GetOutput() * 360) - 237.0;
}

std::pair<double, double> ShooterSubsystem::calculateDistanceTravelled(std::pair<double, double> velocity, double time)
{
    double x, y;
    x = velocity.first * (time + .1);
    y = velocity.second * (time + .1);
    return {x,y};
}

std::pair<double, double> ShooterSubsystem::calculateFinalPosition(std::pair<double, double> initial, std::pair<double, double> travel)
{
    return {travel.first - initial.first, travel.second - initial.first};
}
