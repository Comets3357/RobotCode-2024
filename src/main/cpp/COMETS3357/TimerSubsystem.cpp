#include "COMETS3357/TimerSubsystem.h"

using namespace COMETS3357;

TimerSubsystem::TimerSubsystem() : COMETS3357::Subsystem("TimerSubsystem")
{
    ResetAndBeginTimer();
}


void TimerSubsystem::Periodic()
{
    
    timeSinceEnabled = timer.Get();

    frc::SmartDashboard::PutNumber("Time", (double)timeSinceEnabled);

    deltaTime = timeSinceEnabled - lastTime;
    frc::SmartDashboard::PutNumber("TicksPerSecond", 1.0/(double)deltaTime);
    frc::SmartDashboard::PutNumber("TicksadsPerSecond", (double)deltaTime);

    subsystemData->GetEntry("timeSinceEnabled").SetDouble((double)timeSinceEnabled);
    subsystemData->GetEntry("deltaTime").SetDouble((double)deltaTime);
    subsystemData->GetEntry("lastTime").SetDouble((double)lastTime);

    lastTime = timeSinceEnabled;
}

void TimerSubsystem::ResetAndBeginTimer()
{
    
    timer.Reset();
    timer.Start();
}