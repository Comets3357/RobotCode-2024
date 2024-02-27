#include "COMETS3357/GyroSubsystem.h"
#include <thread>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

using namespace COMETS3357;

GyroSubsystem::GyroSubsystem() : COMETS3357::Subsystem("GyroSubsystem"), m_navx{frc::SPI::Port::kMXP}
{
    m_navx.Calibrate();
}

void GyroSubsystem::Periodic()
{
    if (!m_navx.IsCalibrating())
    {
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
        subsystemData->GetEntry("angle").SetDouble((-m_navx.GetAngle() * 3.14159 / 180) + angleOffset);
        }
        else
        {
            subsystemData->GetEntry("angle").SetDouble((-m_navx.GetAngle() * 3.14159 / 180) + angleOffset + 3.14159);
        }
        subsystemData->GetEntry("angleRate").SetDouble(m_navx.GetRate() * 3.14159 / 180);
    }
  
    m_navx.UpdateDisplacement(0, 9.81, 500, true);

    //frc::SmartDashboard::PutNumber("Robot Displacement X", m_navx.GetDisplacementX());
    //frc::SmartDashboard::PutNumber("Robot Displacement Y", m_navx.GetDisplacementY());
    //frc::SmartDashboard::PutNumber("Robot Displacement Z", m_navx.GetDisplacementZ());
}

void GyroSubsystem::ZeroGyro()
{
    angleOffset = m_navx.GetAngle() * 3.14159 / 180;
}