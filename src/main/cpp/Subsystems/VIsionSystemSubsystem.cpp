#include "Subsystems/VisionSystemSubsystem.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>



VisionSystemSubsystem::VisionSystemSubsystem(COMETS3357::SwerveSubsystem* swerve) : COMETS3357::Subsystem("VisionSubsystem")
{
    swerveSubsystem = swerve;
}

void VisionSystemSubsystem::Initialize()
{
    xSub = subsystemData->GetDoubleTopic("X").Subscribe(0.0);
    ySub = subsystemData->GetDoubleTopic("Y").Subscribe(0.0);
    timestampSub = subsystemData->GetDoubleTopic("Timestamp").Subscribe(0.0);
}

void VisionSystemSubsystem::Periodic()
{
    double currentTimestamp = subsystemData->GetEntry("Timestamp").GetDouble(0);
    if (currentTimestamp != lastTimestamp)
    {
        lastTimestamp = currentTimestamp;
        frc::Translation2d position{units::meter_t{subsystemData->GetEntry("X").GetDouble(0)}, units::meter_t{subsystemData->GetEntry("Y").GetDouble(0)}};
        frc::Rotation2d rotation{units::radian_t{GetSubsystemData("GyroSubsystem")->GetEntry("angle").GetDouble(0)}};

        frc::Timer::GetFPGATimestamp();

        frc::Pose2d robotPosition{position, rotation};

        swerveSubsystem->m_odometry.AddVisionMeasurement(robotPosition, units::second_t{currentTimestamp});
    }

    nt::DoublePublisher xPub;
    frc::SmartDashboard::PutNumber("Robot X", xSub.Get());
    frc::SmartDashboard::PutNumber("Robot Y", ySub.Get());
    frc::SmartDashboard::PutNumber("ASD HEHE", i++);

}