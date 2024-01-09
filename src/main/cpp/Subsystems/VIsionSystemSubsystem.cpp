#include "Subsystems/VisionSystemSubsystem.h"



VisionSystemSubsystem::VisionSystemSubsystem(COMETS3357::SwerveSubsystem* swerve) : COMETS3357::Subsystem("swerveSubsystem")
{
    swerveSubsystem = swerve;
}

void VisionSystemSubsystem::Initialize()
{

}

void VisionSystemSubsystem::Periodic()
{
    double currentTimestamp = subsystemData->GetEntry("Timestamp").GetDouble(0);
    if (currentTimestamp != lastTimestamp)
    {
        lastTimestamp = currentTimestamp;
        frc::Translation2d position{units::meter_t{subsystemData->GetEntry("X").GetDouble(0)}, units::meter_t{subsystemData->GetEntry("Y").GetDouble(0)}};
        frc::Rotation2d rotation{units::radian_t{GetSubsystemData("GyroSubsystem")->GetEntry("angle").GetDouble(0)}};

        frc::Pose2d robotPosition{position, rotation};

        swerveSubsystem->m_odometry.AddVisionMeasurement(robotPosition, units::second_t{currentTimestamp});
    }

}