#include "Subsystems/VisionSystemSubsystem.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>


VisionSystemSubsystem::VisionSystemSubsystem(COMETS3357::SwerveSubsystem* swerve) : COMETS3357::Subsystem("VisionSubsystem"),  poseEstimator{&swerve->m_odometry}
{
    

    frc::Translation2d position{units::meter_t{subsystemData->GetEntry("X").GetDouble(0)}, units::meter_t{subsystemData->GetEntry("Y").GetDouble(0)}};
    frc::Rotation2d rotation{units::radian_t{GetSubsystemData("GyroSubsystem")->GetEntry("angle").GetDouble(0)}};
    frc::Pose2d robotPosition{position, rotation};

}

void VisionSystemSubsystem::Initialize()
{
    xSub = subsystemData->GetDoubleTopic("X").Subscribe(0.0);
    ySub = subsystemData->GetDoubleTopic("Y").Subscribe(0.0);
    timestampSub = subsystemData->GetDoubleTopic("Timestamp").Subscribe(0.0);
}

void VisionSystemSubsystem::Periodic()
{
    poseEstimator.Periodic();
    double currentTimestamp = subsystemData->GetEntry("Timestamp").GetDouble(0);
    if (currentTimestamp != lastTimestamp)
    {
        lastTimestamp = currentTimestamp;
        frc::Translation2d position{units::meter_t{subsystemData->GetEntry("X").GetDouble(0)}, units::meter_t{subsystemData->GetEntry("Y").GetDouble(0)}};
        frc::Rotation2d rotation{units::radian_t{GetSubsystemData("GyroSubsystem")->GetEntry("angle").GetDouble(0)}};

        frc::Timer::GetFPGATimestamp();

        frc::Pose2d robotPosition{position, rotation};


        frc::Pose2d robot = swerveSubsystem->GetPose();

        poseEstimator.AddPose(robotPosition, nt::Now()-10);


        // if (sqrt(pow((float)(robotPosition.X() - robot.X()), 2.0f) + pow((float)(robotPosition.Y() + robot.Y()), 2)) < 1)
        // {
            // swerveSubsystem->m_odometry.ResetPosition(frc::Rotation2d{units::radian_t{GetSubsystemData("GyroSubsystem")->GetEntry("angle").GetDouble(0)}}, swerveSubsystem->GetPositions(), robotPosition );
        // }

        // swerveSubsystem->m_odometry.AddVisionMeasurement(robotPosition, units::second_t{currentTimestamp});

        
    }

    // Do this in either robot or subsystem init
    frc::SmartDashboard::PutData("Field", &m_field);

    // Do this in either robot periodic or subsystem periodic

    
    m_field.SetRobotPose(poseEstimator.GetPose(GetSubsystemData("GyroSubsystem")->GetEntry("angle").GetDouble(0)));

}