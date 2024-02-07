#include "Subsystems/VisionSystemSubsystem.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>


VisionSystemSubsystem::VisionSystemSubsystem(COMETS3357::SwerveSubsystem* swerve) : COMETS3357::Subsystem("VisionSubsystem"), swerveSubsystem{swerve}//, poseEstimator{&swerve->m_odometry}
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
    rotationSpeedSub = subsystemData->GetDoubleTopic("RotVelocity").Subscribe(0.0);
    velocitySub = subsystemData->GetDoubleTopic("Velocity").Subscribe(0.0);
    distanceSub = subsystemData->GetDoubleTopic("Distance").Subscribe(0.0);
    timePublisher = subsystemData->GetDoubleTopic("Time").Publish();
    timePublisher.SetDefault(0.0);
}

void VisionSystemSubsystem::Periodic()
{
    // poseEstimator.Periodic();
    double currentTimestamp = subsystemData->GetEntry("Timestamp").GetDouble(0);
    frc::SmartDashboard::PutNumber("TImesd", (double)wpi::math::MathSharedStore::GetTimestamp());
    subsystemData->GetEntry("Time").SetDouble((double)wpi::math::MathSharedStore::GetTimestamp());

    timePublisher.Set((double)wpi::math::MathSharedStore::GetTimestamp());
    frc::SmartDashboard::PutData("Fielsd", &m_field2);
    

    if (currentTimestamp != lastTimestamp)
    {
        double rotationSpeed = subsystemData->GetEntry("RotVelocity").GetDouble(0);
        double velocity = subsystemData->GetEntry("Velocity").GetDouble(0);
        double tagDistance = subsystemData->GetEntry("Distance").GetDouble(0);



        lastTimestamp = currentTimestamp;
        // frc::SmartDashboard::PutData("Fielsd", &m_field2);
        frc::Translation2d position{units::meter_t{subsystemData->GetEntry("X").GetDouble(0)}, units::meter_t{subsystemData->GetEntry("Y").GetDouble(0)}};
        frc::Rotation2d rotation{units::radian_t{GetSubsystemData("GyroSubsystem")->GetEntry("angle").GetDouble(0)}};

      
        frc::Pose2d robotPosition{position, rotation};
        frc::SmartDashboard::PutNumber("TIME IDK", wpi::Now());

        
        frc::Pose2d robot = swerveSubsystem->GetPose();
        m_field2.SetRobotPose(robotPosition);

        if (ResetPose)
        {
            swerveSubsystem->ResetOdometry(robotPosition);
            ResetPose = false;
        }

        // poseEstimator.AddPose(robotPosition, wpi::Now()-60);//subsystemData->GetEntry("Timestamp").GetDouble(0));


        // if (sqrt(pow((float)(robotPosition.X() - robot.X()), 2.0f) + pow((float)(robotPosition.Y() + robot.Y()), 2)) < 1)
        // {
            // swerveSubsystem->m_odometry.ResetPosition(frc::Rotation2d{units::radian_t{GetSubsystemData("GyroSubsystem")->GetEntry("angle").GetDouble(0)}}, swerveSubsystem->GetPositions(), robotPosition );
        // }
        double positionStandardDev = (tagDistance * 0.2) + (rotationSpeed * 1) + (velocity * 0.1);
        swerveSubsystem->m_odometry.SetVisionMeasurementStdDevs({positionStandardDev, positionStandardDev, positionStandardDev/2});

        double cameraX = -0.25;
        double cameraY = 0.3;
        double cameraDistance = sqrt(pow(cameraX, 2) + pow(cameraY, 2));
        double angle = atan2(cameraX, cameraY);
        double newAngle = angle + subsystemData->GetEntry("Yaw").GetDouble(0);
        frc::Translation2d newPos{robotPosition.X() + units::meter_t{cameraDistance * cos(newAngle)}, robotPosition.Y() + units::meter_t(cameraDistance * sin(newAngle))};
        frc::Rotation2d newRotation{units::radian_t{subsystemData->GetEntry("Yaw").GetDouble(0)}};


        if (abs(rotationSpeed) < 0.5)
        swerveSubsystem->m_odometry.AddVisionMeasurement(frc::Pose2d{newPos, newRotation}, units::second_t{currentTimestamp-0.25});

        
    }

    // Do this in either robot or subsystem init
    frc::SmartDashboard::PutData("Field", &m_field);

    // // Do this in either robot periodic or subsystem periodic

    
    m_field.SetRobotPose(swerveSubsystem->m_odometry.GetEstimatedPosition());

}