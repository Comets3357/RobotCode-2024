#include "Subsystems/VisionSystemSubsystem.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


VisionSystemSubsystem::VisionSystemSubsystem(COMETS3357::SwerveSubsystem* swerve, COMETS3357::GyroSubsystem* gyro) : COMETS3357::Subsystem("VisionSubsystem"), gyroSubsystem{gyro}, swerveSubsystem{swerve}//, poseEstimator{&swerve->m_odometry}
{
    
}

void VisionSystemSubsystem::Initialize()
{

    
    // prepare subscribers
    tagSub = subsystemData->GetDoubleArrayTopic("TagData").Subscribe({});
    timePublisher = subsystemData->GetDoubleTopic("Time").Publish();
    epochPublisher = subsystemData->GetDoubleTopic("Epoch").Publish();
    timePublisher.SetDefault(0.0);




    // send time data to the pi
    subsystemData->GetEntry("Time").SetDouble((double)wpi::math::MathSharedStore::GetTimestamp());
    auto currentTime = std::chrono::system_clock::now();
    subsystemData->GetEntry("Epoch").SetDouble((double)(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count()) / 1000.0);

    swerveSubsystem->ResetOdometry(frc::Pose2d{frc::Translation2d{units::meter_t{0}, units::meter_t{0}}, frc::Rotation2d{units::radian_t{0}}});
}

void VisionSystemSubsystem::Periodic()
{


    subsystemData->GetEntry("Time").SetDouble((double)wpi::math::MathSharedStore::GetTimestamp(), (double)wpi::math::MathSharedStore::GetTimestamp());
    // double currentTimestamp = subsystemData->GetEntry("Timestamp").GetDouble(0);
    frc::SmartDashboard::PutNumber("TImesd", (double)wpi::math::MathSharedStore::GetTimestamp());


    timePublisher.Set((double)wpi::math::MathSharedStore::GetTimestamp());
    frc::SmartDashboard::PutData("Fielsd", &m_field2);

    // populate gyro history
    gyroValues.insert(gyroValues.end(), {(double)wpi::math::MathSharedStore::GetTimestamp(), (-gyroSubsystem->m_navx.GetYaw() * 3.14159 / 180.0) + ((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroSubsystem->angleOffset});

    // limit gyro history size
    if (gyroValues.size() > 100)
    {
        gyroValues.erase(gyroValues.begin());
    }
    
    // gets the tag data 
    nt::TimestampedDoubleArray tagDataBufferTimestamped = tagSub.GetAtomic();
    
    std::vector<double> tagDataBuffer = tagDataBufferTimestamped.value;
    





    if (tagDataBuffer.size() >= 4 && tagDataBuffer[3] > lastTimestamp+ 0.001)
    {
        lastTimestamp = tagDataBuffer[3];
        
        uint64_t deltaTime = nt::Now() - tagDataBufferTimestamped.time;
    
    units::time::second_t time = wpi::math::MathSharedStore::GetTimestamp() - units::time::second_t{deltaTime * 0.000001f};
    currentTimestamp = (double)time;
        frc::SmartDashboard::PutNumber("Timestamp Delta", deltaTime);
        frc::SmartDashboard::PutNumber("timestamp not accurate", tagDataBuffer[3]);

        frc::SmartDashboard::PutNumber("CAMERA ANGLE OFFSET", tagDataBuffer[2]);
        frc::SmartDashboard::PutNumber("CAMERA DISTANCE", tagDataBuffer[1]);


        for (int i = 0; i < tagDataBuffer.size(); i+=4)
        {
            std::pair<double, double> gyroAbove = *gyroValues.upper_bound({currentTimestamp, std::numeric_limits<double>::lowest()});
            std::pair<double, double> gyroBelow = *std::prev(gyroValues.lower_bound({currentTimestamp, std::numeric_limits<double>::lowest()}));

            double slope = (gyroAbove.second - gyroBelow.second)/(gyroAbove.first - gyroBelow.first);
            double gyroAngle = ((currentTimestamp - gyroBelow.first) * slope) + gyroBelow.second;

            double angleOffset = tagDataBuffer[i + 2];
            double tagDistance = tagDataBuffer[i + 1];
            int ID = (int)tagDataBuffer[i];

            double actualAngleOffset =  (-gyroSubsystem->m_navx.GetYaw() * 3.14159 / 180.0) - angleOffset +((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroSubsystem->angleOffset;
            double x = cos(actualAngleOffset) * tagDistance + tagPositions[ID].first;
            double y = sin(actualAngleOffset) * tagDistance + tagPositions[ID].second;

            double positionStandardDev = (tagDistance * 0.04) + 0.3 + (angleOffset * 0.6);
            swerveSubsystem->m_odometry.SetVisionMeasurementStdDevs({positionStandardDev, positionStandardDev, positionStandardDev/2});

            double cameraX = -0.241 * ((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 1 : -1);
            double cameraY = -0.2617 * ((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 1 : -1);
            double cameraDistance = sqrt(pow(cameraX, 2) + pow(cameraY, 2));
            double angle = atan2(cameraX, cameraY);
            double newAngle = angle + 3.14159;
            frc::Translation2d newPos{units::meter_t{x + cameraDistance * cos(newAngle)}, units::meter_t(y + cameraDistance * sin(newAngle))};
            frc::Rotation2d newRotation{units::radian_t{gyroAngle}};


            if (abs(gyroSubsystem->m_navx.GetRate()) < 0.5)
            swerveSubsystem->m_odometry.AddVisionMeasurement(frc::Pose2d{newPos, newRotation}, units::second_t{currentTimestamp});
            // m_field2.SetRobotPose(frc::Pose2d{newPos, newRotation});
        }
        
    }

    frc::SmartDashboard::PutData("Fielsd", &m_field2);

    frc::SmartDashboard::PutData("Field", &m_field);

    m_field.SetRobotPose(swerveSubsystem->m_odometry.GetEstimatedPosition());

}