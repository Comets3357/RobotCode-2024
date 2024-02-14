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

    swerveSubsystem->ResetOdometry(frc::Pose2d{frc::Translation2d{units::meter_t{0}, units::meter_t{0}}, frc::Rotation2d{units::radian_t{0}}});
}

void VisionSystemSubsystem::Periodic()
{


    timePublisher.Set((double)wpi::math::MathSharedStore::GetTimestamp());
    frc::SmartDashboard::PutData("Fielsd", &m_field2);

    // populate gyro history
    yawInterpolationBuffer.AddSample(wpi::math::MathSharedStore::GetTimestamp(), (-gyroSubsystem->m_navx.GetYaw() * 3.14159 / 180.0) + ((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroSubsystem->angleOffset);
    rateInterpolationBuffer.AddSample(wpi::math::MathSharedStore::GetTimestamp(), gyroSubsystem->m_navx.GetRate());
    
    std::vector<double> tagDataBuffer = tagSub.GetAtomic().value;

frc::SmartDashboard::PutNumber("ASDASD TIME", nt::Now() * 0.000001);

    if (tagDataBuffer.size() > 1 && tagDataBuffer[3] > lastTimestamp)
    {
        lastTimestamp = tagDataBuffer[3];
        
        uint64_t deltaTime = nt::Now() - tagSub.GetAtomic().time;

        
    
        units::time::second_t time = wpi::math::MathSharedStore::GetTimestamp() - units::time::second_t{(nt::Now() - tagSub.GetAtomic().time) * 0.000001f};
        currentTimestamp = (double)time;
        frc::SmartDashboard::PutNumber("Timestamp Delta", deltaTime);

        for (int i = 0; i < tagDataBuffer.size(); i+=4)
        {
            std::optional<double> gyroAngleOptional = yawInterpolationBuffer.Sample(time);
            std::optional<double> gyroRateOptional = rateInterpolationBuffer.Sample(time);

            double gyroAngle = gyroAngleOptional.has_value() ? gyroAngleOptional.value() : 0;
            double gyroRate = gyroRateOptional.has_value() ? gyroRateOptional.value() : 0;

            double angleOffset = tagDataBuffer[i + 2];
            double tagDistance = tagDataBuffer[i + 1];
            int ID = (int)tagDataBuffer[i];

            double actualAngleOffset =  (-gyroSubsystem->m_navx.GetYaw() * 3.14159 / 180.0) - angleOffset +((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroSubsystem->angleOffset;
            double x = cos(actualAngleOffset) * tagDistance + tagPositions[ID].first;
            double y = sin(actualAngleOffset) * tagDistance + tagPositions[ID].second;

            double positionStandardDev = (tagDistance * 0.04) + 0.3 + abs(gyroRate);
            swerveSubsystem->m_odometry.SetVisionMeasurementStdDevs({positionStandardDev, positionStandardDev, positionStandardDev/2});

            double cameraX = -0.241 * ((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 1 : -1);
            double cameraY = -0.2617 * ((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 1 : -1);
            double cameraDistance = sqrt(pow(cameraX, 2) + pow(cameraY, 2));
            double angle = atan2(cameraX, cameraY);
            double newAngle = angle + 3.14159;
            frc::Translation2d newPos{units::meter_t{x + cameraDistance * cos(newAngle)}, units::meter_t(y + cameraDistance * sin(newAngle))};
            frc::Rotation2d newRotation{units::radian_t{gyroAngle}};


            if (abs(gyroRate) < 0.5)
            swerveSubsystem->m_odometry.AddVisionMeasurement(frc::Pose2d{newPos, newRotation}, units::second_t{currentTimestamp});
            // m_field2.SetRobotPose(frc::Pose2d{newPos, newRotation});

            if (ID == 3)
            {
                m_field3.SetRobotPose(frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}}, newRotation});
            }
            else if (ID == 4)
            {
                m_field4.SetRobotPose(frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}}, newRotation});
            }
            else if (ID == 5)
            {
                m_field5.SetRobotPose(frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}}, newRotation});
            }
        }
        
    }

    frc::SmartDashboard::PutData("Fielsd", &m_field2);

    frc::SmartDashboard::PutData("Field", &m_field);



    frc::SmartDashboard::PutData("FieldTag3", &m_field3);
    frc::SmartDashboard::PutData("FieldTag4", &m_field4);
    frc::SmartDashboard::PutData("FieldTag5", &m_field5);

    m_field.SetRobotPose(swerveSubsystem->m_odometry.GetEstimatedPosition());

}