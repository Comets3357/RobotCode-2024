#include "COMETS3357/PoseEstimator.h"
#include <iostream>
#define MOVEMENTFRAMECOUNT 1000

using namespace COMETS3357;

PoseEstimator::PoseEstimator(frc::SwerveDriveOdometry<4>* odometry) : m_odometry{odometry}
{
}

void PoseEstimator::AddPose(frc::Pose2d pose, double timestamp)
{
    frc::Pose2d pos = m_odometry->GetPose();
    offset = offset + (RobotPose{(double)pose.X(), (double)pose.Y(), (double)pose.Rotation().Radians()} - (offset + RobotPose{(double)pos.X(), (double)pos.Y(), (double)pos.Rotation().Radians()}))/RobotPose{10.0, 10.0, 10.0};

    if (startup)
    {
        offset = RobotPose{(double)pose.X(), (double)pose.Y(), (double)pose.Rotation().Radians()};
        startup = false;
    }

    // int top = 1;
    // for (int i = 1; i < 45; i++)
    // {
    //     if (i < map.size()-1)
    //     {
    //        // std::puts("IDKasldjhakjsdhkahsdkashdjkahskdhakjsdhkjashdkjasd");
    //         if (timestamp < get<0>(map[map.size()-i]))
    //         {
    //             std::puts("IDKasldjhakjsdhkahsdkashdjkahskdhakjsdhkjashdkjasd");
    //             top = map.size()-i;
    //             break;
    //         }
    //     }
    //     else
    //     {
    //         return;
    //     }
    // }
    // if (top > 1)
    // {
    //     double deltaTime = get<0>(map[top]) - get<0>(map[top-1]);
        
    //     lastPoseDelta = (calculatedPoseDelta * RobotPose{(double)poseAdjustmentCount, (double)poseAdjustmentCount, (double)poseAdjustmentCount} / RobotPose{MOVEMENTFRAMECOUNT, MOVEMENTFRAMECOUNT, MOVEMENTFRAMECOUNT}) + lastPoseDelta;
    //     calculatedPoseDelta = ((RobotPose{(double)pose.X(), (double)pose.Y(), (double)pose.Rotation().Radians()} - lastPoseDelta) - (((get<1>(map[top]) - get<1>(map[top-1]))/RobotPose{deltaTime, deltaTime, deltaTime})*RobotPose{timestamp, timestamp, timestamp})+get<1>(map[top-1]));
    //     map.erase(map.begin(), map.begin() + (top-2));
    //     poseAdjustmentCount = 0;
    // }
}

void PoseEstimator::Periodic()
{
    frc::Pose2d pose = m_odometry->GetPose();
    

    if (map.size() > 50)
    {
        map.erase(map.begin(), map.begin()+1);
    }

    if (poseAdjustmentCount < MOVEMENTFRAMECOUNT)
    {
        poseAdjustmentCount++;
        map.push_back({wpi::Now(), RobotPose{(double)pose.X(), (double)pose.Y(), (double)pose.Rotation().Radians()} , (calculatedPoseDelta * RobotPose{(double)poseAdjustmentCount, (double)poseAdjustmentCount, (double)poseAdjustmentCount} / RobotPose{MOVEMENTFRAMECOUNT, MOVEMENTFRAMECOUNT, MOVEMENTFRAMECOUNT}) + lastPoseDelta});
        
    }
    else
    {
        map.push_back({wpi::Now(), RobotPose{(double)pose.X(), (double)pose.Y(), (double)pose.Rotation().Radians()}, calculatedPoseDelta});
    }
}

frc::Pose2d PoseEstimator::GetPose(double yaw)
{
    if (map.size() > 1)
    {
        return frc::Pose2d{frc::Translation2d{units::meter_t{offset.x + (double)m_odometry->GetPose().X()}, units::meter_t{offset.x + (double)m_odometry->GetPose().Y()}}, frc::Rotation2d{units::radian_t{yaw}}};
    }
    else
    {
        return m_odometry->GetPose();
    }
}