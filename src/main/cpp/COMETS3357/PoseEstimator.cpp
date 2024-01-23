#include "COMETS3357/PoseEstimator.h"

#define MOVEMENTFRAMECOUNT 50

using namespace COMETS3357;

PoseEstimator::PoseEstimator(frc::SwerveDriveOdometry<4>* odometry) : m_odometry{odometry}
{

}

void PoseEstimator::AddPose(frc::Pose2d pose, double timestamp)
{
    int top = 1;
    for (int i = 1; i < 45; i++)
    {
        if (i < map.size()-1)
        {
            if (timestamp < get<0>(map[map.size()-i]))
            {
                top = map.size()-i;
                break;
            }
        }
        else
        {
            return;
        }
    }
    if (top > 1)
    {
        double deltaTime = get<0>(map[top]) - get<0>(map[top-1]);
        
        lastPoseDelta = (calculatedPoseDelta * RobotPose{(double)poseAdjustmentCount, (double)poseAdjustmentCount, (double)poseAdjustmentCount} / RobotPose{MOVEMENTFRAMECOUNT, MOVEMENTFRAMECOUNT, MOVEMENTFRAMECOUNT}) + lastPoseDelta;
        calculatedPoseDelta = ((RobotPose{(double)pose.X(), (double)pose.Y(), (double)pose.Rotation().Radians()} - lastPoseDelta) - (((get<1>(map[top]) - get<1>(map[top-1]))/RobotPose{deltaTime, deltaTime, deltaTime})*RobotPose{timestamp, timestamp, timestamp})+get<1>(map[top-1]));
        map.erase(map.begin(), map.begin() + (top-2));
        poseAdjustmentCount = 0;
    }
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
        map.push_back({nt::Now(), RobotPose{(double)pose.X(), (double)pose.Y(), (double)pose.Rotation().Radians()} , (calculatedPoseDelta * RobotPose{(double)poseAdjustmentCount, (double)poseAdjustmentCount, (double)poseAdjustmentCount} / RobotPose{MOVEMENTFRAMECOUNT, MOVEMENTFRAMECOUNT, MOVEMENTFRAMECOUNT}) + lastPoseDelta});
        
    }
    else
    {
        map.push_back({nt::Now(), RobotPose{(double)pose.X(), (double)pose.Y(), (double)pose.Rotation().Radians()}, calculatedPoseDelta});
    }
}

frc::Pose2d PoseEstimator::GetPose(double yaw)
{
    if (map.size() > 1)
    {
        return frc::Pose2d{frc::Translation2d{units::meter_t{get<1>(map[map.size()-1]).x + get<2>(map[map.size()-1]).x}, units::meter_t{get<1>(map[map.size()-1]).y + get<2>(map[map.size()-1]).y}}, frc::Rotation2d{units::radian_t{yaw}}};
    }
    else
    {
        return m_odometry->GetPose();
    }
}