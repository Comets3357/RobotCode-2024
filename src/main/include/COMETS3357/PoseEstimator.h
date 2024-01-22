#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/filter/LinearFilter.h>

namespace COMETS3357
{

    struct RobotPose
    {
        double x, y, rotation;

        RobotPose operator- (const RobotPose& other) const {
            return RobotPose{x - other.x, y - other.y, rotation - other.rotation};
        }
        RobotPose operator/ (const RobotPose& other) const {
            return RobotPose{x / other.x, y / other.y, rotation / other.rotation};
        }
        RobotPose operator+ (const RobotPose& other) const {
            return RobotPose{x + other.x, y + other.y, rotation + other.rotation};
        }
        RobotPose operator* (const RobotPose& other) const {
            return RobotPose{x * other.x, y * other.y, rotation * other.rotation};
        }
    };

    class PoseEstimator
    {
    public:
        PoseEstimator(frc::SwerveDriveOdometry<4>* odometry);

        void AddPose(frc::Pose2d pose, double timestamp);

        void Periodic();

        frc::SwerveDriveOdometry<4>* m_odometry;

        RobotPose poseZero{0.0, 0.0, 0.0};
        std::vector<std::tuple<double, RobotPose, RobotPose>> map;
        int poseAdjustmentCount = 0;
        RobotPose calculatedPoseDelta{0.0, 0.0, 0.0};

        frc::Pose2d GetPose(double yaw);

        std::shared_ptr<nt::NetworkTable> gyroSubsystemData;
        RobotPose lastPoseDelta{0.0, 0.0, 0.0};

    };
};