#pragma once

#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/Subsystems/Subsystem.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <units/length.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "COMETS3357/PoseEstimator.h"
#include <set>
#include <map>
#include <networktables/DoubleArrayTopic.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/opencv.hpp>


SUBSYSTEM_START(VisionSystem)

    VisionSystemSubsystem(COMETS3357::SwerveSubsystem* swerve, COMETS3357::GyroSubsystem* gyro);

    std::shared_ptr<nt::NetworkTable>* driveTable;

    COMETS3357::GyroSubsystem *gyroSubsystem;


    COMETS3357::SwerveSubsystem* swerveSubsystem;
    double lastTimestamp = 0;

    nt::DoubleArraySubscriber tagSub;

    nt::DoublePublisher timePublisher;
    nt::DoublePublisher epochPublisher;
    nt::DoubleSubscriber rotationSpeedSub;

    frc::Field2d m_field;
    frc::Field2d m_field2;
    double currentTimestamp;

    frc::Pose2d lastPose;
    bool ResetPose = true;

    std::set<std::pair<double, double>> gyroValues = {};
    std::map<double, std::pair<double, double>> tagPositions =
    {
        {1, {15.0794719999,0.2458719999}},
        {2, {16.185134, 0.883666}},
        {3, {16.503142, 4.982717999999999}},
        {4, {16.503142, 5.547867999999999}},
        {5, {14.700757999999999, 8.2042}},
        {6, {1.8415, 8.2042}},
        {7, {-0.038099999999999995, 5.547867999999999}},
        {8, {-0.038099999999999995, 4.982717999999999}},
        {9, {0.356108, 0.883666}},
        {10, {1.4615159999999998, 0.24587199999999998}},
        {11, {11.904726, 3.7132259999999997}},
        {12, {11.904726, 4.49834}},
        {13, {11.220196, 4.105148}},
        {14, {5.320792, 4.105148}},
        {15, {4.641342, 4.49834}},
        {16, {4.641342, 3.7132259999999997}}
    };

    // COMETS3357::PoseEstimator poseEstimator;

SUBSYSTEM_END