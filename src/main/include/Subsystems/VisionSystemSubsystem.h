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


SUBSYSTEM_START(VisionSystem)

    VisionSystemSubsystem(COMETS3357::SwerveSubsystem* swerve);

    std::shared_ptr<nt::NetworkTable>* driveTable;


    COMETS3357::SwerveSubsystem* swerveSubsystem;
    double lastTimestamp = 0;

    nt::DoubleSubscriber xSub;
    int i = 0;
    nt::DoubleSubscriber ySub;
    nt::DoubleSubscriber timestampSub;
    nt::DoublePublisher timePublisher;
    nt::DoubleSubscriber rotationSpeedSub;
    nt::DoubleSubscriber velocitySub;
    nt::DoubleSubscriber distanceSub;
    frc::Field2d m_field;
    frc::Field2d m_field2;

    frc::Pose2d lastPose;
    bool ResetPose = true;

    // COMETS3357::PoseEstimator poseEstimator;

SUBSYSTEM_END