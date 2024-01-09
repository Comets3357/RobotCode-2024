#pragma once

#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/Subsystems/Subsystem.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/length.h>
#include <frc/geometry/Pose2d.h>

SUBSYSTEM_START(VisionSystem)

    VisionSystemSubsystem(COMETS3357::SwerveSubsystem* swerve);

    std::shared_ptr<nt::NetworkTable>* driveTable;

    

    COMETS3357::SwerveSubsystem* swerveSubsystem;
    double lastTimestamp = 0;

SUBSYSTEM_END