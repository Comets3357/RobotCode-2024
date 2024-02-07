#pragma once

#include "COMETS3357/Commands/Command.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/GyroSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>


COMMAND_START(LegAvoidance)


LegAvoidanceCommand(COMETS3357::SwerveSubsystem *swerve);

    frc::Field2d bottom1Field;
    frc::Field2d bottom2Field;
    frc::Field2d point3Field;

COMETS3357::SwerveSubsystem* swerveSubsystem;

COMMAND_END

