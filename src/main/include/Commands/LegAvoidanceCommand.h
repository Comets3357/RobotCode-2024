#pragma once;

#include "COMETS3357/Commands/Command.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/GyroSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

COMMAND_START(LegAvoidance)


LegAvoidanceCommand(COMETS3357::SwerveSubsystem *swerve);

COMETS3357::SwerveSubsystem* swerveSubsystem;

COMMAND_END

