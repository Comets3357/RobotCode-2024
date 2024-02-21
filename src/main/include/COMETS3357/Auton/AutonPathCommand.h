#pragma once

#include "COMETS3357/Commands/Command.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"



COMMAND_START(AutonPath)

AutonPathCommand(COMETS3357::SwerveSubsystem *swerveSubsystem, double maxTurn, double maxSpeed, frc::Pose2d pose); 

COMETS3357::SwerveSubsystem *swerve; 

frc::PIDController translatePID{1, 0, 0};
frc::PIDController rotPID{1, 0, 0};

frc::Pose2d targetPose;

double rot;
double speed;


COMMAND_END