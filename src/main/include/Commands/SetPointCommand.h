#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/IndexerSubsytem.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/LookupTable.h"
#include <cmath> 
#include <frc/DriverStation.h>
COMMAND_START(SetPoint)

SetPointCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem, double angle); 
SetPointCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem, double angle, double speed); 

ShooterSubsystem * shooterSubsystem; 
IndexerSubsystem * indexerSubsystem;
COMETS3357::SwerveSubsystem * swerve; 
frc::Translation2d targetPos; 

double setPointAngle;
double speed;

bool useSpeed = false;


COMMAND_END