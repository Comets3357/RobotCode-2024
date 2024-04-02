#pragma once 

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/IndexerSubsytem.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/LookupTable.h"
// #include "Subsystems/IntakeSubsystem.h"
#include <cmath> 
#include <frc/DriverStation.h>

COMMAND_START(SourceShoot)

SourceShootCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem);//, IntakeSubsystem* intake); 

ShooterSubsystem * shooterSubsystem; 
IndexerSubsystem * indexerSubsystem;
// IntakeSubsystem* intakeSubsystem;
COMETS3357::SwerveSubsystem * swerve; 
frc::Translation2d targetPos; 

int xOffset = -1;

COMMAND_END