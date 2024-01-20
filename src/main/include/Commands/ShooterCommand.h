#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/IndexerSubsytem.h"

COMMAND_START(Shooter)

ShooterCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer); 



ShooterSubsystem * shooterSubsystem; 
IndexerSubsystem * indexerSubsystem;

COMMAND_END