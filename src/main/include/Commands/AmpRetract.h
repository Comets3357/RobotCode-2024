#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/AmpSubsystem.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/IndexerSubsytem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

COMMAND_START(AmpRetract)

AmpRetractCommand(ShooterSubsystem* shooter, AmpSubsystem* amp, IndexerSubsystem* indexer);


ShooterSubsystem* shooterSubsystem;
AmpSubsystem* ampSubsystem;
IndexerSubsystem* indexerSubsystem;

bool alreadySetIt = false;
double time;

COMMAND_END