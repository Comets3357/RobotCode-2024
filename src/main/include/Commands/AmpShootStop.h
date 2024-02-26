#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/IndexerSubsytem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>


COMMAND_START(AmpShootStop)


AmpShootStopCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator, IndexerSubsystem* indexer);


ElevatorSubsystem* elevatorSubsystem;
IndexerSubsystem* indexerSubsystem;
ShooterSubsystem* shooterSubsystem;

COMMAND_END

