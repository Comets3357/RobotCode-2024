#pragma once

#include "COMETS3357/Commands/Command.h"
#include "ShooterCommand.h"
#include "Subsystems/IndexerSubsystem.h"

COMMAND_START(PinPoint)

PinPointCommand(IndexerSubsystem *FlyWheel, ShooterSubsystem *ShooterAngle); 

IndexerSubsystem *FlyWheel;
ShooterSubsystem *ShooterAngle;

COMMAND_END