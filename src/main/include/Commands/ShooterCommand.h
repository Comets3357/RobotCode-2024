#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/ShooterSubsystem.h"

COMMAND_START(Shooter)

ShooterCommand(ShooterSubsystem *shooter); 

ShooterSubsystem *ShooterSubsytem; 

COMMAND_END