#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/AmpSubsystem.h"
#include "Subsystems/ShooterSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

COMMAND_START(AmpRetract)

AmpRetractCommand(ShooterSubsystem* shooter, AmpSubsystem* amp);


ShooterSubsystem* shooterSubsystem;
AmpSubsystem* ampSubsystem;

bool alreadySetIt = false;
double time;

COMMAND_END