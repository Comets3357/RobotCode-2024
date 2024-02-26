#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/ShooterSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>


COMMAND_START(Climb)

ClimbCommand(ElevatorSubsystem* elevator, ShooterSubsystem* shooter);


ElevatorSubsystem* elevatorSubsystem;
ShooterSubsystem* shooterSubsystem;

COMMAND_END

