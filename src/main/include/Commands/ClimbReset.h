#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/ElevatorSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>


COMMAND_START(ClimbReset)

ClimbResetCommand(ElevatorSubsystem* elevator);


ElevatorSubsystem* elevatorSubsystem;

COMMAND_END

