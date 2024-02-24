#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include <frc/DigitalInput.h>
#include <frc2/command/InstantCommand.h>


SUBSYSTEM_START(AutonGyroReset)

AutonGyroResetSubsystem(frc2::InstantCommand* gyroReset);

frc2::InstantCommand* resetCommand;
frc::DigitalInput resetButton{0};

SUBSYSTEM_END
