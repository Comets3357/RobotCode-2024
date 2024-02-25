#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include <frc/DigitalInput.h>
#include <frc2/command/InstantCommand.h>
#include "Subsystems/LEDsSubsystem.h"
#include "COMETS3357/GyroSubsystem.h"


SUBSYSTEM_START(AutonGyroReset)

AutonGyroResetSubsystem(COMETS3357::GyroSubsystem* gyroSubsystem, LEDsSubsystem* ledsSubsystem);

COMETS3357::GyroSubsystem* gyro;
LEDsSubsystem* leds;
frc::DigitalInput resetButton{0};

SUBSYSTEM_END
