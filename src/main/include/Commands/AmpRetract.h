#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/AmpSubsystem.h"
#include "Subsystems/ShooterSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

class AmpRetractCommand : public frc2::CommandHelper<frc2::SequentialCommandGroup, AmpRetractCommand>
{
    public:
    AmpRetractCommand(AmpSubsystem* amp, ShooterSubsystem* shooter);
};