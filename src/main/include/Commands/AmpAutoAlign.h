#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/IndexerSubsytem.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/LookupTable.h"
#include <cmath> 
#include <frc/DriverStation.h>
#include "COMETS3357/Auton/AutonPathCommand.h"


COMMAND_START(AmpAutoAlign) 

    AmpAutoAlignCommand(COMETS3357::SwerveSubsystem* swerveSubsystem);
    COMETS3357::SwerveSubsystem *swerveSubsystem; 
    
    frc::Translation2d targetPos; 


COMMAND_END 