#pragma once

#include "COMETS3357/Commands/Command.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/LookupTable.h"
#include "COMETS3357/GyroSubsystem.h"
#include "COMETS3357/LookupTable.h"
#include <cmath> 
#include <frc/DriverStation.h>
COMMAND_START(TurnTo)

TurnToCommand(COMETS3357::SwerveSubsystem* swerveSubsystem, COMETS3357::GyroSubsystem* gyro); 

COMETS3357::SwerveSubsystem * swerve; 
COMETS3357::GyroSubsystem* gyro;
COMETS3357::LookupTable rotationPLookup{"RotationP"};
frc::Translation2d targetPos; 

COMMAND_END