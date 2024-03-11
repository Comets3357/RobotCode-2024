#pragma once

#include "COMETS3357/Commands/Command.h"
#include "COMETS3357/Subsystems/Vision/NoteDetection.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/GyroSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include "Subsystems/IndexerSubsytem.h"


COMMAND_START(MiddleNoteDetection)


MiddleNoteDetectionCommand(NoteDetectionSubsystem *noteDetection, COMETS3357::SwerveSubsystem* swerve, IndexerSubsystem* indexer);

NoteDetectionSubsystem* noteDetectionSubsystem;
IndexerSubsystem* indexerSubsystem;
COMETS3357::SwerveSubsystem* swerveSubsystem;

COMMAND_END

