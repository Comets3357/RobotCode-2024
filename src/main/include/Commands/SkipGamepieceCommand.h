#pragma once

#include "COMETS3357/Commands/Command.h"
#include "COMETS3357/Subsystems/Vision/NoteDetection.h"

COMMAND_START(SkipGamepiece)


SkipGamepieceCommand(NoteDetectionSubsystem *noteDetection);

NoteDetectionSubsystem* noteDetectionSubsystem;

COMMAND_END

