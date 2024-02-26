#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/IndexerSubsytem.h"



COMMAND_START(IntakeIndexerAuton)

IntakeIndexerAutonCommand(IndexerSubsystem *indexer); 

IndexerSubsystem *indexerSubsytem; 


COMMAND_END