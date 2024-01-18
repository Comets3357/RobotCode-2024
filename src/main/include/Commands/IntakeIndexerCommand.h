#pragma once

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/IndexerSubsytem.h"



COMMAND_START(IntakeIndexer)

IntakeIndexerCommand(IndexerSubsystem *indexer); 

IndexerSubsystem *indexerSubsytem; 


COMMAND_END