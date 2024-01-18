
#include "Commands/IntakeIndexerCommand.h"

IntakeIndexerCommand::IntakeIndexerCommand(IndexerSubsystem *indexer) {
    indexerSubsytem = indexer; 
    AddRequirements({indexer}); 
}

void IntakeIndexerCommand::Initialize()
{
    indexerSubsytem->SetVelocity("IndexerSpeed"); 

}

void IntakeIndexerCommand::Execute()
{

}

bool IntakeIndexerCommand::IsFinished()
{
    return indexerSubsytem->IsDetected(); 
}

void IntakeIndexerCommand::End(bool interrupted)
{
    indexerSubsytem->SetVelocity(0); 
}