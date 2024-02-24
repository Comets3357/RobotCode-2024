
#include "Commands/IntakeIndexerAutonCommand.h"

IntakeIndexerAutonCommand::IntakeIndexerAutonCommand(IndexerSubsystem *indexer) {
    indexerSubsytem = indexer; 
    AddRequirements({indexer}); 
}

void IntakeIndexerAutonCommand::Initialize()
{
    if (indexerSubsytem->IsDetected())
    {
        indexerSubsytem->SetPercent(0.4); 
    }
}

void IntakeIndexerAutonCommand::Execute()
{
    if (!indexerSubsytem->IsDetected())
    {
        indexerSubsytem->SetPercent(0); 
    }
}

bool IntakeIndexerAutonCommand::IsFinished()
{
    return false;
}

void IntakeIndexerAutonCommand::End(bool interrupted)
{
    indexerSubsytem->SetVelocity(0); 
}