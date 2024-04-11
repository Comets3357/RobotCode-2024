
#include "Commands/SkipGamepieceCommand.h"

SkipGamepieceCommand::SkipGamepieceCommand(NoteDetectionSubsystem* noteDetection) {
    noteDetectionSubsystem = noteDetection;
}

void SkipGamepieceCommand::Initialize()
{

}

void SkipGamepieceCommand::Execute()
{

}
bool SkipGamepieceCommand::IsFinished()
{
    return !noteDetectionSubsystem->autonNoteValid;
}   

void SkipGamepieceCommand::End(bool interrupted)
{
   
}