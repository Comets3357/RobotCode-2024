
#include "Commands/MiddleNoteDetectionCommand.h"

MiddleNoteDetectionCommand::MiddleNoteDetectionCommand(NoteDetectionSubsystem* noteDetection, COMETS3357::SwerveSubsystem* swerve, IndexerSubsystem* indexer) {
    swerveSubsystem = swerve;
    noteDetectionSubsystem = noteDetection;
    indexerSubsystem = indexer;

    SetName("MiddleNoteDetection");
}

void MiddleNoteDetectionCommand::Initialize()
{
    noteDetectionSubsystem->goToNote();
    noteDetectionSubsystem->autonNoteValid = true;
    indexerSubsystem->SetPercent(0.4); 
}

void MiddleNoteDetectionCommand::Execute()
{
     indexerSubsystem->SetPercent(0.4); 
}

bool MiddleNoteDetectionCommand::IsFinished()
{
    if ((double)swerveSubsystem->GetPose().X() < 8.7 || !indexerSubsystem->IsDetected() || !noteDetectionSubsystem->autonNoteValid)
    {

        noteDetectionSubsystem->stopGoToNote();
        swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);
        frc::SmartDashboard::PutNumber("LJKASHDKJAHSDOASOIDUANSD", 1);
        return true;
    }
    return false;
}

void MiddleNoteDetectionCommand::End(bool interrupted)
{
   
}