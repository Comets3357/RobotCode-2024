#include "Commands/SourceShootCommand.h"

SourceShootCommand::SourceShootCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem, IntakeSubsystem* intake) {
    shooterSubsystem = shooter;
    indexerSubsystem = indexer; 
    intakeSubsystem = intake;
    swerve = swerveSubsystem; 
    AddRequirements({shooter}); 
    SetName("Shoot Command");
}

void SourceShootCommand::Initialize()
{
    frc::Pose2d robotPose = swerve->GetPose();
    xOffset++;
    if (xOffset > 2) xOffset = 0;
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
         if ((double)robotPose.X() > 5)
        {
            targetPos = frc::Translation2d{units::meter_t{14.66}, units::meter_t{6.86}};
        }
        else
        {
            targetPos = frc::Translation2d{units::meter_t{9}, units::meter_t{7}};
        }
        targetPos = frc::Translation2d{units::meter_t{(double)targetPos.X() + (0.4 * xOffset)}, units::meter_t{(double)targetPos.Y()}};
    }
    else
    {
        if ((double)robotPose.X() < 12)
        {
            targetPos = frc::Translation2d{units::meter_t{1.91}, units::meter_t{6.86}};
        }
        else
        {
            targetPos = frc::Translation2d{units::meter_t{7.61}, units::meter_t{7}};
        }
        targetPos = frc::Translation2d{units::meter_t{(double)targetPos.X() - (0.4 * xOffset)}, units::meter_t{(double)targetPos.Y()}};
    }
}

void SourceShootCommand::Execute()
{
    intakeSubsystem->SetPercent("IntakeSpeed");
    if (indexerSubsystem->IsDetected())
    {
        indexerSubsystem->SetPercent(0.4); 
        indexerAlreadySet = false;
    }
    else if (!indexerAlreadySet)
    {
        indexerSubsystem->SetPercent(0); 
        indexerAlreadySet = true;
    }
    frc::Pose2d pos = swerve->GetPose();
    //frc::Translation2d targetPos = frc::Translation2d{units::meter_t{8.5}, units::meter_t{8.25}};

    double distance = sqrt(pow((double)(targetPos.X() - pos.X()), 2) + pow((double)(targetPos.Y() - pos.Y()), 2)); 
    double velocity = shooterSubsystem->SourceSpeed.GetValue(distance); 
    shooterSubsystem->Pivot.SetPosition(58.25, shooterSubsystem->offset);
    shooterSubsystem->SetVelocityFlyWheel(-velocity);
    shooterSubsystem->SetVelocityKickerWheel(velocity); 
    
}

bool SourceShootCommand::IsFinished()
{
    return false;
}

void SourceShootCommand::End(bool interrupted)
{
    
}