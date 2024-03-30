#include "Commands/SourceShootCommand.h"

SourceShootCommand::SourceShootCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem) {
    shooterSubsystem = shooter;
    indexerSubsystem = indexer; 
    swerve = swerveSubsystem; 
    AddRequirements({shooter}); 
    SetName("Shoot Command");
}

void SourceShootCommand::Initialize()
{
    frc::Translation2d targetPos; 
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
       targetPos = frc::Translation2d{units::meter_t{8.5}, units::meter_t{8.25}};
    } else {
        targetPos = frc::Translation2d{units::meter_t{0}, units::meter_t{0}};   // change value
    }
}

void SourceShootCommand::Execute()
{

    frc::Pose2d pos = swerve->GetPose();
    //frc::Translation2d targetPos = frc::Translation2d{units::meter_t{8.5}, units::meter_t{8.25}};

    double distance = sqrt(pow((double)(targetPos.X() - pos.X()), 2) + pow((double)(targetPos.Y() - pos.Y()), 2)); 
    double velocity = shooterSubsystem->SourceSpeed.GetValue(distance); 
    shooterSubsystem->Pivot.SetPosition(shooterSubsystem->SourceAngle.GetValue(distance), shooterSubsystem->offset);
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