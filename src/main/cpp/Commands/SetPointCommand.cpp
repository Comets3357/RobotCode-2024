
#include "Commands/SetPointCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

SetPointCommand::SetPointCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem, double angle) {
    shooterSubsystem = shooter;
    indexerSubsystem = indexer; 
    swerve = swerveSubsystem; 
    setPointAngle = angle;
    AddRequirements({shooter}); 
}

void SetPointCommand::Initialize()
{
    shooterSubsystem->SetVelocityFlyWheel("ShooterSpeed"); 
    shooterSubsystem->SetVelocityKickerWheel("ShooterSpeed"); 
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        targetPos = frc::Translation2d{units::meter_t{16.579}, units::meter_t{5.547867999999999}};
    }
    else
    {
        targetPos = frc::Translation2d{units::meter_t{0}, units::meter_t{5.547867999999999}};
    }
}

void SetPointCommand::Execute()
{

    shooterSubsystem->Pivot.SetPosition(setPointAngle); 
    shooterSubsystem->SetVelocityKickerWheel(2000);
    shooterSubsystem->SetVelocityFlyWheel(-2000);

    frc::SmartDashboard::PutNumber("FLywheel Velocity", shooterSubsystem->GetVelocityFlyWheel());
    // if (shooterSubsystem->GetVelocityFlyWheel() < shooterSubsystem->FlyWheel.config.velocities["ShooterSpeed"] + 100)
    // {

    //         indexerSubsystem->SetPercent(0.35);
    //         // return true;
    // }
}

bool SetPointCommand::IsFinished()
{
    // if (shooterSubsystem->GetVelocityFlyWheel() < shooterSubsystem->FlyWheel.config.velocities["ShooterSpeed"] + 100)
    // {
    //         indexerSubsystem->SetPercent(0.35);
    //         return true;
    // }

    return false;
}

void SetPointCommand::End(bool interrupted)
{
    
}