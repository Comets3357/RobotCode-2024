
#include "Commands/ShooterCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

ShooterCommand::ShooterCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem) {
    shooterSubsystem = shooter;
    indexerSubsystem = indexer; 
    swerve = swerveSubsystem; 
    AddRequirements({shooter}); 
}

void ShooterCommand::Initialize()
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

void ShooterCommand::Execute()
{
    frc::Pose2d pos = swerve->GetPose();
    double distance2 = sqrt(pow((double)(targetPos.X() - pos.X()), 2) + pow((double)(targetPos.Y() - pos.Y()), 2)); 
    double shooterAngle2 = shooterSubsystem->angleLookup.GetValue(distance2);
    double velocity2 = cos(shooterAngle2 * 3.14159 / 180) * 17; 

    pos = swerve->GetMovingPose(distance2 / velocity2); 
    double distance = sqrt(pow((double)(targetPos.X() - pos.X()), 2) + pow((double)(targetPos.Y() - pos.Y()), 2)); 
    //frc::SmartDashboard::PutNumber("Distance From Target", distance);
   
    double shooterAngle = shooterSubsystem->angleLookup.GetValue(distance);
    //frc::SmartDashboard::PutNumber("angle", shooterAngle);
    double velocity = shooterSubsystem->velocityLookup.GetValue(distance);
    shooterSubsystem->Pivot.SetPosition(shooterAngle, shooterSubsystem->offset);//shooterAngle); 
    shooterSubsystem->SetVelocityKickerWheel(velocity);
    shooterSubsystem->SetVelocityFlyWheel(-velocity);

    //frc::SmartDashboard::PutNumber("FLywheel Velocity", shooterSubsystem->GetVelocityFlyWheel());
    // if (shooterSubsystem->GetVelocityFlyWheel() < shooterSubsystem->FlyWheel.config.velocities["ShooterSpeed"] + 100)
    // {

    //         indexerSubsystem->SetPercent(0.35);
    //         // return true;
    // }
}

bool ShooterCommand::IsFinished()
{
    // if (shooterSubsystem->GetVelocityFlyWheel() < shooterSubsystem->FlyWheel.config.velocities["ShooterSpeed"] + 100)
    // {
    //         indexerSubsystem->SetPercent(0.35);
    //         return true;
    // }

    return false;
}

void ShooterCommand::End(bool interrupted)
{
    
}