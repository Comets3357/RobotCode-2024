#include "Commands/AmpAutoAlign.h"

AmpAutoAlignCommand::AmpAutoAlignCommand(COMETS3357::SwerveSubsystem* swerve)
{
    swerveSubsystem = swerve; 
    AddRequirements({swerve}); 
}

void AmpAutoAlignCommand::Initialize()
{

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        targetPos = frc::Translation2d{units::meter_t{14.68}, units::meter_t{7.79}};
    }
    else
    {
        targetPos = frc::Translation2d{units::meter_t{1.85}, units::meter_t{7.73}};
    }
}

void AmpAutoAlignCommand::Execute()
{
    
}

