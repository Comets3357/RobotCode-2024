



#include "COMETS3357/Auton/AutonPathCommand.h"

AutonPathCommand::AutonPathCommand(COMETS3357::SwerveSubsystem* swerveSubsystem, double rotSpeed, double moveSpeed, frc::Pose2d pose, bool isVision, int turnSpeaker, double speed) 
{
    swerve = swerveSubsystem;
    translatePID.SetTolerance(0.1);
    rotPID.SetTolerance(0.1);
    translatePID.SetP(0.85);
    rotPID.SetP(0.6);
    rot = rotSpeed;
    speed = moveSpeed;
    targetPose = pose;
    rotPID.SetSetpoint((double)targetPose.Rotation().Radians());
    rotPID.EnableContinuousInput(-3.14159265, 3.14159265);
    // AddRequirements({swerve}); 
    visionUsed = isVision;
    isTurnSpeaker = turnSpeaker;
    SetName("AutonPathCommand");
    endSpeed = speed;
}

void AutonPathCommand::Initialize()
{

}

void AutonPathCommand::Execute()
{
    swerve -> overrideXSpeed = 0_mps;
    swerve -> overrideYSpeed = 0_mps;
    if (!isTurnSpeaker)
    {
        swerve -> controllingSwerveRotation = true;
        swerve -> overrideRotation = units::radians_per_second_t{0};
    }
    
    swerve -> controllingSwerveMovement = true;
    


    frc::Pose2d currentPose;
    if (visionUsed)
    {
        currentPose = swerve->GetPose();
    }
    else
    {
        currentPose = swerve->GetPose();
    }
    double rotationSpeed = std::clamp(rotPID.Calculate((double)currentPose.Rotation().Radians()), -rot, rot);
    double angle = atan2((double)currentPose.X() - (double)targetPose.X(), (double)currentPose.Y() - (double)targetPose.Y());
    double robotSpeed = std::clamp(translatePID.Calculate((double)currentPose.Translation().Distance(targetPose.Translation()), 0), -speed, speed);
    double movementX = sin(angle) * robotSpeed;
    double movementY = cos(angle) * robotSpeed;//std::clamp(translatePID.Calculate((double)currentPose.Y(), (double)targetPose.Y()), -speed, speed);
       if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        movementX *= -1;
        movementY *= -1;
    }
    swerve->DriveXRotate(units::meters_per_second_t{movementX}, units::meters_per_second_t{movementY}, units::radians_per_second_t{rotationSpeed});
}

bool AutonPathCommand::IsFinished()
{
    frc::Pose2d currentPose = swerve->GetPose();
    if ((currentPose.Translation().Distance(targetPose.Translation()) < units::meter_t{0.1} && rotPID.AtSetpoint()) || (isTurnSpeaker && currentPose.Translation().Distance(targetPose.Translation()) < units::meter_t{0.1}))
    {
            swerve->DriveXRotate(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0});

return true;
    }
    return false;
}

void AutonPathCommand::End(bool interrupted)
{
    swerve->DriveXRotate(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0});
}