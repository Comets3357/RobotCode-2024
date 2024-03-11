
#include "Commands/MiddleNoteDetectionCommand.h"

MiddleNoteDetectionCommand::MiddleNoteDetectionCommand(NoteDetectionSubsystem* noteDetection, COMETS3357::SwerveSubsystem* swerve, IndexerSubsystem* indexer, COMETS3357::LimelightSubsystem* limelight, COMETS3357::GyroSubsystem* gyro) {
    swerveSubsystem = swerve;
    noteDetectionSubsystem = noteDetection;
    indexerSubsystem = indexer;
    limelightSubsystem = limelight;
    gyroSubsystem = gyro;

    translatePID.SetTolerance(0.1);
    rotPID.SetTolerance(0.1);
    translatePID.SetP(0.85);
    rotPID.SetP(0.6);
    rotPID.SetSetpoint(-gyro->m_navx.GetYaw() * 3.14159 / 180.0);
    rotPID.EnableContinuousInput(-3.14159265, 3.14159265);

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
    if (!gotPosition)
    {
        if (limelightSubsystem->hasTarget())
        {
            units::second_t time = wpi::math::MathSharedStore::GetTimestamp() - units::second_t{(limelightSubsystem->getPipelineLatency() / 1000.0)};
            double D = h*tan(theta+(limelightSubsystem->getY() * 3.14159 / 180));
            double Distance = D+(gr/2);
            y = (double)swerveSubsystem->GetPose().Y() + (Distance*std::sin((double)swerveSubsystem->GetSampledVisionPose(time).Rotation().Radians() - (limelightSubsystem->getX() * 3.141592654/180)) + 0.3*std::sin((double)swerveSubsystem->GetSampledVisionPose(time).Rotation().Radians()));
            x = (double)swerveSubsystem->GetPose().X() + (Distance*std::cos((double)swerveSubsystem->GetSampledVisionPose(time).Rotation().Radians() - (limelightSubsystem->getX() * 3.141592654/180)) + 0.3*std::cos((double)swerveSubsystem->GetSampledVisionPose(time).Rotation().Radians()));
            gotPosition = true;
        }
    }
    else
    {
        frc::Pose2d currentPose = swerveSubsystem->GetPose();
        double rotationSpeed = std::clamp(rotPID.Calculate((double)currentPose.Rotation().Radians()), -0.5, 0.5);
        double angle = atan2((double)currentPose.X() - x, (double)currentPose.Y() -x);
        double robotSpeed = std::clamp(translatePID.Calculate(sqrt(pow((double)swerveSubsystem->GetPose().Y() - y, 2) + pow((double)swerveSubsystem->GetPose().X() - x, 2)), 0), -1.0, 1.0);
        double movementX = sin(angle) * robotSpeed;
        double movementY = cos(angle) * robotSpeed;//std::clamp(translatePID.Calculate((double)currentPose.Y(), (double)targetPose.Y()), -speed, speed);
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
        {
            movementX *= -1;
            movementY *= -1;
        }
        swerveSubsystem->DriveXRotate(units::meters_per_second_t{movementX}, units::meters_per_second_t{movementY}, units::radians_per_second_t{rotationSpeed});
    }
}

// bool MiddleNoteDetectionCommand::IsFinished()
// {
//     if ((double)swerveSubsystem->GetPose().X() < 8.7 || !indexerSubsystem->IsDetected() || !noteDetectionSubsystem->autonNoteValid)
//     {

//         noteDetectionSubsystem->stopGoToNote();
//         swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);
//         frc::SmartDashboard::PutNumber("LJKASHDKJAHSDOASOIDUANSD", 1);
//         return true;
//     }
//     return false;
// }

bool MiddleNoteDetectionCommand::IsFinished()
{
    frc::Pose2d currentPose = swerve->GetPose();
    if ((sqrt(pow((double)swerveSubsystem->GetPose().Y() - y, 2) + pow((double)swerveSubsystem->GetPose().X() - x, 2)) < 0.1) && gotPosition)
    {
            swerve->DriveXRotate(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0});

return true;
    }
    return false;
}

void MiddleNoteDetectionCommand::End(bool interrupted)
{
   
}