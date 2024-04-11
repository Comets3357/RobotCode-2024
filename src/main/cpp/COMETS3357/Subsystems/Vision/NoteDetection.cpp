#include "COMETS3357/Subsystems/Vision/LimelightSubsystem.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/Subsystems/Vision/NoteDetection.h"
#include "COMETS3357/GyroSubsystem.h"

NoteDetectionSubsystem::NoteDetectionSubsystem(COMETS3357::SwerveSubsystem *swervePointer, COMETS3357::LimelightSubsystem *limelightPointer, COMETS3357::GyroSubsystem *gyroPointer) : COMETS3357::Subsystem("NoteDetection"){
    swerveField = swervePointer;
    limelightField = limelightPointer;
    gyroField = gyroPointer;

    translatePID.SetTolerance(0.1);
    rotPID.SetTolerance(0.1);
    translatePID.SetP(0.75);
    rotPID.SetP(0.6);
    
    rotPID.EnableContinuousInput(-3.14159265, 3.14159265);
}

void NoteDetectionSubsystem::Initialize()
{

}

void NoteDetectionSubsystem::Periodic()
{
    units::second_t time = wpi::math::MathSharedStore::GetTimestamp() - units::second_t{(limelightField->getPipelineLatency() / 1000.0)};
    if (limelightField->hasTarget())
    {
         double D = h*tan(theta+(limelightField->getY() * 3.14159 / 180));
            double Distance = D+(gr/2);
            y = Distance*std::sin((double)swerveField->GetSampledVisionPose(time).Rotation().Radians() - (limelightField->getX() * 3.141592654/180)) + 0.3*std::sin((double)swerveField->GetSampledVisionPose(time).Rotation().Radians() + ((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) ? 3.14159 : 0));
            x = Distance*std::cos((double)swerveField->GetSampledVisionPose(time).Rotation().Radians() - (limelightField->getX() * 3.141592654/180)) + 0.3*std::cos((double)swerveField->GetSampledVisionPose(time).Rotation().Radians() + ((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) ? 3.14159 : 0));
            m_field.SetRobotPose(frc::Pose2d{frc::Translation2d{units::meter_t{x + (double)swerveField->GetSampledVisionPose(time).X()}, units::meter_t{y + (double)swerveField->GetSampledVisionPose(time).Y()}}, frc::Rotation2d{units::radian_t{0}}});

            frc::SmartDashboard::PutData("GamepiecePosition", &m_field);
    }
    if (goingToNote) {
       
        
        // frc::SmartDashboard::PutNumber("Xasdasdasd", x + (double)swerveField->GetPose().X());
        // if (limelightField->hasTarget() && (x + (double)swerveField->GetPose().X()) > 8) {
        //     swerveField -> controllingSwerveRotation = false;
        // swerveField -> overrideRotation = -units::radians_per_second_t{std::clamp(limelightField->getX() * 3.14159 * 0.6 / 180, -0.5, 0.5)};
rotPID.SetSetpoint(atan2(x, y) + (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0);
            swerveField -> controllingSwerveMovement = false;

            double rotationSpeed = std::clamp(rotPID.Calculate((double)swerveField->GetPose().Rotation().Radians()), -0.5, 0.5);
    double angle = atan2(y, x);
    double robotSpeed = std::clamp(translatePID.Calculate(sqrt(pow(x, 2) + pow(y, 2))), -1.0, 1.0);
    double movementX = sin(angle) * robotSpeed;
    double movementY = cos(angle) * robotSpeed;//std::clamp(translatePID.Calculate((double)currentPose.Y(), (double)targetPose.Y()), -speed, speed);


            swerveField->overrideRotation = units::radians_per_second_t{rotationSpeed};
            swerveField -> overrideYSpeed = units::meters_per_second_t{movementX};// units::meters_per_second_t{ std::clamp(std::sin(((-gyroField->m_navx.GetYaw() * 3.14159 / 180.0) +((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroField->angleOffset) - (limelightField->getX() * 3.141592654/180)), -0.5, 0.5)};
            swerveField -> overrideXSpeed = units::meters_per_second_t{movementY};//-units::meters_per_second_t{ std::clamp(std::cos(((-gyroField->m_navx.GetYaw() * 3.14159 / 180.0) +((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroField->angleOffset) - (limelightField->getX() * 3.141592654/180)), -0.5, 0.5)};
        

              if (frc::DriverStation::IsAutonomous())
        {
            swerveField->DriveXRotate(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::angular_velocity::radians_per_second_t{0});
        }
    }
}

void NoteDetectionSubsystem::goToNote()
{
    goingToNote = true;   
}

void NoteDetectionSubsystem::stopGoToNote()
{   
    goingToNote = false;
    swerveField -> overrideXSpeed = 0_mps;
    swerveField -> overrideYSpeed = 0_mps;
    swerveField -> overrideRotation = units::radians_per_second_t{0};
    swerveField -> controllingSwerveMovement = true;
    swerveField -> controllingSwerveRotation = true;
    swerveField->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveField->kDriveKinematics);
}

