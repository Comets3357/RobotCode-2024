#include "COMETS3357/Subsystems/Vision/LimelightSubsystem.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/Subsystems/Vision/NoteDetection.h"
#include "COMETS3357/GyroSubsystem.h"

NoteDetectionSubsystem::NoteDetectionSubsystem(COMETS3357::SwerveSubsystem *swervePointer, COMETS3357::LimelightSubsystem *limelightPointer, COMETS3357::GyroSubsystem *gyroPointer) : COMETS3357::Subsystem("NoteDetection"){
    swerveField = swervePointer;
    limelightField = limelightPointer;
    gyroField = gyroPointer;
}

void NoteDetectionSubsystem::Initialize()
{

}

void NoteDetectionSubsystem::Periodic()
{
    if (limelightField->hasTarget())
    {
         double D = h*tan(theta+(limelightField->getY() * 3.14159 / 180));
            double Distance = D+(gr/2);
            y = Distance*std::sin(((-gyroField->m_navx.GetYaw() * 3.14159 / 180.0) +((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroField->angleOffset) - (limelightField->getX() * 3.141592654/180)) + 0.3*std::sin(((-gyroField->m_navx.GetYaw() * 3.14159 / 180.0) +((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroField->angleOffset));
            x = Distance*std::cos(((-gyroField->m_navx.GetYaw() * 3.14159 / 180.0) +((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroField->angleOffset) - (limelightField->getX() * 3.141592654/180)) + 0.3*std::cos(((-gyroField->m_navx.GetYaw() * 3.14159 / 180.0) +((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroField->angleOffset));
            m_field.SetRobotPose(frc::Pose2d{frc::Translation2d{units::meter_t{x + (double)swerveField->GetPose().X()}, units::meter_t{y + (double)swerveField->GetPose().Y()}}, frc::Rotation2d{units::radian_t{0}}});

            frc::SmartDashboard::PutData("GamepiecePosition", &m_field);
    }
    if (goingToNote) {
        // swerveField -> controllingSwerveRotation = false;
        // swerveField -> overrideRotation = -units::radians_per_second_t{std::clamp(limelightField->getX() * 3.14159 * 0.6 / 180, -0.5, 0.5)};

        frc::SmartDashboard::PutNumber("Xasdasdasd", x + (double)swerveField->GetPose().X());
        if (limelightField->hasTarget() && (x + (double)swerveField->GetPose().X()) > 8) {
           

            swerveField -> controllingSwerveMovement = false;
            swerveField -> overrideYSpeed = -units::meters_per_second_t{ std::clamp(std::sin(((-gyroField->m_navx.GetYaw() * 3.14159 / 180.0) +((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroField->angleOffset) - (limelightField->getX() * 3.141592654/180)), -0.5, 0.5)};
            swerveField -> overrideXSpeed = -units::meters_per_second_t{ std::clamp(std::cos(((-gyroField->m_navx.GetYaw() * 3.14159 / 180.0) +((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) + gyroField->angleOffset) - (limelightField->getX() * 3.141592654/180)), -0.5, 0.5)};
        }
        // else
        // {
        //     stopGoToNote();
        // }
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
}

