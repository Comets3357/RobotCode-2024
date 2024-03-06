#include "COMETS3357/Subsystems/Vision/LimelightSubsystem.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/Subsystems/Vision/NoteDetection.h"
#include "COMETS3357/GyroSubsystem.h"

NoteDetectionSubsystem::NoteDetectionSubsystem(COMETS3357::SwerveSubsystem *swervePointer, COMETS3357::LimelightSubsystem *limelightPointer, COMETS3357::GyroSubsystem *gyroPointer){
    swerveField = swervePointer;
    limelightField = limelightPointer;
    gyroField = gyroPointer;
}

void NoteDetectionSubsystem::Periodic()
{
    if (goingToNote) {
        swerveField -> controllingSwerveRotation = false;
        swerveField -> overrideRotation = std::clamp(limelightField->getX() * k, -0.5, 0.5);
        if (limelightField->hasTarget()) {
            swerveField -> controllingSwerveMovement = false;
            swerveField -> overrideXSpeed = std::clamp(std::sin(gyroField->m_navx.GetYaw() + (limelightPointer->getX() * 3.141592654/180)), -1, 1);
            swerveField -> overrideYSpeed = std::clamp(std::cos(gyroField->m_navx.GetYaw() + (limelightPointer->getX() * 3.141592654/180)), -1, 1);
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
    swerveField -> overrideXSpeed = 0;
    swerveField -> overrideYSpeed = 0;
    swerveField -> overrideRotation = 0;""
    swerveField -> controllingSwerveMovement = true;
    swerveField -> controllingSwerveRotation = true;
}

