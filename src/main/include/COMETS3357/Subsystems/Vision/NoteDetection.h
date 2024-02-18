#include "COMETS3357/Subsystems/Subsystem.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"

SUBSYSTEM_START(NoteDetection)
NoteDetectionSubsystem(COMETS3357::SwerveSubsystem *swervePointer, COMETS3357::LimelightSubsystem *limelightPointer, COMETS3357::GyroSubsystem *gyroPointer);
COMETS3357::SwerveSubsystem swerveField;
COMETS3357::LimelightSubsystem limelightField;
COMETS3357::GyroSubsystem gyroField;

void goToNote();

SUBSYSTEM_END