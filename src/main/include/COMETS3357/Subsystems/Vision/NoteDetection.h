#include "COMETS3357/Subsystems/Subsystem.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/Subsystems/Vision/LimelightSubsystem.h"
#include "COMETS3357/GyroSubsystem.h"

SUBSYSTEM_START(NoteDetection)


NoteDetectionSubsystem(COMETS3357::SwerveSubsystem* swervePointer, COMETS3357::LimelightSubsystem *limelightPointer, COMETS3357::GyroSubsystem *gyroPointer);
COMETS3357::SwerveSubsystem* swerveField;
COMETS3357::LimelightSubsystem* limelightField;
COMETS3357::GyroSubsystem* gyroField;

bool goingToNote = false;
double theta = 1.1344530556;
double gr = .18;
double h = .32;

void goToNote();
void stopGoToNote();

SUBSYSTEM_END