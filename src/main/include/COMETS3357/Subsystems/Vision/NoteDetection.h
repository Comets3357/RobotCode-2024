#pragma once

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
double y = 0;
double x = 0;
double theta = 1.1344530556;
double gr = .18;
double h = .333;

frc::PIDController translatePID{1, 0, 0};
frc::PIDController rotPID{1, 0, 0};

bool autonNoteValid = true;

frc::Field2d m_field;

void goToNote();
void stopGoToNote();

SUBSYSTEM_END