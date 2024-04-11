#pragma once

#include "COMETS3357/Commands/Command.h"
#include "COMETS3357/Subsystems/Vision/NoteDetection.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/GyroSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include "Subsystems/IndexerSubsytem.h"
#include "COMETS3357/Subsystems/Vision/LimelightSubsystem.h"
#include "COMETS3357/GyroSubsystem.h"
#include <frc/smartdashboard/Field2d.h>

COMMAND_START(MiddleNoteDetection)


MiddleNoteDetectionCommand(NoteDetectionSubsystem *noteDetection, COMETS3357::SwerveSubsystem* swerve, IndexerSubsystem* indexer, COMETS3357::LimelightSubsystem* limelight, COMETS3357::GyroSubsystem* gyro, frc::Translation2d estimatedGamepiecePose);

NoteDetectionSubsystem* noteDetectionSubsystem;
IndexerSubsystem* indexerSubsystem;
COMETS3357::SwerveSubsystem* swerveSubsystem;
COMETS3357::LimelightSubsystem* limelightSubsystem;
COMETS3357::GyroSubsystem* gyroSubsystem;

double y = 0;
double x = 0;
double theta = 1.1344530556;
double gr = .18;
double h = .333;

frc::PIDController translatePID{1, 0, 0};
frc::PIDController rotPID{1, 0, 0};

frc::Pose2d targetPose;

frc::Translation2d pose;


bool gotPosition = false;

COMMAND_END

