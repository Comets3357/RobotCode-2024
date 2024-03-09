#pragma once

#include "COMETS3357/Commands/Command.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include <vector>
#include <frc/geometry/Translation2d.h>

COMMAND_START(AdvancedAutonPath)

AdvancedAutonPathCommand(COMETS3357::SwerveSubsystem *swerveSubsystem, double maxTurn, double maxSpeed, frc::Pose2d pose); 

COMETS3357::SwerveSubsystem *swerve; 

frc::Pose2d targetPose;

frc::PIDController translatePID{1, 0, 0};
frc::PIDController rotPID{1, 0, 0};

std::optional<frc::Translation2d> CheckIntersection(frc::Translation2d startPos, frc::Translation2d endPos, frc::Translation2d obstaclePos, units::meter_t rad);

std::vector<frc::Translation2d> trussPositions =
{
    {units::meter_t{3.3}, units::meter_t{4.12}},
    {units::meter_t{5.59}, units::meter_t{5.5}},
    {units::meter_t{5.57}, units::meter_t{2.85}},
    {units::meter_t{13.18}, units::meter_t{4.11}},
    {units::meter_t{10.98}, units::meter_t{2.85}},
    {units::meter_t{10.94}, units::meter_t{5.4}}
};

    frc::Field2d m_field;

units::meter_t radiusLimit = 1.25_m;

units::meter_t lowestIntersectDistance;
std::optional<frc::Translation2d> lowestIntersectPos;
int lowestIntersectObstacle;

frc::Pose2d idk{frc::Translation2d{16_m, 4_m}, frc::Rotation2d{units::radian_t{0}}};

double rot;
double speed;


COMMAND_END