#pragma once 

#include "COMETS3357/Commands/Command.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/IndexerSubsytem.h"
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/LookupTable.h"
#include <cmath> 
#include <frc/DriverStation.h>

COMMAND_START(SourceShoot)

SourceShootCommand(ShooterSubsystem *shooter, IndexerSubsystem* indexer, COMETS3357::SwerveSubsystem* swerveSubsystem); 

ShooterSubsystem * shooterSubsystem; 
IndexerSubsystem * indexerSubsystem;
COMETS3357::SwerveSubsystem * swerve; 
frc::Translation2d targetPos; 


int cycle; 

COMMAND_END
/*
{
    "x": 8.28,
    "y": 5.77,
    "rotation": -165,
    "maxVelocity": 0.5,
    "maxRotation": 0.5,
    "avoid": 0,
    "TurnSpeaker": 0,
    "EndVelocity": 0
    },
    {
      "x": 4.38,
      "y": 6.59,
      "rotation": -155,
      "maxVelocity": 0.5,
      "maxRotation": 0.5,
      "avoid": 0,
      "TurnSpeaker": 0,
      "EndVelocity": 0
    },
    "Shoot",
    "wait 1",
    "StopShoot",
    "StopIntake",
    {
      "x": 8.28,
      "y": 4.14,
      "rotation": -155,
      "maxVelocity": 1,
      "maxRotation": 0.5,
      "avoid": 0,
      "TurnSpeaker": 0,
      "EndVelocity": 0
    },
    "Intake",
    {
      "x": 8.28,
      "y": 4.12,
      "rotation": 180,
      "maxVelocity": 0.5,
      "maxRotation": 0.5,
      "avoid": 0,
      "TurnSpeaker": 0,
      "EndVelocity": 0
    },
    [
    {
      "x":7.7,
      "y": 3.7,
      "rotation": 180,
      "maxVelocity": 10.5,
      "maxRotation": 0.5,
      "avoid": 0,
      "TurnSpeaker": 0,
      "EndVelocity": 0
    },
    "IntakeIndexer"
  ],
  "4PieceSetpoint42",
  [
    "IntakeIndexer",
    {
      "x":11.83,
      "y": 4.05,
      "rotation": 180,
      "maxVelocity": 0.5,
      "maxRotation": 0.5,
      "avoid": 0,
      "TurnSpeaker": 0,
      "EndVelocity": 0
    }
  ],
  "4PieceSetpoint42",
  [
    "IntakeIndexer",
    {
      "x":11.83,
      "y": 4.05,
      "rotation": -170,
      "maxVelocity": 0.5,
      "maxRotation": 0.5,
      "avoid": 0,
      "TurnSpeaker": 0,
      "EndVelocity": 0
    }
  ],
  "Shoot",
    "wait 1",
    "StopShoot",
    "StopIntake"

*/
