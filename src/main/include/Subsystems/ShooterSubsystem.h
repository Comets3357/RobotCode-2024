#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxVelocity.h"
#include "Subsystems/IndexerSubsytem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxPosition.h"
#include "COMETS3357/LookupTable.h"
#include <frc/AnalogInput.h>
#include <frc/DutyCycle.h>
#include <frc/DigitalSource.h>
#include <frc/geometry/Pose2d.h>
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include "COMETS3357/GyroSubsystem.h"


SUBSYSTEM_START(Shooter)

ShooterSubsystem(COMETS3357::SwerveSubsystem* swerveSubsystem, COMETS3357::GyroSubsystem* gyroSubsystem);

void SetVelocityFlyWheel(double percent); 

void SetVelocityFlyWheel(std::string percent); 

void SetVelocityKickerWheel(double percent); 

void SetVelocityKickerWheel(std::string percent);

void SetPercentKickerWheel(double percent); 

void SetPercentFlyWheel(double percent);

double GetVelocityKickerWheel();

double GetVelocityFlyWheel();

void SetPositionPivot(double position); 

void SetPercentPivot(double percent); 

double GetPivotRelativePosition();
double GetPivotAbsolutePosition();
void SetPositionPivot(std::string position); 

double offset = 0; 


bool pivotInRelative = false;

std::pair<double, double> calculateDistanceTravelled(std::pair<double, double> velocity, double time); 

std::pair<double, double> calculateFinalPosition(std::pair<double, double> initial, std::pair<double, double> travel); 


void startTurnToTarget();
void stopTurnToTarget();
void startTurnPassZone(); 

bool turningTowardsTarget = false;

COMETS3357::SparkMaxVelocity KickerWheel{"KickerWheel"}; 
COMETS3357::SparkMaxVelocity FlyWheel{"FlyWheel"}; 
COMETS3357::SparkMaxPosition Pivot{"Pivot"}; 
COMETS3357::LookupTable angleLookup{"AngleLookup"}; 
COMETS3357::LookupTable velocityLookup{"VelocityLookup"}; 
COMETS3357::LookupTable kickerWheelFFLookup{"KickerWheelLookup"};
COMETS3357::LookupTable flyWheelFFLookup{"FlyWheelLookup"};
COMETS3357::LookupTable SourceSpeed{"SourceShootVelocity"};
COMETS3357::LookupTable SourceAngle{"SourceShootAngle"};


 frc::DigitalInput inputPivot{1};
 frc::DutyCycle pivotAbsoluteEncoder = frc::DutyCycle{inputPivot};

COMETS3357::LookupTable rotationPLookup{"RotationP"};
frc::Translation2d targetPos; 
frc::Translation2d passPos; 

frc::PIDController turnToPID{1, 0, 0};

COMETS3357::SwerveSubsystem* swerve;
COMETS3357::GyroSubsystem* gyro;

SUBSYSTEM_END