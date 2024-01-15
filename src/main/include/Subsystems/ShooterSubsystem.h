#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxVelocity.h"

SUBSYSTEM_START(Shooter)

ShooterSubsystem();

void SetVelocityFlyWheel(double percent); 

void SetVelocityFlyWheel(std::string percent); 

void SetVelocityKSickerWheel(double percent); 

void SetVelocityKickerWheel(std::string percent);

COMETS3357::SparkMaxVelocity KickerWheelMotor{"KickerMotor"}; 
COMETS3357::SparkMaxVelocity FlyMotor{"FlyMotor"}; 

SUBSYSTEM_END