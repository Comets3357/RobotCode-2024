#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxVelocity.h"
#include "Subsystems/IndexerSubsytem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxPosition.h"

SUBSYSTEM_START(Shooter)

ShooterSubsystem();

void SetVelocityFlyWheel(double percent); 

void SetVelocityFlyWheel(std::string percent); 

void SetVelocityKickerWheel(double percent); 

void SetVelocityKickerWheel(std::string percent);

void SetPercentKickerWheel(double percent); 

void SetPercentFlyWheel(double percent);

double GetVelocityKickerWheel();

double GetVelocityFlyWheel();

void SetVelocityIndexer();

void SetPositionPivot(double position); 

void SetPercentPivot(double percent); 

void SetPositionPivot(std::string position); 

COMETS3357::SparkMaxVelocity KickerWheel{"KickerWheel"}; 
COMETS3357::SparkMaxVelocity FlyWheel{"FlyWheel"}; 
COMETS3357::SparkMaxPosition Pivot{"Pivot"}; 


SUBSYSTEM_END