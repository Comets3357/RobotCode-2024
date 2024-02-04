#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxVelocity.h"
#include "Subsystems/IndexerSubsytem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxPosition.h"
#include "COMETS3357/LookupTable.h"
#include <frc/AnalogInput.h>


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

void SetPositionPivot(double position); 

void SetPercentPivot(double percent); 

double GetPivotRelativePosition();
double GetPivotAbsolutePosition();
void SetPositionPivot(std::string position); 


std::pair<double, double> calculateDistanceTravelled(std::pair<double, double> velocity, double time); 

std::pair<double, double> calculateFinalPosition(std::pair<double, double> initial, std::pair<double, double> travel); 


COMETS3357::SparkMaxVelocity KickerWheel{"KickerWheel"}; 
COMETS3357::SparkMaxVelocity FlyWheel{"FlyWheel"}; 
COMETS3357::SparkMaxPosition Pivot{"Pivot"}; 
COMETS3357::LookupTable Table{"AngleLookUp"}; 

frc::AnalogInput pivotEncoder{1};

SUBSYSTEM_END