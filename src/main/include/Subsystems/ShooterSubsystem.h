#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxVelocity.h"
#include "Subsystems/IndexerSubsytem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxPosition.h"
#include "COMETS3357/LookupTable.h"
#include <frc/AnalogInput.h>
#include <frc/DutyCycle.h>
#include <frc/DigitalSource.h>


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
COMETS3357::LookupTable angleLookup{"AngleLookup"}; 
COMETS3357::LookupTable velocityLookup{"VelocityLookup"}; 
COMETS3357::LookupTable kickerWheelFFLookup{"KickerWheelLookup"};
COMETS3357::LookupTable flyWheelFFLookup{"FlyWheelLookup"};

 frc::DigitalInput inputPivot{1};
 frc::DutyCycle pivotAbsoluteEncoder = frc::DutyCycle{inputPivot};

SUBSYSTEM_END