# pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxVelocity.h"
#include <frc/DigitalInput.h>
#include <string>

SUBSYSTEM_START(Indexer) 

IndexerSubsystem(); 

void SetVelocity(double velocity); 

void SetVelocity(std::string velocity); 

void SetPercent(double percent);

COMETS3357::SparkMaxVelocity IndexerMotor{"Indexer"}; 
bool IsDetected(); 
frc::DigitalInput noteDetector{0}; 

SUBSYSTEM_END