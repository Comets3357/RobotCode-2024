# pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxPercent.h"
#include <frc/DigitalInput.h>
#include <string>

SUBSYSTEM_START(Amp) 

AmpSubsystem(); 

void SetPercent(double percent);

COMETS3357::SparkMaxPercent AmpMotor{"Indexer"}; 

SUBSYSTEM_END