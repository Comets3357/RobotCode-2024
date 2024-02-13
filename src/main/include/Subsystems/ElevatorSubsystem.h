#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxPercent.h"


SUBSYSTEM_START(Elevator)

ElevatorSubsystem();

void SetPercent(double percent); 

void SetPercent(std::string percent); 

COMETS3357::SparkMaxPercent intakeMotor{"ElevatorMotor"}; 

SUBSYSTEM_END