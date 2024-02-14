#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxPosition.h"


SUBSYSTEM_START(Elevator)

ElevatorSubsystem();

void SetPercent(double percent); 

void SetPosition(double position);

void SetPosition(std::string position);

COMETS3357::SparkMaxPosition elevatorMotor{"ElevatorMotor"}; 

SUBSYSTEM_END
