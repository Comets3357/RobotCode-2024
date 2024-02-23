#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/IntegerTopic.h>
#include <frc/DriverStation.h>
#include "Subsystems/IndexerSubsytem.h"
#include "COMETS3357/Configs/ControllerMap.h"
#include "RobotContainer.h"


SUBSYSTEM_START(LEDs)

LEDsSubsystem();

void writeLEDs(int r, int g, int b);

bool hpSignal = false; 
bool ampSignal = false; 
bool comms = frc::DriverStation::IsDSAttached; 

nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

bool enabled = frc::DriverStation::IsEnabled();  
bool gyroZero = false; 


std::shared_ptr< nt::NetworkTable > table = inst.GetTable("datatable");

nt::IntegerPublisher greenPub;
nt::IntegerPublisher redPub;
nt::IntegerPublisher bluePub;


SUBSYSTEM_END
