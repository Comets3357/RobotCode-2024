#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/IntegerTopic.h>


SUBSYSTEM_START(LEDs)

LEDsSubsystem();

void writeLEDs(int r, int g, int b);

nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

std::shared_ptr< nt::NetworkTable > table = inst.GetTable("datatable");

nt::IntegerPublisher greenPub;
nt::IntegerPublisher redPub;
nt::IntegerPublisher bluePub;

SUBSYSTEM_END
