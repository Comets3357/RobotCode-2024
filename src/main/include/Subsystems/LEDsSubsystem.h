#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"
#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/IntegerTopic.h>
#include <frc/DriverStation.h>
#include "Subsystems/IndexerSubsytem.h"
#include "Commands/IntakeIndexerCommand.h"
#include "COMETS3357/Configs/ControllerMap.h"
#include <frc/AddressableLED.h>


SUBSYSTEM_START(LEDs)

LEDsSubsystem(IndexerSubsystem* indexer);

void writeLEDs(int r, int g, int b);

bool hpSignal = false; 
bool ampSignal = false; 
bool comms = false; 
bool detect =false;
bool enabled = false;  
bool gyroZero = false; 


std::shared_ptr< nt::NetworkTable > table = nt::NetworkTableInstance::GetDefault().GetTable("datatable");
IndexerSubsystem* indexerSubsytem;

nt::IntegerPublisher greenPub;
nt::IntegerPublisher redPub;
nt::IntegerPublisher bluePub;
nt::IntegerSubscriber frameSub;

frc::AddressableLED m_led{9};
std::array<frc::AddressableLED::LEDData, 50> ledBuffer;


double lastTestTimestamp = 0;
double lastFrame = 0;

SUBSYSTEM_END
