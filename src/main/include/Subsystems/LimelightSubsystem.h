#pragma once

#include "COMETS3357/Subsystems/Subsystem.h"

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

SUBSYSTEM_START(Limelight)

LimelightSubsystem();

std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
double targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
double targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
double targetArea = table->GetNumber("ta", 0.0);
double targetSkew = table->GetNumber("ts", 0.0);
double targetDetected = table->GetNumber("tv", 0.0);

void makeRobotFaceTarget(double, double);

SUBSYSTEM_END