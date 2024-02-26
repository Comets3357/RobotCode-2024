#include "Subsystems/AutonResetGyroButtonSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>


AutonGyroResetSubsystem::AutonGyroResetSubsystem(COMETS3357::GyroSubsystem* gyroSubsystem, LEDsSubsystem* ledsSubsystem) : COMETS3357::Subsystem("AutonGyroResetSubsystem") {
    gyro = gyroSubsystem;
    leds = ledsSubsystem;
}

void AutonGyroResetSubsystem::Initialize()
{

}

void AutonGyroResetSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean("Button", resetButton.Get());
    if (!resetButton.Get())
    {
        leds->gyroZero = true;
        gyro->ZeroGyro();
    }
}