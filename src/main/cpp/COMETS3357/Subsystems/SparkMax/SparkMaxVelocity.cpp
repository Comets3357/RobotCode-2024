#include "COMETS3357/Subsystems/SparkMax/SparkMaxVelocity.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <COMETS3357/Subsystems/SubsystemManager.h>

#define FORCEINIT

using namespace COMETS3357;

SparkMaxVelocity::SparkMaxVelocity(std::string configName) : config{ConfigFiles::getInstance().GetConfigFiles().sparkMaxVelocityConfigs[configName]},
motor{config.ID, rev::CANSparkMax::MotorType::kBrushless}, encoder{motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)}, PIDController{motor.GetPIDController()}
{
    config.motor = this;
    COMETS3357::SubsystemManager::GetInstance().AddInit([this]{RobotInit();});
}

void SparkMaxVelocity::RobotInit()
{

    if (
        #ifdef FORCEINIT
        true
        #else
        motor.GetInverted() != config.invertedRelative || 
        motor.GetIdleMode() != config.idleMode || 
        encoder.GetPositionConversionFactor() != config.relativePositionConversionFactor ||
        encoder.GetVelocityConversionFactor() != config.relativeVelocityConversionFactor || 
        motor.IsFollower() != (config.follow == "NONE")
        #endif
    )
    {

        motor.RestoreFactoryDefaults();
        motor.EnableVoltageCompensation(10);
        motor.SetInverted(config.invertedRelative);
        motor.SetSmartCurrentLimit(config.currentLimit);
        motor.SetIdleMode(config.idleMode);
        SetVelocityPID(config.velocityPID);
        encoder.SetPositionConversionFactor(config.relativePositionConversionFactor);
        encoder.SetVelocityConversionFactor(config.relativeVelocityConversionFactor);

        if (config.follow != "NONE")
        {
            motor.Follow(COMETS3357::ConfigFiles::getInstance().GetConfigFiles().sparkMaxVelocityConfigs[config.follow].motor->motor);
        }

        motor.BurnFlash();
    }
}

void SparkMaxVelocity::SetVelocityPID(PID velocityPID)
{
    pid = velocityPID;
    PIDController.SetP(pid.P, 0);
    PIDController.SetI(pid.I, 0);
    PIDController.SetD(pid.D, 0);
    PIDController.SetFF(pid.FF, 0);
}

void SparkMaxVelocity::SetVelocity(double velocity)
{
    PIDController.SetReference(velocity, rev::CANSparkMax::ControlType::kVelocity, 0);
}

void SparkMaxVelocity::SetVelocity(double velocity, double feedForward)
{
    SetVelocityPID(PID{0, 0, 0, feedForward});
    PIDController.SetReference(velocity, rev::CANSparkMax::ControlType::kVelocity, 0);
}

void SparkMaxVelocity::SetVelocity(std::string velocity)
{
    PIDController.SetReference(config.velocities[velocity], rev::CANSparkMax::ControlType::kVelocity, 0);
}

double SparkMaxVelocity::GetRelativeVelocity()
{
    return encoder.GetVelocity();
}

void SparkMaxVelocity::SetPercent(double power)
{
    motor.Set(power);
}

PID SparkMaxVelocity::GetPID()
{
    return pid;
}

double SparkMaxVelocity::GetRelativePosition()
{
    return encoder.GetPosition();
}


void SparkMaxVelocity::SetRelativePosition(double position)
{
    encoder.SetPosition(position);
}