#include "COMETS3357/Subsystems/SparkMax/SparkMaxPosition.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <COMETS3357/Subsystems/SubsystemManager.h>

#define FORCEINIT

using namespace COMETS3357;

SparkMaxPosition::SparkMaxPosition(std::string configName)
    : config{ConfigFiles::getInstance().GetConfigFiles().sparkMaxPositionConfigs[configName]},
     motor{config.ID, rev::CANSparkMax::MotorType::kBrushless},
    PIDController{motor.GetPIDController()},
    relativeEncoder{motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
    absoluteEncoder{motor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)}
{
    defaultRunMode = config.defaultMode;
    runMode = defaultRunMode;

    changeRunMode(defaultRunMode);
    config.motor = this;

   COMETS3357::SubsystemManager::GetInstance().AddInit([this]{RobotInit();});

};

SparkMaxPosition::SparkMaxPosition(std::string configName, bool setAbsoluteOffset)
    : config{ConfigFiles::getInstance().GetConfigFiles().sparkMaxPositionConfigs[configName]},
     motor{config.ID, rev::CANSparkMax::MotorType::kBrushless},
    PIDController{motor.GetPIDController()},
    relativeEncoder{motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
    absoluteEncoder{motor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)}
{
    defaultRunMode = config.defaultMode;
    runMode = defaultRunMode;

    changeRunMode(defaultRunMode);

    setAbsPos = setAbsoluteOffset;
    config.motor = this;

    COMETS3357::SubsystemManager::GetInstance().AddInit([this]{RobotInit();});

};

void SparkMaxPosition::RobotInit()
{

    if ( 

        #ifdef FORCEINIT
        true
        #else
        motor.GetInverted() != config.invertedRelative || 
        motor.GetIdleMode() != config.idleMode || 
        PIDController.GetOutputMin(1) != config.minSpeed ||
        PIDController.GetOutputMax(1) != config.maxSpeed ||
        relativeEncoder.GetPositionConversionFactor() != config.relativePositionConversionFactor ||
        relativeEncoder.GetVelocityConversionFactor() != config.relativeVelocityConversionFactor ||
        absoluteEncoder.GetInverted() != config.invertedAbsolute ||
        absoluteEncoder.GetPositionConversionFactor() != config.absolutePositionConversionFactor ||
        absoluteEncoder.GetVelocityConversionFactor() != config.absoluteVelocityConversionFactor ||
        PIDController.GetPositionPIDWrappingEnabled() != config.positionPIDWrappingEnabled ||
        PIDController.GetPositionPIDWrappingMinInput() != config.turningEncoderPositionPIDMinInput ||
        PIDController.GetPositionPIDWrappingMaxInput() != config.turningEncoderPositionPIDMaxInput
        #endif
   )
    {
        motor.RestoreFactoryDefaults();
        motor.SetInverted(config.invertedRelative);
        motor.SetSmartCurrentLimit(config.currentLimit);
        motor.SetIdleMode(config.idleMode);
        PIDController.SetOutputRange(config.minSpeed, config.maxSpeed, 1);
        SetVelocityPID(config.velocityPID);
        SetPositionPID(config.positionPID);
        relativeEncoder.SetPositionConversionFactor(config.relativePositionConversionFactor);
        relativeEncoder.SetVelocityConversionFactor(config.relativeVelocityConversionFactor);
        absoluteEncoder.SetInverted(config.invertedAbsolute);
        absoluteEncoder.SetPositionConversionFactor(config.absolutePositionConversionFactor);
        absoluteEncoder.SetVelocityConversionFactor(config.absoluteVelocityConversionFactor);
        if (setAbsPos)
        {
            absoluteEncoder.SetZeroOffset(config.absoluteZeroOffset);
        }


        PIDController.SetPositionPIDWrappingEnabled(config.positionPIDWrappingEnabled);
        PIDController.SetPositionPIDWrappingMinInput(config.turningEncoderPositionPIDMinInput);
        PIDController.SetPositionPIDWrappingMaxInput(config.turningEncoderPositionPIDMaxInput);

        motor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, config.forwardSoftLimitEnabled);
        motor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, config.reverseSoftLimitEnabled);
        motor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, config.forwardSoftLimit);
        motor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, config.reverseSoftLimit);
        

        // if (config.follow != "NONE")
        // {
        //     motor.Follow(COMETS3357::ConfigFiles::getInstance().GetConfigFiles().sparkMaxPositionConfigs[config.follow].motor->motor);
        // }
        motor.BurnFlash();
        
    }

    relativeEncoder.SetPosition(0);


        

}

void SparkMaxPosition::ZeroRelativeEncoder()
{
    relativeEncoder.SetPosition(absoluteEncoderPosition);
}

void SparkMaxPosition::SetPower(double percent)
{
    motor.Set(percent);
    
}

void SparkMaxPosition::ChangeFeedBackDevice(SparkMaxPositionRunMode mode)
{
    switch (mode)
    {
    case POSITION_SPARK_MAX_ABSOLUTE:
        PIDController.SetFeedbackDevice(absoluteEncoder);
        break;
    case POSITION_SPARK_MAX_RELATIVE:
        PIDController.SetFeedbackDevice(relativeEncoder);
        break;
    default:
        break;
    }
}

void SparkMaxPosition::SetVelocityPID(PID pid)
{
    PIDController.SetP(pid.P, 0);
    PIDController.SetI(pid.I, 0);
    PIDController.SetD(pid.D, 0);
    PIDController.SetFF(pid.FF, 0);

    velocityPID = pid;
}

void SparkMaxPosition::SetPositionPID(PID pid)
{
    PIDController.SetP(pid.P, 1);
    PIDController.SetI(pid.I, 1);
    PIDController.SetD(pid.D, 1);
    PIDController.SetFF(pid.FF, 1);

    positionPID = pid;
}


void SparkMaxPosition::SetPIDOutputRange(double min, double max, int slot)
{
    PIDController.SetOutputRange(min, max, slot);
}

double SparkMaxPosition::GetPosition()
{
    switch (runMode)
    {
    case POSITION_SPARK_MAX_ABSOLUTE:
        return absoluteEncoderPosition;
        break;
    case POSITION_SPARK_MAX_RELATIVE:
        return relativeEncoderPosition;
        break;
    default:
        return 0;
        break;
    }
}

void SparkMaxPosition::SetVelocity(double velocity)
{
    commandVelocityError = PIDController.SetReference(velocity, rev::CANSparkMax::ControlType::kVelocity, 0);
}

void SparkMaxPosition::SetPosition(double position)
{
    commandPositionError = PIDController.SetReference(position, rev::CANSparkMax::ControlType::kPosition, 1, feedForwardFunction(absoluteEncoderPosition));
}

void SparkMaxPosition::SetPosition(std::string position)
{
    commandPositionError = PIDController.SetReference(config.positions[position], rev::CANSparkMax::ControlType::kPosition, 1, feedForwardFunction(absoluteEncoderPosition));
}

double SparkMaxPosition::GetRelativePosition()
{
    return relativeEncoderPosition;
}

double SparkMaxPosition::GetAbsolutePosition()
{
    return absoluteEncoderPosition;
}

double SparkMaxPosition::GetRelativeVelocity()
{
    return relativeEncoder.GetVelocity();
}

double SparkMaxPosition::GetAbsoluteVelocity()
{
    return absoluteEncoder.GetVelocity();
}

void SparkMaxPosition::Periodic()
{

    absoluteEncoderPosition = absoluteEncoder.GetPosition();
    relativeEncoderPosition = absoluteEncoder.GetPosition();

    

    CheckAbsoluteEncoder();
    if (runMode == POSITION_SPARK_MAX_ABSOLUTE && std::abs(absoluteEncoderPosition - relativeEncoderPosition) < 10)
    {
        ZeroRelativeEncoder();
    }
    if (motor.GetLastError() != rev::REVLibError::kOk)
    {
        std::cout << "Motor ID: " << config.ID << " " << ConvertError((int)motor.GetLastError()) << std::endl;
    }
    if (motor.GetOutputCurrent() < 1 & relativeEncoder.GetVelocity() == 0) 
    // Tweak output current
    {
        // std::cout << "Motor ID: " << config.id << " is recieving power but nothing is happening" << std::endl;
    }
    if (commandPositionError != rev::REVLibError::kOk)
    {
        std::cout << "Motor ID: " << config.ID << " " << ConvertError((int)motor.GetLastError()) << " | Position Error" << std::endl;
    }
    if (commandVelocityError != rev::REVLibError::kOk)
    {
        std::cout << "Motor ID: " << config.ID << " " << ConvertError((int)motor.GetLastError()) << " | Velocity Error" << std::endl;
    }
    if (motor.GetOutputCurrent() > outputCurrentLimit)
    {
        std::cout << "Motor ID: " << config.ID << " Output Current Exceeding Limit of " << outputCurrentLimit << std::endl;
    }
    

}

void SparkMaxPosition::changeRunMode(SparkMaxPositionRunMode mode)
{
    runMode = mode;
    ChangeFeedBackDevice(runMode);
}

void SparkMaxPosition::SetFeedForward(std::function<double(double)> feedforward)
{
    feedForwardFunction = feedforward;
}

std::string SparkMaxPosition::ConvertError(int error)
{
    switch (error)
        {
            case 1: return ("kError: General error condition"); break;
            case 2: return ("kTimeout: Operation or task exceeded the allowed time limit"); break;
            case 3: return ("kNotImplemented: The functionality or feature is not implemented yet"); break;
            case 4: return ("kHALError: Error related to High Availability (HA) functionality"); break;
            case 5: return ("kCantFindFirmware: Unable to locate or load required firmware"); break;
            case 6: return ("kFirmwareTooOld: The firmware version is too old to be compatible"); break;
            case 7: return ("kFirmwareTooNew: The firmware version is too new to be compatible"); break;
            case 8: return ("kParamInvalidID: Invalid parameter ID"); break;
            case 9: return ("kParamMismatchType: Parameter type mismatch"); break;
            case 10: return ("kParamAccessMode: Invalid parameter access mode"); break;
            case 11: return ("kParamInvalid: Invalid parameter"); break;
            case 12: return ("kParamNotImplementedDeprecated: Parameter not implemented or deprecated"); break;
            case 13: return ("kFollowConfigMismatch: Configuration mismatch for follower device"); break;
            case 14: return ("kInvalid: Generic invalid state or condition"); break;
            case 15: return ("kSetpointOutOfRange: Setpoint value is out of the allowed range"); break;
            case 16: return ("kUnknown: Unknown or unspecified error"); break;
            case 17: return ("kCANDisconnected: Disconnected from Controller Area Network (CAN)"); break;
            case 18: return ("kDuplicateCANId: Duplicate Controller Area Network (CAN) identifier"); break;
            case 19: return ("kInvalidCANId: Invalid Controller Area Network (CAN) identifier"); break;
            case 20: return ("kSparkMaxDataPortAlreadyConfiguredDifferently: Configuration conflict for a data port on Spark MAX device"); break;
        }
}


void SparkMaxPosition::CheckAbsoluteEncoder()
{
    // if (runMode != POSITION_SPARK_MAX_ABSOLUTE) {
    //     return;
    // }


    // if (lastPosition != absoluteEncoderPosition)
    // {
    //     absAttempts++;
    // }
    // else
    // {
    //     absAttempts = 0;
    // }
    // lastPosition = absoluteEncoderPosition;

    // if (absAttempts > 20)
    // {
    //     changeRunMode(POSITION_SPARK_MAX_RELATIVE);
    // }
}