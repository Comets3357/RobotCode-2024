#include "COMETS3357/Configs/ConfigFiles.h"


#include <COMETS3357/Subsystems/SparkMax/SparkMaxPosition.h>
#include <COMETS3357/Subsystems/SparkMax/SparkMaxPercent.h>
#include <COMETS3357/Subsystems/SparkMax/SparkMaxVelocity.h>

using namespace COMETS3357;

RobotConfig& ConfigFiles::GetConfigFiles()
{
    if (!initialized)
    {
        LoadConfigFiles("Comp");
        initialized = true;
    }
    return robotConfig;
}

void ConfigFiles::LoadConfigFiles(std::string fileName)
{
   
    std::ifstream jsonFile(frc::filesystem::GetDeployDirectory() + "/ConfigFiles/" + fileName + ".json");
    if (!jsonFile.is_open()) {
        std::cerr << "Failed to open file" << std::endl;
    }

    std::string jsonString((std::istreambuf_iterator<char>(jsonFile)), std::istreambuf_iterator<char>());

    picojson::value jsonValue;
    std::string error = picojson::parse(jsonValue, jsonString);
    if (!error.empty()) {
        std::cerr << "JSON parse error: " << error << std::endl;
    }

    picojson::array sparkMaxPositionConfigs = jsonValue.get("SparkMaxPositionConfigs").get<picojson::array>();
    picojson::array sparkMaxPercentConfigs = jsonValue.get("SparkMaxPercentConfigs").get<picojson::array>();
    picojson::array sparkMaxVelocityConfigs = jsonValue.get("SparkMaxVelocityConfigs").get<picojson::array>();
    picojson::array swerveConfigs = jsonValue.get("SwerveConfigs").get<picojson::array>();
    picojson::array swerveModuleConfigs = jsonValue.get("SwerveModuleConfigs").get<picojson::array>();
    picojson::array sparkMaxPWMConfigs = jsonValue.get("SparkMaxPWMConfigs").get<picojson::array>();

    for (auto& config : swerveModuleConfigs)
    {
        SwerveModuleConfig moduleConfig;
        moduleConfig.azimuthConfigName = config.get("Azimuth").get<std::string>();
        moduleConfig.driveConfigName = config.get("Drive").get<std::string>();
        moduleConfig.angularOffset = config.get("AngularOffset").get<double>();

        robotConfig.swerveModuleConfigs[config.get("Name").get<std::string>()] = moduleConfig;
    }

    for (auto& config : sparkMaxPWMConfigs)
    {
        SparkMaxPWMConfig sparkMaxPWMConfig;

        sparkMaxPWMConfig.ID = (int)config.get("ID").get<double>();
        sparkMaxPWMConfig.inverted = config.get("Inverted").get<bool>();

        robotConfig.sparkMaxPWMConfigs[config.get("Name").get<std::string>()] = sparkMaxPWMConfig;


    }

    for (auto& config : swerveConfigs)
    {
        SwerveConfig swerveConfig;

        swerveConfig.frontLeftModule = robotConfig.swerveModuleConfigs[config.get("FrontLeftModule").get<std::string>()];
        swerveConfig.frontRightModule = robotConfig.swerveModuleConfigs[config.get("FrontRightModule").get<std::string>()];
        swerveConfig.backLeftModule = robotConfig.swerveModuleConfigs[config.get("BackLeftModule").get<std::string>()];
        swerveConfig.backRightModule = robotConfig.swerveModuleConfigs[config.get("BackRightModule").get<std::string>()];

        swerveConfig.trackWidth = units::meter_t{config.get("TrackWidth").get<double>()};
        swerveConfig.wheelBase = units::meter_t{config.get("WheelBase").get<double>()};
        swerveConfig.maxSpeed = units::meters_per_second_t{config.get("MaxSpeed").get<double>()};
        swerveConfig.maxTurnSpeed = units::radians_per_second_t{config.get("MaxTurnSpeed").get<double>()};

        swerveConfig.directionSlewRate = config.get("DirectionSlewRate").get<double>();
        swerveConfig.magnitudeSlewRate = config.get("MagnitudeSlewRate").get<double>();
        swerveConfig.rotationalSlewRate = config.get("RotationalSlewRate").get<double>();

        robotConfig.swerveConfigs[config.get("Name").get<std::string>()] = swerveConfig;
    }

    for (auto& config : sparkMaxPositionConfigs) {
        SparkMaxPositionConfig motorConfig;

        if (!config.contains("Name")) std::cerr << "SparkMaxVelocity Config Does not containe \"Name\"" << std::endl;
        std::string deviceName = config.get("Name").get<std::string>();
        SetConfigValue<int>(config, deviceName, "ID", motorConfig.ID, 0);
        std::string runModeString;
        SetConfigValue<std::string>(config, deviceName, "DefaultRunMode", runModeString, "NONE");
        motorConfig.defaultMode = (runModeString == "Absolute") ? COMETS3357::SparkMaxPositionRunMode::POSITION_SPARK_MAX_ABSOLUTE : COMETS3357::SparkMaxPositionRunMode::POSITION_SPARK_MAX_RELATIVE;
        if (runModeString == "NONE") motorConfig.defaultMode = COMETS3357::SparkMaxPositionRunMode::POSITION_SPARK_MAX_NONE;
        SetConfigValue<bool>(config, deviceName, "InvertedAbsolute", motorConfig.invertedAbsolute, false);
        SetConfigValue<bool>(config, deviceName, "InvertedRelative", motorConfig.invertedRelative, false);
        SetConfigValue<double>(config, deviceName, "CurrentLimit", motorConfig.currentLimit, 0);
        SetConfigValue<double>(config, deviceName, "RelativePositionConversionFactor", motorConfig.relativePositionConversionFactor, 1);
        SetConfigValue<double>(config, deviceName, "RelativeVelocityConversionFactor", motorConfig.relativeVelocityConversionFactor, 1);
        SetConfigValue<double>(config, deviceName, "AbsolutePositionConversionFactor", motorConfig.absolutePositionConversionFactor, 1);
        SetConfigValue<double>(config, deviceName, "AbsoluteVelocityConversionFactor", motorConfig.absoluteVelocityConversionFactor, 1);
        SetConfigValue<double>(config, deviceName, "AbsoluteZeroOffset", motorConfig.absoluteZeroOffset, 0);
        SetConfigValue<double>(config, deviceName, "MaxSpeed", motorConfig.maxSpeed, 0);
        SetConfigValue<double>(config, deviceName, "MinSpeed", motorConfig.minSpeed, 0);
        std::string idleModeString;
        SetConfigValue<std::string>(config, deviceName, "IdleMode", idleModeString, "Brake");
        motorConfig.idleMode = (idleModeString == "Brake") ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast;
        SetConfigValue<std::string>(config, deviceName, "IdleMode", motorConfig.follow, "NONE");
        if (!config.contains("VelocityPID")) printNotContains(config.get("Name").get<std::string>(), "VelocityPID");
        SetConfigValue<double>(config, deviceName, "VelocityPID", "P", motorConfig.velocityPID.P, 0);
        SetConfigValue<double>(config, deviceName, "VelocityPID", "I", motorConfig.velocityPID.I, 0);
        SetConfigValue<double>(config, deviceName, "VelocityPID", "D", motorConfig.velocityPID.D, 0);
        SetConfigValue<double>(config, deviceName, "VelocityPID", "FF", motorConfig.velocityPID.FF, 0);
        if (!config.contains("PositionPID")) printNotContains(config.get("Name").get<std::string>(), "VelocityPID");
        SetConfigValue<double>(config, deviceName, "PositionPID", "P", motorConfig.positionPID.P, 0);
        SetConfigValue<double>(config, deviceName, "PositionPID", "I", motorConfig.positionPID.I, 0);
        SetConfigValue<double>(config, deviceName, "PositionPID", "D", motorConfig.positionPID.D, 0);
        SetConfigValue<double>(config, deviceName, "PositionPID", "FF", motorConfig.positionPID.FF, 0);
        SetConfigValue<bool>(config, deviceName, "PositionPIDWrappingEnabled", motorConfig.positionPIDWrappingEnabled, false);
        SetConfigValue<double>(config, deviceName, "TurningEncoderPositionPIDMinInput", motorConfig.turningEncoderPositionPIDMinInput, 0);
        SetConfigValue<double>(config, deviceName, "TurningEncoderPositionPIDMaxInput", motorConfig.turningEncoderPositionPIDMaxInput, 0);
        SetConfigValue<bool>(config, deviceName, "ForwardSoftLimitEnabled", motorConfig.forwardSoftLimitEnabled, false);
        SetConfigValue<bool>(config, deviceName, "ReverseSoftLimitEnabled", motorConfig.reverseSoftLimitEnabled, false);
        SetConfigValue<double>(config, deviceName, "ForwardSoftLimit", motorConfig.forwardSoftLimit, 0);
        SetConfigValue<double>(config, deviceName, "ReverseSoftLimit", motorConfig.reverseSoftLimit, 0);

        picojson::object positions = config.get("Positions").get<picojson::object>();
        for (auto& position : positions)
        {
            motorConfig.positions[position.first] = position.second.get<double>();
        }

        robotConfig.sparkMaxPositionConfigs[config.get("Name").get<std::string>()] = motorConfig;

    }

    for (auto& config : sparkMaxVelocityConfigs)
    {
    
        SparkMaxVelocityConfig motorConfig;

        if (!config.contains("Name")) std::cerr << "SparkMaxVelocity Config Does not containe \"Name\"" << std::endl;
        std::string deviceName = config.get("Name").get<std::string>();
        SetConfigValue<int>(config, deviceName, "ID", motorConfig.ID, 0);
        SetConfigValue<bool>(config, deviceName, "InvertedRelative", motorConfig.invertedRelative, false);
        SetConfigValue<double>(config, deviceName, "CurrentLimit", motorConfig.currentLimit, 20);
        SetConfigValue<double>(config, deviceName, "RelativePositionConversionFactor", motorConfig.relativePositionConversionFactor, 1);
        SetConfigValue<double>(config, deviceName, "RelativeVelocityConversionFactor", motorConfig.relativeVelocityConversionFactor, 1);
        std::string idleModeString;
        SetConfigValue<std::string>(config, deviceName, "IdleMode", idleModeString, "Brake");
        motorConfig.idleMode = (idleModeString == "Brake") ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast;
        SetConfigValue<std::string>(config, deviceName, "Follow", motorConfig.follow, "NONE");
        if (!config.contains("VelocityPID")) printNotContains(config.get("Name").get<std::string>(), "VelocityPID");
        SetConfigValue<double>(config, deviceName, "VelocityPID", "P", motorConfig.velocityPID.P, 0);
        SetConfigValue<double>(config, deviceName, "VelocityPID", "I", motorConfig.velocityPID.I, 0);
        SetConfigValue<double>(config, deviceName, "VelocityPID", "D", motorConfig.velocityPID.D, 0);
        SetConfigValue<double>(config, deviceName, "VelocityPID", "FF", motorConfig.velocityPID.FF, 0);
        
        picojson::object velocities = config.get("Velocities").get<picojson::object>();
        for (auto& velocity : velocities)
        {
            motorConfig.velocities[velocity.first] = velocity.second.get<double>();
        }

        robotConfig.sparkMaxVelocityConfigs[config.get("Name").get<std::string>()] = motorConfig;
    }

    for (auto& config : sparkMaxPercentConfigs)
    {
        SparkMaxPercentConfig motorConfig;
        motorConfig.ID = (int)config.get("ID").get<double>();
        motorConfig.invertedRelative = config.get("InvertedRelative").get<bool>();
        motorConfig.currentLimit = config.get("CurrentLimit").get<double>();
        motorConfig.idleMode = config.get("IdleMode").get<std::string>() == "Brake" ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast;
        motorConfig.follow = config.get("Follow").get<std::string>();
        picojson::object percents = config.get("Percents").get<picojson::object>();

        for (auto& percent : percents)
        {
            motorConfig.percents[percent.first] = percent.second.get<double>();
        }

        robotConfig.sparkMaxPercentConfigs[config.get("Name").get<std::string>()] = motorConfig;
    }
}

template <typename T>
void ConfigFiles::SetConfigValue(picojson::value& config, std::string deviceName, std::string key, T& setValue, T defaultValue)
{
    if constexpr  (std::is_same<T, int>::value)
    {
        if (!config.contains(key)) {printNotContains(deviceName, key); setValue = defaultValue;}
        else setValue = (T)config.get(key).get<double>();
        
    }
    else
    {
        if (!config.contains(key)) {printNotContains(deviceName, key); setValue = defaultValue;}
        else setValue = config.get(key).get<T>();
    }
}

template <typename T>
void ConfigFiles::SetConfigValue(picojson::value& config, std::string deviceName, std::string key1, std::string key2, T& setValue, T defaultValue)
{
    if constexpr (std::is_same<T, int>::value)
    {
        if (!config.get(key1).contains(key2)) {printNotContains(deviceName, key2); setValue = defaultValue;}
        else setValue = (T)config.get(key1).get(key2).get<double>();
    }
    else
    {
        if (!config.get(key1).contains(key2)) {printNotContains(deviceName, key2); setValue = defaultValue;}
        else setValue = config.get(key2).get(key2).get<T>();
    }
}

void ConfigFiles::printNotContains(std::string deviceName, std::string missingComponent)
{
    std::cerr << "Device " << deviceName << " Does not contain a \"" << missingComponent << "\" in the Config File." << std::endl;
}