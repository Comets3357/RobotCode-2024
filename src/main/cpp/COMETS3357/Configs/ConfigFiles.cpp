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
    picojson::array lookupTableConfigs = jsonValue.get("LookupTableConfigs").get<picojson::array>();

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
        motorConfig.ID = (int)config.get("ID").get<double>();
        motorConfig.defaultMode = config.get("DefaultRunMode").get<std::string>()=="ABSOLUTE" ? SparkMaxPositionRunMode::POSITION_SPARK_MAX_ABSOLUTE : SparkMaxPositionRunMode::POSITION_SPARK_MAX_RELATIVE;
        motorConfig.invertedAbsolute = config.get("InvertedAbsolute").get<bool>();
        motorConfig.invertedRelative = config.get("InvertedRelative").get<bool>();
        motorConfig.currentLimit = config.get("CurrentLimit").get<double>();
        motorConfig.relativePositionConversionFactor = config.get("RelativePositionConversionFactor").get<double>();
        motorConfig.relativeVelocityConversionFactor = config.get("RelativeVelocityConversionFactor").get<double>();
        motorConfig.absolutePositionConversionFactor = config.get("AbsolutePositionConversionFactor").get<double>();
        motorConfig.absoluteVelocityConversionFactor = config.get("AbsoluteVelocityConversionFactor").get<double>();
        motorConfig.absoluteZeroOffset = config.get("AbsoluteZeroOffset").get<double>();
        motorConfig.maxSpeed = config.get("MaxSpeed").get<double>();
        motorConfig.minSpeed = config.get("MinSpeed").get<double>();
        motorConfig.idleMode = config.get("IdleMode").get<std::string>() == "Brake" ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast;
        motorConfig.follow = config.get("Follow").get<std::string>();

        motorConfig.velocityPID.P = config.get("VelocityPID").get("P").get<double>();
        motorConfig.velocityPID.I = config.get("VelocityPID").get("I").get<double>();
        motorConfig.velocityPID.D = config.get("VelocityPID").get("D").get<double>();
        motorConfig.velocityPID.FF = config.get("VelocityPID").get("FF").get<double>();

        motorConfig.positionPID.P = config.get("PositionPID").get("P").get<double>();
        motorConfig.positionPID.I = config.get("PositionPID").get("I").get<double>();
        motorConfig.positionPID.D = config.get("PositionPID").get("D").get<double>();
        motorConfig.positionPID.FF = config.get("PositionPID").get("FF").get<double>();

        motorConfig.positionPIDWrappingEnabled = config.get("PositionPIDWrappingEnabled").get<bool>();
        motorConfig.turningEncoderPositionPIDMinInput = config.get("TurningEncoderPositionPIDMinInput").get<double>();
        motorConfig.turningEncoderPositionPIDMaxInput = config.get("TurningEncoderPositionPIDMaxInput").get<double>();

        motorConfig.forwardSoftLimitEnabled = config.get("ForwardSoftLimitEnabled").get<bool>();
        motorConfig.reverseSoftLimitEnabled = config.get("ReverseSoftLimitEnabled").get<bool>();
        motorConfig.forwardSoftLimit = config.get("ForwardSoftLimit").get<double>();
        motorConfig.reverseSoftLimit = config.get("ReverseSoftLimit").get<double>();

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
        motorConfig.ID = (int)config.get("ID").get<double>();
        motorConfig.invertedRelative = config.get("InvertedRelative").get<bool>();
        motorConfig.currentLimit = config.get("CurrentLimit").get<double>();
        motorConfig.relativePositionConversionFactor = config.get("RelativePositionConversionFactor").get<double>();
        motorConfig.relativeVelocityConversionFactor = config.get("RelativeVelocityConversionFactor").get<double>();
        motorConfig.idleMode = config.get("IdleMode").get<std::string>() == "Brake" ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast;

        motorConfig.velocityPID.P = config.get("VelocityPID").get("P").get<double>();
        motorConfig.velocityPID.I = config.get("VelocityPID").get("I").get<double>();
        motorConfig.velocityPID.D = config.get("VelocityPID").get("D").get<double>();
        motorConfig.velocityPID.FF = config.get("VelocityPID").get("FF").get<double>();

        motorConfig.follow = config.get("Follow").get<std::string>();

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

    for (auto& config : lookupTableConfigs)
    {
        LookupTableConfig lookupTableConfig;
        picojson::array dataArray = config.get("Data").get<picojson::array>();
        for (auto& point : dataArray)
        {
            picojson::array pairData = point.get<picojson::array>();
            std::pair<double, double> pairPoint = {pairData.at(0).get<double>(), pairData.at(1).get<double>()};
            lookupTableConfig.lookupTableData.emplace(pairPoint);
        }
        robotConfig.lookupTableConfigs[config.get("Name").get<std::string>()] = lookupTableConfig;
    }
}