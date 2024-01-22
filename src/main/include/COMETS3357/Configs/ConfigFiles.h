#pragma once

#include <iostream>
#include <fstream>
#include "COMETS3357/Json/picojson.h"
#include <string>
#include <vector>
#include <map>
#include <frc/fileSystem.h>
#include "COMETS3357/PID.h"
#include <rev/CANSparkMax.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include "COMETS3357/SparkMaxPositionRunMode.h"
#include <unordered_map>
#include <frc/Filesystem.h>
#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/PWMSparkMax.h>




namespace COMETS3357
{
    class SparkMaxPosition;
    class SparkMaxPercent;
    class SparkMaxVelocity;

    struct SparkMaxPositionConfig {
        COMETS3357::SparkMaxPosition* motor;
        std::string follow;
        int ID = 1;
        SparkMaxPositionRunMode defaultMode;
        bool invertedAbsolute;
        bool invertedRelative;
        double currentLimit;
        double relativePositionConversionFactor;
        double relativeVelocityConversionFactor;
        double absolutePositionConversionFactor;
        double absoluteVelocityConversionFactor;
        double absoluteZeroOffset;
        double maxSpeed;
        double minSpeed;
        rev::CANSparkMax::IdleMode idleMode;
        bool positionPIDWrappingEnabled;
        double turningEncoderPositionPIDMinInput;
        double turningEncoderPositionPIDMaxInput;
        PID velocityPID;
        PID positionPID;

        bool forwardSoftLimitEnabled;
        bool reverseSoftLimitEnabled;
        double forwardSoftLimit;
        double reverseSoftLimit;

        std::map<std::string, double> positions;
    };

    struct SparkMaxVelocityConfig {
        COMETS3357::SparkMaxVelocity* motor;
        std::string follow;
        int ID = 100;
        bool invertedRelative;
        double currentLimit;
        double relativePositionConversionFactor;
        double relativeVelocityConversionFactor;
        rev::CANSparkMax::IdleMode idleMode;
        PID velocityPID;

        std::map<std::string, double> velocities;
    };

    struct SparkMaxPWMConfig
    {
        frc::PWMSparkMax* motor;
        int ID = 100;
        bool inverted;
    };

    struct SparkMaxPercentConfig {
        COMETS3357::SparkMaxPercent* motor;
        std::string follow;
        int ID = 1;
        bool invertedRelative;
        double currentLimit;
        rev::CANSparkMax::IdleMode idleMode;

        std::map<std::string, double> percents;
    };

    struct SwerveModuleConfig
    {
        std::string azimuthConfigName;
        std::string driveConfigName;
        double angularOffset;
    };

    struct SwerveConfig
    {
        SwerveModuleConfig frontLeftModule{};
        SwerveModuleConfig frontRightModule{};
        SwerveModuleConfig backLeftModule{};
        SwerveModuleConfig backRightModule{};
        units::meter_t trackWidth{10};
        units::meter_t wheelBase{10};
        units::radians_per_second_t maxTurnSpeed{1};
        double directionSlewRate;
        units::meters_per_second_t maxSpeed{15};
        double magnitudeSlewRate;
        double rotationalSlewRate;
    };

    struct DrivebaseConfig
    {
        
    };

    struct RobotConfig {
        std::map<std::string, SparkMaxPositionConfig> sparkMaxPositionConfigs;
        std::map<std::string, SparkMaxVelocityConfig> sparkMaxVelocityConfigs;
        std::map<std::string, SparkMaxPercentConfig> sparkMaxPercentConfigs;
        std::map<std::string, SwerveConfig> swerveConfigs;
        std::map<std::string, SwerveModuleConfig> swerveModuleConfigs;
        std::map<std::string, SparkMaxPWMConfig> sparkMaxPWMConfigs;
    };


    class ConfigFiles
    {
    public:

        /**
         * Returns a static instance of the ConfigFiles which is used
         * to globally distribute the ConfigFile Data
         */
        static ConfigFiles& getInstance()
        {
            static ConfigFiles instance;
            return instance;
        }

        /**
         * Loads data out of the selected config file and loads the data into a 
         * RobotConfig object which is then distributed out to subsystems
         *
         * @param fileName The name of the config file that is being loaded
         */
        void LoadConfigFiles(std::string fileName);

        /**
         * Returns a reference to the RobotConfig data while making sure that it is generated
         */
        RobotConfig& GetConfigFiles();

        std::unordered_map<std::string, double> positions;

        void printNotContains(std::string deviceName, std::string missingComponent);

    private:

        bool initialized = false;
        RobotConfig robotConfig;

        ConfigFiles() {} // Private constructor to prevent instantiation outside of class
        ConfigFiles(const ConfigFiles&) = delete; // Disable copy constructor
        ConfigFiles& operator=(const ConfigFiles&) = delete; // Disable copy assignment operator
        ConfigFiles(ConfigFiles&&) = delete; // Disable move constructor
        ConfigFiles& operator=(ConfigFiles&&) = delete; // Disable move assignment operator
    };

};