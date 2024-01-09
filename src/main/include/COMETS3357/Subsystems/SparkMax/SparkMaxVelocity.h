#pragma once

#include <rev/CANSparkMax.h>

#include "COMETS3357/PID.h"
#include "COMETS3357/Configs/ConfigFiles.h"

namespace COMETS3357
{
    class SparkMaxVelocity
    {
    public:

        /**
         * @brief Constructs a SparkMaxVelocity class
         * The SparkMaxVelocity class is for a motor purely controlled by either velocity or percent output
         * @param configName The name of the config file for the motor being created
        */
        SparkMaxVelocity(std::string configName);
        
        /**
         * @brief Sets the PID constants on the Velocity PID
         * @param velocityPID The PID values being used to set the velocity PID
        */
        void SetVelocityPID(PID velocityPID);

        /**
         * @brief Sets the target velocity of the motor
         * @param velocity The target velocity
        */
        void SetVelocity(double velocity);

        /**
         * @brief Sets the target velocity of the motor
         * @param velocity The target velocity
        */
        void SetVelocity(std::string velocity);

        /**
         * @brief Returns the velocity from the relative encoder
         * @return Relative Velocity
        */
        double GetRelativeVelocity();

        /**
         * @brief Sets the percent output on the motor
         * @param power A power ranging from -1 to 1 to power the motor
        */
        void SetPercent(double power);

        /**
         * @brief Returns the Velocity PID
         * @return Velocity PID
        */
        PID GetPID();

        /**
         * @brief Returns the Encoder Position from the relative encoder
         * @return Relative Position
        */
        double GetRelativePosition();

        /**
         * @brief Sets the encoder position of the relative encoder
         * @param position The value being set to the encoder position
        */
        void SetRelativePosition(double position);

        COMETS3357::SparkMaxVelocityConfig config;
        rev::CANSparkMax motor;

    private:

        /**
         * @brief The initialization for the motor. This is already ran in the constructor
        */
        void RobotInit();

        rev::SparkRelativeEncoder encoder;
        rev::SparkPIDController PIDController;
        COMETS3357::PID pid;

    };
};