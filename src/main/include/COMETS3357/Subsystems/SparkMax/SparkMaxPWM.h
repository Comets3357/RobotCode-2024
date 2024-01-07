#pragma once

#include <frc/motorcontrol/PWMSparkMax.h>
#include "COMETS3357/Configs/ConfigFiles.h"
#include "COMETS3357/Subsystems/SubsystemManager.h"

namespace COMETS3357
{

    struct SparkMaxPWMConfig;

    class SparkMaxPWM
    {
    public:

        SparkMaxPWM(std::string configName);

        void RobotInit();

        void SetPower(double power);

        frc::PWMSparkMax motor;

        COMETS3357::SparkMaxPWMConfig config;

    };
}