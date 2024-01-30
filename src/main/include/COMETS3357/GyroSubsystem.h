#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/SerialPort.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/SPI.h>
#include <COMETS3357/Subsystems/Subsystem.h>
#include <AHRS.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

namespace COMETS3357
{
    struct GyroData
    {
        double angle = 0;
        double angleRate = 0;
    };

    class GyroSubsystem : public COMETS3357::Subsystem
    {

    public:

        /**
         * @brief Constructs a Gyro Function
        */
        GyroSubsystem();

        /**
         * @brief The Periodic function for the GyroSubsystem
        */
        void Periodic() override;

        void ZeroGyro();

    private:
        AHRS m_navx;
        std::shared_ptr<nt::NetworkTable> driveTable;

        void CalculatePositions();

        float robotX, rototY; // experimental

        double angleOffset;

    };
};