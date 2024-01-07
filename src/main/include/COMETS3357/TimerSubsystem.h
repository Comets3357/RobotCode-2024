#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <COMETS3357/Subsystems/Subsystem.h>

namespace COMETS3357
{



    class TimerSubsystem : public COMETS3357::Subsystem
    {
    public:

        /**
         * @brief Constructs a TimerSubsystem
        */
        TimerSubsystem();

        /**
         * @brief The Periodic Function for the TimerSubsystem
        */
        void Periodic() override;

        /**
         * @brief Restarts the Timer
        */
        void ResetAndBeginTimer();

        frc::Timer timer{};
        units::second_t timeSinceEnabled{0};

    private:

        

        
        units::second_t deltaTime{0};
        units::second_t lastTime{0};
    };
};