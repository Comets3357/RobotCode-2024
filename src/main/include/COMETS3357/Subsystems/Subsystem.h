#pragma once

#include <frc2/Command/SubsystemBase.h>
#include <COMETS3357/Subsystems/SubsystemManager.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <string>

#define SUBSYSTEM_START(name) \
class name##Subsystem : public COMETS3357::Subsystem \
{ \
public: \
    void Initialize() override; \
    void Periodic() override; 

#define SUBSYSTEM_END };

namespace COMETS3357
{
    class Subsystem : public frc2::SubsystemBase
    {
    public:

        /**
         * @brief Constructs the Subsystem object
         * 
         * @param name The name of the subsystem
        */
        Subsystem(std::string_view name)
        {
            COMETS3357::SubsystemManager::GetInstance().AddInit([this]{Initialize();});
            SetName(name);
            subsystemData = nt::NetworkTableInstance::GetDefault().GetTable(name);
        }

        /**
         * @brief Gets the netork table for a different subsystem
         * 
         * @param name The name of the subsystem
        */
        std::shared_ptr<nt::NetworkTable> GetSubsystemData(std::string_view name)
        {
            return nt::NetworkTableInstance::GetDefault().GetTable(name);
        }

        /**
         * @brief An Initialization function for a Subsystem
        */
        virtual void Initialize() {}

        

    protected:

        std::shared_ptr<nt::NetworkTable> subsystemData;
    };
};