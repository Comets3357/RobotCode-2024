#pragma once


#include <map>
#include <vector>
#include <functional>
#include <frc2/command/SubsystemBase.h>

namespace COMETS3357
{


    class SubsystemManager
    {
    public:

        /**
         * @brief Gets a static instance of the SubsystemManagerClass
        */
        static SubsystemManager& GetInstance()
        {
            static SubsystemManager instance;
            return instance;
        }

        /**
         * @brief Runs all of the init functions in the subsystems
        */
        void Init()
        {
            for (const auto& lamda : initFunctions)
            {
                lamda();
            }
            initFunctions.clear();
        }

        /**
         * @brief Adds an init function to the list of init functions
         * 
         * @param func The function adding to the list of init functions
        */
        void AddInit(std::function<void()> func)
        {
            initFunctions.emplace_back(func);
        }

    private:

        std::vector<std::function<void()>> initFunctions;

        SubsystemManager() {} // Private constructor to prevent instantiation outside of class
        SubsystemManager(const SubsystemManager&) = delete; // Disable copy constructor
        SubsystemManager& operator=(const SubsystemManager&) = delete; // Disable copy assignment operator
        SubsystemManager(SubsystemManager&&) = delete; // Disable move constructor
        SubsystemManager& operator=(SubsystemManager&&) = delete; // Disable move assignment operator

            
    };
};
