#pragma once

#include <iostream>
#include <fstream>
#include "COMETS3357/Json/picojson.h"
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <frc2/command/button/Trigger.h>
#include <filesystem>
#include <frc/fileSystem.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/MathUtil.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Command.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <COMETS3357/Commands/ChangeModeCommand.h>
#include <frc2/command/RunCommand.h>

namespace COMETS3357
{

    class Controller
    {
    public:

        enum JoystickCommandMode
        {
            JOYSTICK_DEFAULT_COMMAND,
            JOYSTICK_DEADZONE_COMMAND
        };

        void InitController(picojson::value &controllers);
        void InitController(std::string xboxDefaultMode, std::string taranusDefaultMode);

        /**
         * @brief Contructs a new controller
         * @param slot the slot of the controller
         * @param actions an event list of actions to pair to control bindings
         * @param controllerType the controller you want to control the robot with
        */
        Controller(int slot, std::unordered_map<std::string, std::shared_ptr<frc2::Command>> &buttonActions, std::unordered_map<std::string, std::tuple<std::function<void(double, double, double, double)>, frc2::Subsystem*, JoystickCommandMode>>& joystickActions, std::string controllerType);

        /**
         * @brief Loads the config of the controller
         * @param controllers The json value of the controller
        */
        void LoadConfig(picojson::value &controllers);

        /**
         * @brief Loads the controls from the json file
         * @return Did the controls load Successfully
        */
        bool LoadControls(picojson::value &controllers);

        int slot;
        frc2::CommandXboxController controller;
        std::map<std::string, frc2::Trigger> modeTriggers;
        std::string currentMode = "XBOXSemiAuto";
        std::unordered_map<std::string, std::shared_ptr<frc2::Command>> &buttonActionMap;
        std::unordered_map<std::string, std::tuple<std::function<void(double, double, double, double)>, frc2::Subsystem*, JoystickCommandMode>>& joystickActionMap;
        frc2::Trigger controllerConnectionTrigger;

        void SetButton(frc2::Trigger trigger, std::string button, std::pair<const std::string, picojson::value>& mode);
        void SetJoystickTrigger(frc2::Trigger trigger, std::string joystick, std::pair<const std::string, picojson::value>& mode, std::map<std::string, frc2::Trigger>& joystickTriggers);
        void SetJoysticks(std::map<std::string, frc2::Trigger>& joystickTriggers, std::pair<const std::string, picojson::value>& mode);

        std::string type;
    };


    class ControllerMap
    {
    public:


        /**
         * @brief Constructs a controllerMap object
         * This object is used to automatically bind controller buttons to actions
         * @param actionMap This is the action map that contains each action being paired
         * @param fileName This is the filename of the Controller Config file
         */
        ControllerMap(std::unordered_map<std::string, std::shared_ptr<frc2::Command>> &buttonActionMap, std::unordered_map<std::string, std::tuple<std::function<void(double, double, double, double)>, frc2::Subsystem*, Controller::JoystickCommandMode>>& joystickActionMap, std::string fileName);


        Controller primary;
        Controller secondary;
        Controller test;
        /**
         * Loads data out of the selected config file and loads the data into a 
         * RobotConfig object which is then distributed out to subsystems
         *
         * @param fileName The name of the config file that is being loaded
         */
        void LoadControllerMap(std::string fileName);

        /**
         * Returns a reference to the RobotConfig data while making sure that it is generated
         */


    private:

        

        
    };

};