#include "COMETS3357/Configs/ControllerMap.h"

using namespace COMETS3357;

ControllerMap::ControllerMap(std::unordered_map<std::string, std::shared_ptr<frc2::Command>> &buttonActionMap, std::unordered_map<std::string, std::tuple<std::function<void(double, double, double, double)>, frc2::Subsystem*, Controller::JoystickCommandMode>>& joystickActionMap, std::string fileName) : primary{0, buttonActionMap, joystickActionMap, "XBOX"}, secondary{1, buttonActionMap, joystickActionMap, "XBOX"}, test{2, buttonActionMap, joystickActionMap, "XBOX"}
{
    LoadControllerMap(fileName);
}

void ControllerMap::LoadControllerMap(std::string fileName)
    {
        
        std::ifstream jsonFile(frc::filesystem::GetDeployDirectory() + "/ControllerMaps/" + fileName + ".json");
        if (!jsonFile.is_open()) 
        {
            std::cerr << "Failed to open file" << std::endl;
        }

        std::string jsonString((std::istreambuf_iterator<char>(jsonFile)), std::istreambuf_iterator<char>());

        picojson::value jsonValue;
        std::string error = picojson::parse(jsonValue, jsonString);
        if (!error.empty()) 
        {
            std::cerr << "JSON parse error: " << error << std::endl;
        }

        if (jsonValue.contains("PrimaryController"))
        {
            primary.LoadConfig(jsonValue.get("PrimaryController"));
        }
        if (jsonValue.contains("SecondaryController"))
        {
            secondary.LoadConfig(jsonValue.get("SecondaryController"));
        }
        if (jsonValue.contains("TestController"))
        {
            test.LoadConfig(jsonValue.get("TestController"));
        }

    }