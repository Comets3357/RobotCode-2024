#include "COMETS3357/Auton/Autons.h"
#include "COMETS3357/pathplanner/lib/auto/NamedCommands.h"
#include "COMETS3357/pathplanner/lib/auto/AutoBuilder.h"
#include "COMETS3357/pathplanner/lib/util/PathPlannerLogging.h"
#include <vector>
#include <COMETS3357/pathplanner/lib/commands/PathPlannerAuto.h>
#include <COMETS3357/Json/picojson.h>
#include <frc2/command/InstantCommand.h>
#include <COMETS3357/pathplanner/lib/auto/CommandUtil.h>
#include <frc2/command/ParallelRaceGroup.h>
#include "COMETS3357/Auton/AdvancedAutoPathCommand.h"

using namespace COMETS3357;

Autons::Autons(SwerveSubsystem* drivebase, std::vector<std::pair<std::string, std::shared_ptr<frc2::Command>>> &actionMap) : swerveSubsystem{drivebase}
{
    LoadAutons(actionMap);
}

void Autons::Cancel()
{
    std::string autonName = autoChooser.GetSelected();

    if (autons.contains(autonName))
    {
        autons[autonName].get()->Cancel();
    }
}

void Autons::RunAuton(std::string autonName)
{
    if (autons.contains(autonName))
    {
        autons[autonName].get()->Schedule();
    }
}

void Autons::AutonomousInit()
{
    RunAuton(autoChooser.GetSelected());
}

void Autons::LoadAutons( std::vector<std::pair<std::string, std::shared_ptr<frc2::Command>>>  &actionMap)
{
    std::string const filePath = frc::filesystem::GetDeployDirectory() + "/autos/";

    for (const auto& entry : std::filesystem::directory_iterator(filePath))
    {
        if (entry.is_regular_file())
        {
            std::string autonName = entry.path().stem().string();
            autoChooser.AddOption(autonName, autonName);

            std::ifstream ifs(filePath + autonName + ".json");

            std::string json_str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
            if (json_str.length() != 0)
            {
                // create a picojson::value object to store the parsed JSON data
                picojson::value v;

                // parse the JSON string and check for errors
                std::string err = picojson::parse(v, json_str);
                if (!err.empty()) {
                    std::cerr << "JSON parsing error: " << err << std::endl;

                }

                // get the JSON array from the value object
                picojson::array arr = v.get<picojson::array>();

                // autons[autonName].first = std::make_unique<frc2::SequentialCommandGroup>();
                // std::vector<std::unique_ptr<frc2::Command>> commands = {};
                // autons[autonName].first = std::make_unique<frc2::SequentialCommandGroup>();
                std::vector<frc2::CommandPtr> commands;
                bool start = false;

                frc::Pose2d startPosition;

                for (auto& elem : arr) {
                    // check the type of the element
                    if (elem.is<picojson::object>()) {
                        // if the element is an object, get the pose values
                        picojson::object pose = elem.get<picojson::object>();
                        double x = pose["x"].get<double>();
                        double y = pose["y"].get<double>();
                        double rotation = pose["rotation"].get<double>();
                        double movementSpeed = pose["maxVelocity"].get<double>();
                        // frc::SmartDashboard::PutNumber("movementSpeed From Json" + autonName, movementSpeed);
                        double rotationSpeed = pose["maxRotation"].get<double>();
                        int isAdvanced = (int)pose["avoid"].get<double>();
                        int isTurnSpeaker = (int)pose["TurnSpeaker"].get<double>();
                        double endSpeed = pose["EndVelocity"].get<double>();

                        if (start)
                        {
            
                            // autons[autonName].first->AddCommands(AutonPathCommand{swerveSubsystem, 0.5, 0.5, frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}}, frc::Rotation2d{units::radian_t{rotation * 3.14159 / 180.0}}}});
                            // autons[autonName].first->AddCommands(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}});
                            if (isAdvanced)
                            {
                                commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<AdvancedAutonPathCommand>(swerveSubsystem, rotationSpeed, movementSpeed, frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}}, frc::Rotation2d{units::radian_t{rotation * 3.14159 / 180.0}}})));
                            }
                            else
                            {
                                commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<AutonPathCommand>(swerveSubsystem, rotationSpeed, movementSpeed, frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}}, frc::Rotation2d{units::radian_t{rotation * 3.14159 / 180.0}}}, false, isTurnSpeaker, endSpeed)));
                            }
                            
                            commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                                    commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));

                            commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));

                            commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));

                            commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));

                        
                            commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));

                            commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));

                            commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));

                        
                        }
                        else
                        {
                            start = true;
                            startPosition = frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}}, frc::Rotation2d{units::radian_t{rotation * 3.14159 / 180.0}}};
                            // commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[startPosition, this](){swerveSubsystem->ResetOdometry(startPosition);}, {swerveSubsystem}})));
                        }
                        // do something with the pose values, such as moving the robot to that position
                    } else if (elem.is<std::string>()) {
                        // if the element is a string, get the command name
                        std::string command = elem.get<std::string>();
                        
                        // do something with the command name, such as calling a function with that name
                        if (command.substr(0,4) == "wait")
                        {
                            std::stringstream ss(command);
                            std::string value;
                            ss >> value; ss >> value;
                            commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::WaitCommand>(units::second_t{stod(value)})));
                        }
                        else
                        {
                            for (auto& c : actionMap)
                            {
                                if (command == c.first)
                                {
                                    commands.push_back(pathplanner::CommandUtil::wrappedEventCommand(c.second));
                                    // autons[autonName].first->AddCommands(c.second.get());
                                }
                            }
                        }
                    } 
                    else if (elem.is<picojson::array>()) 
                    {
                        std::vector<frc2::CommandPtr> commands2;
                        for (auto& elem2 : elem.get<picojson::array>())
                        {
                            
                            if (elem2.is<picojson::object>()) 
                            {
                                // if the element is an object, get the pose values
                                picojson::object pose = elem2.get<picojson::object>();
                                double x = pose["x"].get<double>();
                                double y = pose["y"].get<double>();
                                double rotation = pose["rotation"].get<double>();
                                double movementSpeed = pose["maxVelocity"].get<double>();
                                frc::SmartDashboard::PutNumber("movementSpeed From Json", movementSpeed);
                                double rotationSpeed = pose["maxRotation"].get<double>();
                                int isAdvanced = (int)pose["avoid"].get<double>();
                                int isTurnSpeaker = (int)pose["TurnSpeaker"].get<double>();
                                double endSpeed = pose["EndVelocity"].get<double>();

                                if (start)
                                {
                                
                                    std::vector<frc2::CommandPtr> commands3;
                                
                                    if (isAdvanced)
                                    {
                                        commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<AdvancedAutonPathCommand>(swerveSubsystem, rotationSpeed, movementSpeed, frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}}, frc::Rotation2d{units::radian_t{rotation * 3.14159 / 180.0}}})));
                                    }
                                    else
                                    {
                                        commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<AutonPathCommand>(swerveSubsystem, rotationSpeed, movementSpeed, frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}}, frc::Rotation2d{units::radian_t{rotation * 3.14159 / 180.0}}}, false, isTurnSpeaker, endSpeed)));
                                    }                                   
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands3.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[this](){swerveSubsystem->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerveSubsystem->kDriveKinematics);}, {swerveSubsystem}})));
                                    commands2.push_back(frc2::cmd::Sequence(std::move(commands3)));
                                }
                                else
                                {
                                    start = true;
                                    startPosition = frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}}, frc::Rotation2d{units::radian_t{rotation * 3.14159 / 180.0}}};
                                    // commands2.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::InstantCommand>(frc2::InstantCommand{[startPosition, this](){swerveSubsystem->ResetOdometry(startPosition);}, {swerveSubsystem}})));
                                }
                                // do something with the pose values, such as moving the robot to that position
                            } else if (elem2.is<std::string>()) {
                                // if the element is a string, get the command name
                                std::string command2 = elem2.get<std::string>();
                                
                                // do something with the command name, such as calling a function with that name
                                if (command2.substr(0,4) == "wait")
                                {
                                    std::stringstream ss(command2);
                                    std::string value;
                                    ss >> value; ss >> value;
                                    commands2.push_back(pathplanner::CommandUtil::wrappedEventCommand(std::make_shared<frc2::WaitCommand>(units::second_t{stod(value)})));
                                }
                                else
                                {
                                    for (auto& c : actionMap)
                                    {
                                        if (command2 == c.first)
                                        {
                                            commands2.push_back(pathplanner::CommandUtil::wrappedEventCommand(c.second));
                                            // autons[autonName].first->AddCommands(c.second.get());
                                        }
                                    }
                                }
                            }
                        }
                        commands.push_back(frc2::cmd::Race(std::move(commands2)));
                    }
                }
                autons[autonName] = std::make_unique<frc2::CommandPtr>(frc2::cmd::Sequence(std::move(commands)));
            }
        }
    }

    autoChooser.AddOption("Potato", "Potato");

    frc::SmartDashboard::PutData("Autonomous Mode", &autoChooser);

}

