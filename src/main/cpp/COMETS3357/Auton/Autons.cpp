#include "COMETS3357/Auton/Autons.h"
#include "COMETS3357/pathplanner/lib/auto/NamedCommands.h"
#include "COMETS3357/pathplanner/lib/auto/AutoBuilder.h"
#include "COMETS3357/pathplanner/lib/util/PathPlannerLogging.h"
#include <vector>

using namespace COMETS3357;

Autons::Autons(SwerveSubsystem* drivebase, std::vector<std::pair<std::string, std::shared_ptr<frc2::Command>>> &actionMap) : swerveSubsystem{drivebase}
{
    LoadAutons();

    pathplanner::HolonomicPathFollowerConfig pathFollowerConfig = pathplanner::HolonomicPathFollowerConfig(
        pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation constants 
        pathplanner::PIDConstants(5.0, 0.0, 0.0), // Rotation constants 
        5_mps,
        0.57_m, // Drive base radius (distance from center to furthest module) 
        pathplanner::ReplanningConfig()
    );

    pathplanner::AutoBuilder::configureHolonomic(
        [this]() {return swerveSubsystem->GetPose();},
        [this](frc::Pose2d pose){swerveSubsystem->ResetOdometry(pose);},
        [this]() {return swerveSubsystem->getSpeeds();},
        [this](frc::ChassisSpeeds robotSpeed){swerveSubsystem->SetChassisSpeed(robotSpeed);},
        pathFollowerConfig,
        [this](){return true;},
        swerveSubsystem
    );

    for (int i = 0; i < actionMap.size(); i++)
    {
        pathplanner::NamedCommands::registerCommand(actionMap[i].first, actionMap[i].second);
    }
}

void Autons::RunAuton(std::string autonName)
{
    // if (autons.contains(autonName))
    // autons[autonName]->Schedule();
}

void Autons::AutonomousInit()
{
    // RunAuton(autoChooser.GetSelected());
}

void Autons::LoadAutons()
{

    // std::string const filePath = frc::filesystem::GetDeployDirectory() + "/pathplanner/";
    
    // for (const auto& entry : std::filesystem::directory_iterator(filePath))
    // {
    //     if (entry.is_regular_file())
    //     {
    //         std::string autonName = entry.path().stem().string();
    //         if (autonName != "navgrid")
    //         {
    //                autoChooser.AddOption(autonName, autonName);
    //         // std::vector<pathplanner::PathPlannerTrajectory> pathGroup = pathplanner::PathPlanner::loadPathGroup(autonName, {pathplanner::PathConstraints{5_mps, 3.5_mps_sq}});


    //         autons[autonName] = std::make_unique<frc2::CommandPtr>(pathplanner::AutoBuilder::buildAuto(autonName));

    //         // autons[autonName] = std::make_unique<frc2::CommandPtr>(autoBuilder.fullAuto(pathGroup));
    //         }
         
    //     }
    // }

    // autoChooser.AddOption("Potato", "Potato");

    // frc::SmartDashboard::PutData("Autonomous Mode", &autoChooser);

}

