#include "COMETS3357/Auton/Autons.h"

using namespace COMETS3357;

Autons::Autons(SwerveSubsystem* drivebase, std::unordered_map<std::string, std::shared_ptr<frc2::Command>> &actionMap) : swerveSubsystem{drivebase}, autoBuilder(
        [this]() {return swerveSubsystem->GetPose();},
        [this](auto initPose) {swerveSubsystem->ResetOdometry(initPose);},
        swerveSubsystem->kDriveKinematics,
        pathplanner::PIDConstants(5.0, 0.0, 0.0),
        pathplanner::PIDConstants(2, 0.0, 0.0),
        [this](auto speeds) {swerveSubsystem->SetModuleStates(speeds);},
        actionMap,
        {swerveSubsystem},
        true
    )
{
    LoadAutons();
}

void Autons::RunAuton(std::string autonName)
{
    if (autons.contains(autonName))
    autons[autonName]->Schedule();
}

void Autons::AutonomousInit()
{
    RunAuton(autoChooser.GetSelected());
}

void Autons::LoadAutons()
{

    std::string const filePath = frc::filesystem::GetDeployDirectory() + "/pathplanner/";
    
    for (const auto& entry : std::filesystem::directory_iterator(filePath))
    {
        if (entry.is_regular_file())
        {
            std::string autonName = entry.path().stem().string();
            autoChooser.AddOption(autonName, autonName);
            std::vector<pathplanner::PathPlannerTrajectory> pathGroup = pathplanner::PathPlanner::loadPathGroup(autonName, {pathplanner::PathConstraints{5_mps, 3.5_mps_sq}});

            autons[autonName] = std::make_unique<frc2::CommandPtr>(autoBuilder.fullAuto(pathGroup));
        }
    }

    autoChooser.AddOption("Potato", "Potato");

    frc::SmartDashboard::PutData("Autonomous Mode", &autoChooser);

}

