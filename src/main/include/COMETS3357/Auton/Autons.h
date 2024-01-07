#pragma once

#include "COMETS3357/Configs/ControllerMap.h"
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include "COMETS3357/Subsystems/Chassis/SwerveSubsystem.h"
#include <frc2/command/WaitCommand.h>
#include "COMETS3357/pathplanner/lib/commands/PPSwerveControllerCommand.h"
#include "COMETS3357/pathplanner/lib/auto/SwerveAutoBuilder.h"
#include "COMETS3357/pathplanner/lib/PathPlanner.h"
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <filesystem>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace COMETS3357
{

    class Autons
    {
    public:

        /**
     * Creates a new Autons object that generates and stores each auton located in
     * the auton directory. It is then able to run each auton on command.
     *
     * @param chassis A pointer to the Chassis subsystem that will be performing the auton
     */
        Autons(SwerveSubsystem* chassis, std::unordered_map<std::string, std::shared_ptr<frc2::Command>> &actionMap);

        /**
     * Runs a selected auton with the auton name
     *
     * @param AutonName The name of the auton file that will be ran
     */
        void RunAuton(std::string AutonName);

        /**
     * Goes through the auton directory and generates Commands for each auton
     */
        void LoadAutons();

        /**
         * Runs a selected auton with the auton name
         *
         * @param command A pointer to the command that will be ran when
         * the keyword is present in the marker located on path planner
         * @param keyword The keyword that cooresponds with the command specified
         */

        void AutonomousInit();

    private:

        std::map<std::string, std::unique_ptr<frc2::CommandPtr>> autons;
        COMETS3357::SwerveSubsystem* swerveSubsystem;
        frc::SendableChooser<std::string> autoChooser;
        pathplanner::SwerveAutoBuilder autoBuilder;

    };

};