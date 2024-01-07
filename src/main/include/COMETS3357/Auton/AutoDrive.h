#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/controller/RamseteController.h>

#include <COMETS3357/Subsystems/Subsystem.h>

#include <map>

namespace COMETS3357
{

    enum AutoDriveState
    {

    };

    class AutoDrive : public COMETS3357::Subsystem<AutoDriveState>
    {
    public:
        AutoDrive();

        void RunAutoDrivePath(std::string pathName);
        void AddAutoDrivePathPoint(std::string pathName, frc::Trajectory::State state);
        void GenerateAutoDrivePath(std::string pathName);

    private:

        std::map<std::string, std::vector<frc::Trajectory::State>&> autoDrivePaths;
    };
};