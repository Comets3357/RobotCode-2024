#pragma once

#include <frc2/command/ProfiledPIDCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>

class ChangeModeCommand : public frc2::CommandHelper<frc2::CommandBase, ChangeModeCommand>
{
public:

    ChangeModeCommand(std::string& mode, std::string changeTo)
    {
        mode = changeTo;
    }

    bool End()
    {
        return true;
    }


};