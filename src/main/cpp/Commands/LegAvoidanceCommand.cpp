#include "Commands/LegAvoidanceCommand.h"



LegAvoidanceCommand::LegAvoidanceCommand(COMETS3357::SwerveSubsystem *swerve) {

    frc::Pose2d pos = swerveSubsystem->GetPose();
    swerveSubsystem = swerve;

    pos.X();
    pos.Y();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
    
}

void LegAvoidanceCommand::Initialize()
{

}

void LegAvoidanceCommand::Execute()
{

}

bool LegAvoidanceCommand::IsFinished()
{

}

void LegAvoidanceCommand::End(bool interrupted)
{
 
}