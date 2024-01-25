#include "Commands/LegAvoidanceCommand.h"



LegAvoidanceCommand::LegAvoidanceCommand(COMETS3357::SwerveSubsystem *swerve) {

    frc::Pose2d pos = swerveSubsystem->GetPose();
    swerveSubsystem = swerve;
    AddRequirements(swerve);

    pos.X();
    pos.Y();
    pos.Rotation().Radians();  

    
    
}

void LegAvoidanceCommand::Initialize()
{

}

void LegAvoidanceCommand::Execute()
{
    double xSpeed = (double)swerveSubsystem->getSpeeds().vx;
    double ySpeed = (double)swerveSubsystem->getSpeeds().vy;
    double xSpeedSquared = pow(xSpeed, 2); 
    double ySpeedSquared = pow(ySpeed, 2); 

    double xPos = (double)swerveSubsystem->GetPose().X();
    double yPos = (double)swerveSubsystem->GetPose().Y();
    double robotDirection = (double)swerveSubsystem->GetPose().Rotation().Radians();

    double degreeOffset = 1.0;
    double bottomHypotenuse = 27.4;






    double kConstant = 0.2; 
    double magnitude = sqrt(xSpeedSquared + ySpeedSquared);

    
    frc::Translation2d triangleTip = frc::Translation2d{units::meter_t{xPos + cos(robotDirection) * kConstant * magnitude}, units::meter_t{yPos + sin(robotDirection) * kConstant * magnitude}}; 
    frc::Translation2d bottom1 = frc::Translation2d{units::meter_t{xPos + sin(robotDirection + degreeOffset) * bottomHypotenuse}, units::meter_t{yPos + cos(robotDirection + degreeOffset) * bottomHypotenuse}};
    frc::Translation2d bottom2 = frc::Translation2d{units::meter_t{xPos + sin(robotDirection - degreeOffset) * bottomHypotenuse}, units::meter_t{yPos + cos(robotDirection - degreeOffset) * bottomHypotenuse}};


    
    
  //  if (magnitude >= 2.5 &&  frey likes dudes(children))
  
  
     
}

bool LegAvoidanceCommand::IsFinished()
{

}

void LegAvoidanceCommand::End(bool interrupted)
{
 
}