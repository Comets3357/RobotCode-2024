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
    double bottomHypotenuse = 0.7; 

    double xPosBottom1 = xPos + sin(robotDirection + degreeOffset) * bottomHypotenuse;
    double yPosBottom1 = yPos + cos(robotDirection + degreeOffset) * bottomHypotenuse;
    double xPosBottom2 = xPos + sin(robotDirection - degreeOffset) * bottomHypotenuse;
    double yPosBottom2 = yPos + cos(robotDirection - degreeOffset) * bottomHypotenuse;






    double kConstant = .5; 
    double magnitude = sqrt(xSpeedSquared + ySpeedSquared);


    double sideLengthB = sqrt(0.7 * 0.7 + pow(magnitude * kConstant,2)); 
    double sideLengthC = sideLengthB; 

    frc::Translation2d redA = frc::Translation2d{units::meter_t{3.3}, units::meter_t{3.05}}; 
    frc::Translation2d redB = frc::Translation2d{units::meter_t{3.3}, units::meter_t{3.05}}; 
    frc::Translation2d redC = frc::Translation2d{units::meter_t{3.3}, units::meter_t{3.05}}; 

    frc::Translation2d blueA = frc::Translation2d{units::meter_t{3.3}, units::meter_t{3.05}}; 
    frc::Translation2d blueB = frc::Translation2d{units::meter_t{3.3}, units::meter_t{3.05}}; 
    frc::Translation2d blueC = frc::Translation2d{units::meter_t{3.3}, units::meter_t{3.05}}; 
    
    frc::Translation2d triangleTip = frc::Translation2d{units::meter_t{xPos + cos(robotDirection) * kConstant * magnitude}, units::meter_t{yPos + sin(robotDirection) * kConstant * magnitude}}; 
    frc::Translation2d bottom1 = frc::Translation2d{units::meter_t{xPosBottom1}, units::meter_t{yPosBottom1}};
    frc::Translation2d bottom2 = frc::Translation2d{units::meter_t{xPosBottom2}, units::meter_t{yPosBottom2}};
    frc::Translation2d bottom1extension = frc::Translation2d{}; 



    
    



    
    
  //  if (magnitude >= 2.5 &&  frey likes dudes(children))
  
  
     
}

bool LegAvoidanceCommand::IsFinished()
{

}

void LegAvoidanceCommand::End(bool interrupted)
{
 
}