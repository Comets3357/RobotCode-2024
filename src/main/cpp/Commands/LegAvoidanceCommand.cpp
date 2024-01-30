#include "Commands/LegAvoidanceCommand.h"



LegAvoidanceCommand::LegAvoidanceCommand(COMETS3357::SwerveSubsystem *swerve) {

    frc::Pose2d pos = swerveSubsystem->GetPose();
    swerveSubsystem = swerve;
    // AddRequirements(swerve);

    pos.X();
    pos.Y();
    pos.Rotation().Radians();  

    
    
}

void LegAvoidanceCommand::Initialize()
{

}

double HeronsFormula (double semiperimeter, double a, double b, double c) {
  return sqrt(semiperimeter * (semiperimeter - a) * (semiperimeter - b) * (semiperimeter - c)); 
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


    double xPosBottom1 = xPos + sin(robotDirection + degreeOffset) * bottomHypotenuse;
    double yPosBottom1 = yPos + cos(robotDirection + degreeOffset) * bottomHypotenuse;
    double xPosBottom2 = xPos + sin(robotDirection - degreeOffset) * bottomHypotenuse;
    double yPosBottom2 = yPos + cos(robotDirection - degreeOffset) * bottomHypotenuse;

    double kConstant = 0.2; 
    double magnitude = sqrt(xSpeedSquared + ySpeedSquared);
  
    
    frc::Translation2d triangleTip = frc::Translation2d{units::meter_t{xPos + cos(robotDirection) * kConstant * magnitude}, units::meter_t{yPos + sin(robotDirection) * kConstant * magnitude}}; 
    frc::Translation2d bottom1 = frc::Translation2d{units::meter_t{xPosBottom1}, units::meter_t{yPosBottom1}};
    frc::Translation2d bottom2 = frc::Translation2d{units::meter_t{xPosBottom2}, units::meter_t{yPosBottom2}};

    //frc::Translation2d const points [6]; 
    
    // frc::Translation2d const redA =  frc::Translation2d{units::meter_t{13.25}, units::meter_t{4.1}}; 
    // frc::Translation2d const redB =  frc::Translation2d{units::meter_t{10.93}, units::meter_t{5.41}};
    // frc::Translation2d const redC = frc::Translation2d{units::meter_t{10.93}, units::meter_t{2.8}};
    // frc::Translation2d const blueA = frc::Translation2d{units::meter_t{3.3}, units::meter_t{4.1}};
    // frc::Translation2d const blueB = frc::Translation2d{units::meter_t{5.63}, units::meter_t{5.41}};
    // frc::Translation2d const blueC = frc::Translation2d{units::meter_t{5.63}, units::meter_t{2.80}};
    frc::Translation2d const testPoint = frc::Translation2d{units::meter_t{8.4}, units::meter_t{7}}; 
    frc::Translation2d const points [] = {testPoint}; 

    //frc::Translation2d const points[] = {redA, redB, redC, blueA, blueB, blueC}; 

    double sideLengthA = (double)bottom1.Distance(bottom2); 
    double sideLengthB = (double)bottom1.Distance(triangleTip); 
    double sideLengthC = (double)bottom2.Distance(triangleTip); 

    bool isClear; 
    bool starboardSide; 

    for(int i = 0; i < sizeof(points) / sizeof(frc::Translation2d); i++) {
      double distanceTip = (double)points[i].Distance(triangleTip);  // distane from the point to the tip of the triangle
      double distance1 = (double)points[i].Distance(bottom1);     // 
      double distance2 = (double)points[i].Distance(bottom2);     // 


      double semiperimeterTriangle = (sideLengthA + sideLengthB + sideLengthC) / 2.0; 
      double semiperimeterAlpha = (distance1 + distance2 + sideLengthA) / 2.0; 
      double semiperimeterBeta = (distance1 + distanceTip + sideLengthB) / 2.0;
      double semiperimeterGamma = (distanceTip + distance2 + sideLengthC) / 2.0; 

      double bigTriArea = HeronsFormula(semiperimeterTriangle, sideLengthA, sideLengthB, sideLengthC); 
      double areaA = HeronsFormula(semiperimeterAlpha, distance1, distance2, sideLengthA); 
      double areaB = HeronsFormula(semiperimeterBeta, distance1, distanceTip, sideLengthB); 
      double areaC = HeronsFormula(semiperimeterGamma, distanceTip, distance2, sideLengthC); 

      double alpha, beta, gamma; 
      alpha = areaA / bigTriArea; 
      beta = areaB / bigTriArea; 
      gamma = areaC / bigTriArea; 

      if (alpha + beta + gamma != 1) {
          isClear = false; 
          if (areaC > areaB) {
            starboardSide = false; 
          }
      }

    }
    
    
  
  frc::SmartDashboard::PutBoolean("Is not in Triangle", isClear); 

  swerveSubsystem->controllingSwerveMovement = true;
  swerveSubsystem->overrideVelocityX = units::meters_per_second_t{0};
  swerveSubsystem->overrideVelocityY = units::meters_per_second_t{0};
  
     
}

bool LegAvoidanceCommand::IsFinished()
{

}

void LegAvoidanceCommand::End(bool interrupted)
{
 
}