#include "Commands/LegAvoidanceCommand.h"



LegAvoidanceCommand::LegAvoidanceCommand(COMETS3357::SwerveSubsystem *swerve) {

swerveSubsystem = swerve;
    frc::Pose2d pos = swerveSubsystem->GetPose();
    
    // AddRequirements(swerve);

    pos.X();
    pos.Y();
    pos.Rotation().Radians();  

    
    
}

void LegAvoidanceCommand::Initialize()
{

}

// double HeronsFormula (double semiperimeter, double a, double b, double c) {
//   return sqrt(semiperimeter * (semiperimeter - a) * (semiperimeter - b) * (semiperimeter - c)); 
// }

float area(double x1, double y1, double x2, double y2, double x3, double y3) {
  return std::abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
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
  
    
    //frc::Translation2d triangleTip = frc::Translation2d{units::meter_t{xPos + cos(robotDirection) * kConstant * magnitude}, units::meter_t{yPos + sin(robotDirection) * kConstant * magnitude}}; 
    //frc::Translation2d bottom1 = frc::Translation2d{units::meter_t{xPosBottom1}, units::meter_t{yPosBottom1}};
    //frc::Translation2d bottom2 = frc::Translation2d{units::meter_t{xPosBottom2}, units::meter_t{yPosBottom2}};

    frc::Translation2d triangleTip = frc::Translation2d{units::meter_t{9.68}, units::meter_t{6.88}}; 
    frc::Translation2d bottom1 = frc::Translation2d{units::meter_t{11.30}, units::meter_t{7.68}};
    frc::Translation2d bottom2 = frc::Translation2d{units::meter_t{11.38}, units::meter_t{6.33}};
    //frc::Translation2d const points [6]; 
    
    // frc::Translation2d const redA =  frc::Translation2d{units::meter_t{13.25}, units::meter_t{4.1}}; 
    // frc::Translation2d const redB =  frc::Translation2d{units::meter_t{10.93}, units::meter_t{5.41}};
    // frc::Translation2d const redC = frc::Translation2d{units::meter_t{10.93}, units::meter_t{2.8}};
    // frc::Translation2d const blueA = frc::Translation2d{units::meter_t{3.3}, units::meter_t{4.1}};
    // frc::Translation2d const blueB = frc::Translation2d{units::meter_t{5.63}, units::meter_t{5.41}};
    // frc::Translation2d const blueC = frc::Translation2d{units::meter_t{5.63}, units::meter_t{2.80}};
    frc::Translation2d const testPoint = frc::Translation2d{units::meter_t{13.75}, units::meter_t{6.8}}; 
    frc::Translation2d const points [] = {testPoint}; 

    //frc::Translation2d const points[] = {redA, redB, redC, blueA, blueB, blueC}; 

    double sideLengthA = (double)bottom1.Distance(bottom2); 
    double sideLengthB = (double)bottom1.Distance(triangleTip); 
    double sideLengthC = (double)bottom2.Distance(triangleTip); 

    bool isClear; 
    bool starboardSide; 

    for(int i = 0; i < sizeof(points) / sizeof(frc::Translation2d); i++) {
      // double distanceTip = (double)points[i].Distance(triangleTip);  // distane from the point to the tip of the triangle
      // double distance1 = (double)points[i].Distance(bottom1);     // 
      // double distance2 = (double)points[i].Distance(bottom2);     // 


      // double semiperimeterTriangle = (sideLengthA + sideLengthB + sideLengthC) / 2.0; 
      // double semiperimeterAlpha = (distance1 + distance2 + sideLengthA) / 2.0; 
      // double semiperimeterBeta = (distance1 + distanceTip + sideLengthB) / 2.0;
      // double semiperimeterGamma = (distanceTip + distance2 + sideLengthC) / 2.0; 

      // double bigTriArea = HeronsFormula(semiperimeterTriangle, sideLengthA, sideLengthB, sideLengthC); 
      // double areaA = HeronsFormula(semiperimeterAlpha, distance1, distance2, sideLengthA); 
      // double areaB = HeronsFormula(semiperimeterBeta, distance1, distanceTip, sideLengthB); 
      // double areaC = HeronsFormula(semiperimeterGamma, distanceTip, distance2, sideLengthC); 

      double bigTriArea = area((double)triangleTip.X(), (double)triangleTip.Y(), (double)bottom1.X(), (double)bottom1.Y(), (double)bottom2.X(), (double)bottom2.Y()); 

      double areaA = area((double)points[i].X(), (double)points[i].Y(), (double)triangleTip.X(), (double)triangleTip.Y(), (double)bottom1.X(), (double)bottom1.Y());
      double areaB = area((double)points[i].X(), (double)points[i].Y(), (double)triangleTip.X(), (double)triangleTip.Y(), (double)bottom2.X(), (double)bottom2.Y());
      double areaC = area((double)points[i].X(), (double)points[i].Y(), (double)bottom2.X(), (double)bottom2.Y(), (double)bottom1.X(), (double)bottom1.Y());

      double alpha, beta, gamma; 
      alpha = areaA / bigTriArea; 
      beta = areaB / bigTriArea; 
      gamma = areaC / bigTriArea; 

      double k = alpha + beta + gamma;

      if ((int)(1000 * (alpha + beta + gamma)) != 1000) {
          isClear = false; 
          if (areaC > areaB) {
            starboardSide = false; 
          }
      }
      if (!isClear) {
        swerveSubsystem->addingSwerveMovement = true;
  swerveSubsystem->addingXSpeed = units::meters_per_second_t{6};
  swerveSubsystem->addingYSpeed = units::meters_per_second_t{9};
  
      }

    }
    frc::SmartDashboard::PutBoolean("Is not in Triangle", isClear); 
    
  
  

  
  
     
}

bool LegAvoidanceCommand::IsFinished()
{
  return false;
}

void LegAvoidanceCommand::End(bool interrupted)
{
 
}