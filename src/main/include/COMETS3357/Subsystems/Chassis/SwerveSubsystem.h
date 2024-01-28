#pragma once

#include <frc/ADIS16470_IMU.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "COMETS3357/Configs/ConfigFiles.h"

#include "COMETS3357/Subsystems/Chassis/MAXSwerveModule.h"
#include "units/time.h"
#include "COMETS3357/GyroSubsystem.h"

class RobotContainer;


namespace COMETS3357
{

  class SwerveSubsystem : public COMETS3357::Subsystem
  {
  public:
    SwerveSubsystem(std::string configFileName);

    SwerveConfig configuration;
    std::shared_ptr<nt::NetworkTable> gyroSubsystemData;

    
    COMETS3357::MAXSwerveModule m_frontLeft;
    COMETS3357::MAXSwerveModule m_rearLeft;
    COMETS3357::MAXSwerveModule m_frontRight;
    COMETS3357::MAXSwerveModule m_rearRight;

    void Initialize() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    // Subsystem methods go here.

    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
     * and the linear speeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction
     *                      (forward/backwards).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to
     *                      the field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    void Drive(units::meters_per_second_t xSpeed,
              units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
              bool fieldRelative, bool rateLimit, frc::SwerveDriveKinematics<4>* kinematics);
    
    void Drive(units::meters_per_second_t xSpeed,
              units::meters_per_second_t ySpeed, double directionX, double directionY,
              bool fieldRelative, bool rateLimit);

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    void SetX();

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    void ResetEncoders();

    frc::ChassisSpeeds getSpeeds();

    /**
     * Sets the drive MotorControllers to a power from -1 to 1.
     */
    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    units::degree_t GetHeading() const;

    /**
     * Zeroes the heading of the robot.
     */
    void ZeroHeading();

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    double GetTurnRate();

    wpi::array<frc::SwerveModulePosition, 4U> GetPositions();

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    frc::Pose2d GetPose();

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    void ResetOdometry(frc::Pose2d pose);

    /**
     * Drive for field relative swerve with the X axis for rotating
     * 
     * @param xSpeed drivebase speed in the x direction
     * @param ySpeed drivebase speed in the y direction
     * @param rot drivebase rotation speed with direction
    */
    void DriveXRotate(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot);

    /**
     * Drive for field relative swerve with directional steering
     * 
     * @param xSpeed drivebase speed in the x direction
     * @param ySpeed drivebase speed in the y direction
     * @param directionX x portion of the target facing direction of the drivebase
     * @param directionY y portion of the target facing direction of the drivebase
    */
    void DriveDirectionalRotate(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, double directionX, double directionY);

    /**
     * Drive for field relative swerve with the X axis for rotating while automatically choosing a corner to rotate around
     * 
     * @param xSpeed drivebase speed in the x direction
     * @param ySpeed drivebase speed in the y direction
     * @param rot drivebase rotation speed with direction
    */
    void DriveCornerTurning(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot);

    frc::SwerveDriveKinematics<4> kDriveKinematics{
        frc::Translation2d{},
        frc::Translation2d{},
        frc::Translation2d{},
        frc::Translation2d{}};

    frc::SwerveDriveKinematics<4> kDriveKinematicsFrontLeft{
        frc::Translation2d{},
        frc::Translation2d{},
        frc::Translation2d{},
        frc::Translation2d{}};
    
    frc::SwerveDriveKinematics<4> kDriveKinematicsFrontRight{
        frc::Translation2d{},
        frc::Translation2d{},
        frc::Translation2d{},
        frc::Translation2d{}};

    frc::SwerveDriveKinematics<4> kDriveKinematicsBackLeft{
        frc::Translation2d{},
        frc::Translation2d{},
        frc::Translation2d{},
        frc::Translation2d{}};

    frc::SwerveDriveKinematics<4> kDriveKinematicsBackRight{
        frc::Translation2d{},
        frc::Translation2d{},
        frc::Translation2d{},
        frc::Translation2d{}};

      void SetChassisSpeed(frc::ChassisSpeeds chassisSpeed);

      
    frc::SwerveDriveOdometry<4> m_odometry;

        bool controllingSwerveRotation = true;
    bool controllingSwerveMovement = true;

    units::meters_per_second_t overrideVelocityX{0};
    units::meters_per_second_t overrideVelocityY{0};
    units::radians_per_second_t overrideRotation{0};

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.



    // Slew rate filter variables for controlling lateral acceleration
    double m_currentRotation = 0.0;
    double m_currentTranslationDir = 0.0;
    double m_currentTranslationMag = 0.0;

    frc::SlewRateLimiter<units::scalar> m_magLimiter{units::unit_t<units::compound_unit<units::scalar, units::inverse<units::seconds>>>{0}};
    frc::SlewRateLimiter<units::scalar> m_rotLimiter{units::unit_t<units::compound_unit<units::scalar, units::inverse<units::seconds>>>{0}};
    double m_prevTime = wpi::Now() * 1e-6;

    // Odometry class for tracking robot pose
    // 4 defines the number of modules
    // frc::SwerveDriveOdometry<4> m_odometry;




    double actualAngle = 0; 
    double lastAngle = 0; 

    double kP = 1.25;
    double kD = 0;

    frc::PIDController rotationController{kP, 0, kD};

    bool pickedCorner = false;

    frc::SwerveDriveKinematics<4U>* currentKinematic;

    
  };

};