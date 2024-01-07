#pragma once

#include <frc2/command/SubsystemBase.h>
#include "COMETS3357/Subsystems/SparkMax/SparkMaxVelocity.h"
#include "COMETS3357/PID.h"
#include "COMETS3357/TimerSubsystem.h"


#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
// #include <wpi/uv/Error.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
// #include <frc/kinematics/DifferentialDrivePoseEstimator.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/controller/RamseteController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <deque>

#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <math.h>

#define M_PI 3.14159265358979323846

namespace COMETS3357
{

    struct DrivebaseData
    {

    };

    class DrivebaseSubsystem : public frc2::SubsystemBase
    {
    public:

        DrivebaseSubsystem();

        void SetTicksToMetersConversion(double wheelDiameterInches, double gearRatio);

        void SetTicksToMetersConversion(double conversion);

        void SetVelocityPID(PID pid);

        void SetVelocity(double left, double right);

        void SetPercent(double left, double right);

        void ResetOdometry(double x, double y, double radians, double gyroYaw);

        double getEncoderDistance(double encoderPosition);

        void resetOdometry(const frc::Pose2d &pose, double gyroAngle);

        double rotationsToMeters = 0;

        frc::RamseteController ramseteController{};

        const units::radian_t kZeroAngle{0.0};
        units::meter_t meterX{0};
        units::meter_t meterY{0};
        const frc::Translation2d testTrans{meterX, meterY};
        units::radian_t zeroRadians{0};
        const frc::Rotation2d testRot{zeroRadians};
        const frc::Pose2d kZeroPose{testTrans, testRot};

        const units::meter_t kTrackWidth{0.48};
        frc::DifferentialDriveKinematics kinematics{kTrackWidth};

        frc::DifferentialDrivePoseEstimator odometry{kinematics, testRot, units::meter_t{0.0}, units::meter_t{0.0}, kZeroPose};   

        bool odometryInitialized = false;

        COMETS3357::SparkMaxVelocity dbR{""};
        COMETS3357::SparkMaxVelocity dbL{""};
        COMETS3357::SparkMaxVelocity dbFR{""};
        COMETS3357::SparkMaxVelocity dbFL{""};


        COMETS3357::PID pid{};

    };

};