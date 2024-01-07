#pragma once


#include "COMETS3357/Subsystems/Subsystem.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <vector>

namespace COMETS3357
{


    class LimelightSubsystem : public COMETS3357::Subsystem
    {
    public:

        /**
     * @brief Constructs a new Limelight object.
     */
    LimelightSubsystem();

    /**
     * @brief Gets the horizontal offset from the Limelight.
     * @return The horizontal offset in degrees.
     */
    double getX();

    /**
     * @brief Gets the vertical offset from the Limelight.
     * @return The vertical offset in degrees.
     */
    double getY();

    /**
     * @brief Gets the target area from the Limelight.
     * @return The target area as a percentage.
     */
    double getArea();

    /**
     * @brief Checks if the Limelight has a valid target.
     * @return True if a valid target is found, false otherwise.
     */
    bool hasTarget();

    /**
     * @brief Gets the currently selected pipeline index.
     * @return The pipeline index.
     */
    double getPipeline();

    /**
     * @brief Sets the pipeline index to select a specific pipeline.
     * @param pipelineIndex The pipeline index to set.
     */
    void setPipeline(double pipelineIndex);

    /**
     * @brief Gets the currently selected LED mode.
     * @return The LED mode.
     */
    double getLEDMode();

    /**
     * @brief Sets the LED mode to control the Limelight's LEDs.
     * @param mode The LED mode to set.
     */
    void setLEDMode(double mode);

    /**
     * @brief Gets the currently selected camera mode.
     * @return The camera mode.
     */
    double getCameraMode();

    /**
     * @brief Sets the camera mode to control the Limelight's operation.
     * @param mode The camera mode to set.
     */
    void setCameraMode(double mode);

    /**
     * @brief Gets the pipeline's latency contribution in milliseconds.
     * @return The pipeline latency in milliseconds.
     */
    double getPipelineLatency();

    /**
     * @brief Gets the Limelight's firmware version.
     * @return The firmware version.
     */
    double getFirmwareVersion();

    /**
     * @brief Gets the length of the longer side of the fitted bounding box.
     * @return The length of the longer side.
     */
    double getTargetHorizontalLength();

    /**
     * @brief Gets the length of the shorter side of the fitted bounding box.
     * @return The length of the shorter side.
     */
    double getTargetVerticalLength();

    /**
     * @brief Gets the skew or rotation of the target (-90 degrees to 0 degrees).
     * @return The skew or rotation of the target.
     */
    double getTargetSkew();

    /**
     * @brief Gets the "short" side of the rough bounding box.
     * @return The length of the "short" side.
     */
    double getTargetShort();

    /**
     * @brief Gets the "long" side of the rough bounding box.
     * @return The length of the "long" side.
     */
    double getTargetLong();

    /**
     * @brief Gets the raw target vertices (x1, y1, x2, y2, x3, y3, x4, y4).
     * @return The raw target vertices.
     */

    void Periodic() override;
    
    std::vector<double> getRawTargetVertices();

    private:
        std::shared_ptr<nt::NetworkTable> table;
    };
};
