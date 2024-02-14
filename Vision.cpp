#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <pigpio.h>
#include <string>
#include <fstream>
#include <thread>
#include <map>
#include "nlohmann/json.hpp"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <vector>
#include <networktables/DoubleTopic.h>
#include <chrono>
#include <string>
#include <networktables/DoubleArrayTopic.h>

double cameraPitch = 20 * 3.14159 / 180.0;

int main() {

    int frame = 0;

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 908.537, 0, 619.998, 0, 909.839, 407.677, 0, 0, 1);

    std::vector<double> outputBuffer;
   
    std::vector<cv::Point3f> objectPoints;
    // Fill in the detected image points (in pixels)
    objectPoints.push_back(cv::Point3f(-0.08255, 0.08255, 0));
    objectPoints.push_back(cv::Point3f(0.08255, 0.08255, 0));
    objectPoints.push_back(cv::Point3f(0.08255, -0.08255, -0));
    objectPoints.push_back(cv::Point3f(-0.08255, -0.08255, -0));

    double dist_coeff[] = {0.042, -0.058, -0.00013, 0.00094, -0.003};
    cv::Mat distCoeffs = cv::Mat(1, 5, CV_64F, dist_coeff);
   

    // Gets tag Positions
    std::ifstream ifs("/home/comets/VisionSystem/AprilTagData.json");
    nlohmann::json j = nlohmann::json::parse(ifs);

    // Initialize AprilTag detector
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // Captures video from camera
    cv::VideoCapture cap(-1);


    // Checks if the camera is open
    while (!cap.isOpened())
    {
        cap = cv::VideoCapture(-1);
    }

    cap.set(cv::CAP_PROP_FOURCC, ('M','J','P','G') );
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FPS, 120);

    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    nt::DoubleArrayTopic tagDataTopic = inst.GetTable("VisionSubsystem")->GetDoubleArrayTopic("TagData");

    inst.StartClient4("VisionSubsystem");
    inst.SetServerTeam(3357);

    while (!inst.IsConnected())
    {

    }
    
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(cv::CAP_PROP_EXPOSURE, j["cam1"]["Exposure"].get<double>());
    cap.set(cv::CAP_PROP_BRIGHTNESS, j["cam1"]["Brightness"].get<double>());
    cap.set(cv::CAP_PROP_CONTRAST, j["cam1"]["Contrast"].get<double>());
    cap.set(cv::CAP_PROP_GAIN, j["cam1"]["Gain"].get<double>());


    nt::DoubleArrayPublisher tagData = tagDataTopic.Publish();

    while (true) {

     
        cv::Mat color_image;
        if (!cap.read(color_image))
        {
            // print an error message and exit
            std::cerr << "ERROR: cannot read the frame" << std::endl;
            break;
        }
        auto imageTime = nt::Now();

        cv::Mat gray;
        cv::cvtColor(color_image, gray, cv::COLOR_BGR2GRAY);

        // Convert grayscale image to AprilTag's image_u8_t
        image_u8_t apriltag_image = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        // Detect AprilTags in the grayscale image
        zarray_t *detections = apriltag_detector_detect(td, &apriltag_image);

        frame++;

        for (int i = 0; i < zarray_size(detections); i++ )
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);


            std::vector<cv::Point2f> imagePoints;
            // Fill in the detected image points (in pixels)
            imagePoints.push_back(cv::Point2f(det->p[0][0], det->p[0][1]));
            imagePoints.push_back(cv::Point2f(det->p[1][0], det->p[1][1]));
            imagePoints.push_back(cv::Point2f(det->p[2][0], det->p[2][1]));
            imagePoints.push_back(cv::Point2f(det->p[3][0], det->p[3][1]));

            cv::Mat rvec, tvec;
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

            double tx = tvec.at<double>(0, 0);
            double ty = tvec.at<double>(1, 0);
            double tz = tvec.at<double>(2, 0);

            double distance1 = (sqrt(pow(tx, 2) + pow(ty, 2) + pow(tz, 2))) * cos(atan2(ty, sqrt(pow(tx, 2) + pow(tz, 2))) + (cameraPitch));
            outputBuffer.push_back((double)det->id);
            outputBuffer.push_back(distance1);
            double angleOffset = -acos(((tz - (ty*tan(cameraPitch))) * cos(cameraPitch))/distance1);
            angleOffset += (tx < 0) ? -1 : 1;
            outputBuffer.push_back(angleOffset);
            outputBuffer.push_back((double)frame);

            

        }

    
        if (outputBuffer.size() > 0)
        {
            tagData.Set(outputBuffer, imageTime);
        }

        outputBuffer.clear();
    }       
    return 0;
}
