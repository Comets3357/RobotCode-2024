
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <pigpio.h>
#include <string>
#include <fstream>
#include <AHRS.h>
#include <thread>
#include <map>
#include "nlohmann/json.hpp"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <chrono>

// #define ShowWindow

std::map<int, std::pair<double, double>> tagPositions;

// Define a custom conversion function
void from_json(const nlohmann::json& j, std::map<int, std::pair<double, double>>& m) {
  for (const auto& elem : j) {
    int i = elem.at(0).get<int>();
    std::pair<double, double> p = elem.at(1).get<std::pair<double, double>>();

std::cout << p.first << " " << p.second << std::endl;

    m[i] = p;
  }
}

double cameraX;
double cameraY;
double cameraPitch;
double cameraOffsetDistance;
double cameraOffsetDegree;

std::pair<double, double> triangulate2(int tagID1, int tagID2, double tagAngleOffset1, double tagAngleOffset2, double robotAngle)
{
    double tagDistance = sqrt(pow(tagPositions[tagID2].second - tagPositions[tagID1].second, 2) + pow(tagPositions[tagID2].first - tagPositions[tagID1].first, 2));
double distance = (sin((tagAngleOffset1 + robotAngle)+((M_PI/2)-atan2((tagPositions[tagID2].first - tagPositions[tagID2].first),(tagPositions[tagID2].second - tagPositions[tagID2].second)))) * tagDistance)/(sin(tagAngleOffset2 - tagAngleOffset1));
    return {tagAngleOffset2, distance};
}

std::pair<double, double> transformPosition(double x, double y, double angle)
{
   

    std::pair<double, double> pos = {cameraOffsetDistance * sin(angle + cameraOffsetDegree) + x, cameraOffsetDistance * cos(angle + cameraOffsetDegree) + y};

    return pos;
   

}

double tanX = tan(((70/180.0)*M_PI)/2.0);
        double tanY = tan(((42.5/180.0)*M_PI)/2.0);

std::pair<double, double> triangulateAprilTags(apriltag_detection_t* det1, apriltag_detection_t* det2, double robotAngle)
{
	    double centerX1 = ((det1->p[0][0] + det1->p[1][0] + det1->p[2][0] + det1->p[3][0])/4.0);
            double angleOffset1 = atan((tanX)*((centerX1-640.0)/640.0));

            double centerX2 = ((det2->p[0][0] + det2->p[1][0] + det2->p[2][0] + det2->p[3][0])/4.0);
            double angleOffset2 = atan((tanX)*((centerX2-640.0)/640.0));

            std::pair<double, double> posData = triangulate2(det1->id, det2->id, angleOffset1, angleOffset2, -robotAngle);

std::cout << posData.first << " " << posData.second << std::endl;

            double actualAngleOffset = posData.first + robotAngle;
            double x = sin(-actualAngleOffset) * -posData.second + tagPositions[det2->id].second;
            double y = cos(-actualAngleOffset) * -posData.second + tagPositions[det2->id].first;
            std::cout << "Gyro Angle: " << robotAngle << " Camera: " << posData.first << std::endl;
            std::cout << "X: " << x << " Y: " << y << std::endl;
	return {y, x};
}

int main() {

	nt::DoubleSubscriber timeSub;
   

    // Gets tag Positions
    std::ifstream ifs("/home/comets/VisionSystem/AprilTagData.json");
    nlohmann::json j = nlohmann::json::parse(ifs);
    from_json(j["AprilTagPositions"], tagPositions);


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
        std::cout << "Error opening camera" << std::endl;
	std::pair<double, double> idk = triangulate2(8, 7, -0.1, 0.1, M_PI);
	std::cout << idk.first << " " << idk.second << std::endl;

    }

AHRS ahrs = AHRS("/dev/ttyACM0");
std::this_thread::sleep_for(std::chrono::milliseconds(1000));

nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
timeSub = inst.GetTable("VisionSubsystem")->GetDoubleTopic("Time").Subscribe(0.0);

inst.StartClient4("VisionSubsystem");
inst.SetServerTeam(3357);

while (!inst.IsConnected() || inst.GetTable("VisionSubsystem")->GetEntry("Time").GetDouble(0.0) == 0)
{
	std::cout << "ASDKJAHSDKJ: " << inst.GetTable("VisionSubsystem")->GetEntry("Time").GetDouble(0.0) << " " << timeSub.Get(0.0) << std::endl;	
}
auto start = std::chrono::high_resolution_clock::now();
double timeOffset = inst.GetTable("VisionSubsystem")->GetEntry("Time").GetDouble(0.0);





    
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(cv::CAP_PROP_EXPOSURE, j["cam1"]["Exposure"].get<double>());
    cap.set(cv::CAP_PROP_BRIGHTNESS, j["cam1"]["Brightness"].get<double>());
    cap.set(cv::CAP_PROP_CONTRAST, j["cam1"]["Contrast"].get<double>());
    cap.set(cv::CAP_PROP_GAIN, j["cam1"]["Gain"].get<double>());

    cameraX = j["cam1"]["CameraX"].get<double>();
    cameraY = j["cam1"]["CameraY"].get<double>();
    cameraPitch = j["cam1"]["CameraPitch"].get<double>();

    cameraOffsetDistance = sqrt(pow(cameraY, 2) + pow(cameraX, 2));
    cameraOffsetDegree = atan2(cameraX, cameraY);

    #ifdef ShowWindow
    cv::namedWindow("Camera View", cv::WINDOW_AUTOSIZE);
    #endif

    
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("VisionSubsystem");

    nt::DoubleTopic xTopic = table->GetDoubleTopic("X");
    nt::DoubleTopic yTopic = table->GetDoubleTopic("Y");
    nt::DoubleTopic timestampTopic = table->GetDoubleTopic("Timestamp");
	nt::DoubleTopic yawTopic = table->GetDoubleTopic("Yaw");
nt::DoubleTopic rotVelocityTopic = table->GetDoubleTopic("RotVelocity");
nt::DoubleTopic velocityTopic = table->GetDoubleTopic("Velocity");
nt::DoubleTopic distanceTopic = table->GetDoubleTopic("Distance");
nt::DoubleTopic angleOffsetTopic = table->GetDoubleTopic("AngleOffset");

    nt::DoublePublisher xPublisher = xTopic.Publish();
    nt::DoublePublisher yPublisher = yTopic.Publish();
    nt::DoublePublisher timestampPublisher = timestampTopic.Publish();
	nt::DoublePublisher yawPublisher = yawTopic.Publish();
nt::DoublePublisher rotVelocityPublisher = rotVelocityTopic.Publish();
nt::DoublePublisher velocityPublisher = velocityTopic.Publish();
nt::DoublePublisher distancePublisher = distanceTopic.Publish();
nt::DoublePublusher angleOffsetPublisher = angleOffsetTopic.Publish();
nt::DoublePublisher IDPublisher = table->GetDoubleTopic("ID").Publish();

    //inst.StartClient4("VisionSubsystem");
    //inst.SetServerTeam(3357);
    // Main loop
    while (true) {

        double tempTime = inst.GetTable("VisionSubsystem")->GetEntry("Time").GetDouble(0.0);
        double tempcurrentTime = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() / 1000.0 + timeOffset;
        if (tempTime < tempcurrentTime)
        {
            timeOffset -= (tempcurrentTime - tempTime);
        }

        cv::Mat color_image;
        if (!cap.read(color_image))
        {
            // print an error message and exit
            std::cerr << "ERROR: cannot read the frame" << std::endl;
            break;
        }
	auto end = std::chrono::high_resolution_clock::now();
	double time = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() / 1000.0 + timeOffset;
	std::cout << time << std::endl;
	cv::flip(color_image, color_image, -1);

        cv::Mat gray;
        cv::cvtColor(color_image, gray, cv::COLOR_BGR2GRAY);

        #ifdef ShowWindow
        cv::imshow("Camera View", gray);
        cv::waitKey(10);
        #endif

        // Convert grayscale image to AprilTag's image_u8_t
        image_u8_t apriltag_image = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        // Detect AprilTags in the grayscale image
        zarray_t *detections = apriltag_detector_detect(td, &apriltag_image);
        
        double robotAngle = (ahrs.GetYaw() * M_PI / 180.0) + M_PI;

        if (zarray_size(detections) > 0)
        {
            apriltag_detection_t *det;
            zarray_get(detections, 0, &det);

            // Print ID and pose information

            double top_y = ((det->p[2][1] + det->p[3][1]) / 2);
            double bottom_y = ((det->p[0][1] + det->p[1][1]) / 2);
            double height = -(top_y - bottom_y);
            double angleTop = -atan((tanY)*((top_y-400.0)/400.0));
            double angleBottom = -atan((tanY)*((bottom_y-400.0)/400.0));
            double angle = angleTop - angleBottom;
            double actualHeight = (height * sin((M_PI/2)+(angle/2)))/(sin((M_PI/2)-angleTop));

        
            double centerX = ((det->p[0][0] + det->p[1][0] + det->p[2][0] + det->p[3][0])/4.0);
            double angleOffset = atan((tanX)*((centerX-640.0)/640.0));
            double halfHeightAngleOffset = atan(((actualHeight/2.0)/400.0)*tanY);
            double distance = 0.0795/tan(halfHeightAngleOffset);


        //     double actualAngleOffset = angleOffset + robotAngle;
        //     double x = cos(-actualAngleOffset) * distance + tagPositions[det->id].first;
        //     double y = sin(-actualAngleOffset) * distance + tagPositions[det->id].second;

		// //td::pair<double, double> newPose = transformPosition(x, y, -actualAngleOffset);
        //     std::cout << "Gyro Angle: " << robotAngle << " Camera: " << angleOffset << std::endl;
        //     std::cout << "X: " << x << " Y: " << y << std::endl;

		// std::cout << "ID: " << det->id << " posX: " <<  tagPositions[det->id].first << " posY: " << tagPositions[det->id].second << std::endl;
        std::cout << "detectedTag" << std::endl;

            // xPublisher.Set(x);
            // yPublisher.Set(y);
            timestampPublisher.Set(time);
            // yawPublisher.Set(-robotAngle);
            rotVelocityPublisher.Set(ahrs.GetRate());
            velocityPublisher.Set(0);
            distancePublisher.Set(distance);
            angleOffsetPublisher.Set(angleOffset);
            IDPublisher.Set(det->id);

        }
       
    }

    return 0;
}
