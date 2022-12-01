#include <iostream>
#include <string>
#include <vector>

#include "System.hpp"
#include "DogStatus.hpp"

#include "opencv2/opencv.hpp"

const char* keys = {" \\
    {help h ?               |    | this is help. } \\
    {input-path             |    | Write your Camera Path. } \\
    {lidar-path             |    | Write your LIDAR Path. } \\
    {arduino-path           |    | Write your Arduino Path.}"};

void help() {
    std::cout << "[SYSTEM INFORMATION]\n"
        "--------------------------------------------------------------------------------------------------------\n"
        "You need to write argument for this program. \n"
        "If you want simulation mode, you need to write your dataset path for simulation.\n"
        "If you want real mode, you need to write your camera path\n\n"

        "[Arguments]\n"
        "input-path = Your Camera path. check your connect status. (defualt : NULL)\n"
        "lidar-path = Your LIDAR path. check your connect status. (default : NULL)\n"
        "arduino-path = Your Arduino path. check your connect status. (default : NULL)\n"

        "\n[EXAMPLE]\n"
        "ex) ./auto --input-path=[YOUR PATH] --lidar-path=[YOUR PATH] --arduino-path=[YOUR PATH]\n"
        "--------------------------------------------------------------------------------------------------------\n" << std::endl;
}

int main(int argc, char* argv[]) {
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("AutoDog System v1.0.0");

    if (argc < 2) {
        help();
        return -1;
    }
    if ( parser.has("help")) {
        help();
        return 0;
    }

    std::string input_path = parser.get<std::string>("input-path");
    std::string lidar_path = parser.get<std::string>("lidar-path");
    std::string arduino_path = parser.get<std::string>("arduino-path");

    if ( lidar_path.empty() ) {
        std::cout << "[ERROR]" << std::endl;
        std::cout << "Our System need to LiDAR!, Check Your LiDAR." << std::endl;
        return -1;
    }
    else {
        System* system = new System(input_path, lidar_path, arduino_path);
        system->startProgram();
    }

    return 0;
}
