#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char** argv)
{
    if(argc < 4 || (argc >= 2 && argv[1] == "-h")) {
        std::cout << "marker_generator <marker_id> <marker_size> <output_file_path>" << std::endl;
        return 0;
    }

    const auto marker_id = std::atoi(argv[1]);
    const auto marker_size = std::atoi(argv[2]);
    const auto output_file = argv[3];

    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, marker_id, marker_size, markerImage, 1);
    cv::imwrite(output_file, markerImage);
    return 0;
}