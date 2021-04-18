#include <iostream>
#include <filesystem>
#include <fstream>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>


void writeSDF(const std::string& model_folder, const int tag_id, const double tag_size)
{
  std::ofstream file(model_folder + "/model.sdf");
  file << R"(<?xml version='1.0'?>
<sdf version='1.7'>
    <model name='ArUco Tag )" << tag_id << R"('>
        <link name='link'>
            <visual name='visual'>
                <geometry>
                    <plane>
                        <normal>0 0 1</normal>
                        <size>)" << tag_size << ' ' << tag_size << R"(</size>
                    </plane>
                </geometry>
                <material>
                    <script>
                        <uri>model://aruco_tag_)" << tag_id <<  R"(/materials/scripts/</uri>
                        <uri>model://aruco_tag_)" << tag_id <<  R"(/materials/textures/</uri>
                        <name>ArucoTag)" << tag_id << R"(/Image</name>
                    </script>
                </material>
            </visual>
        </link>
    </model>
</sdf>)";
}

void writeModelConfig(const std::string& model_folder, const int tag_id)
{
  std::ofstream file(model_folder + "/model.config");
  file << R"(<?xml version='1.0'?>
<model>
    <name>ArUco Tag )" << tag_id << R"(</name>
    <version>1.0</version>
    <sdf version='1.7'>model.sdf</sdf>
</model>)";
}

void writeMaterialScript(const std::string& model_folder, const int tag_id)
{
  std::ofstream file(model_folder + "/materials/scripts/aruco_tag_" + std::to_string(tag_id) + ".material");
  file << R"(material ArucoTag)" << tag_id << R"(/Image
{
    technique
    {
        pass
        {
            ambient 1 1 1 1
            diffuse 1 1 1 1
            specular 0.03 0.03 0.03 1.0
            texture_unit
            {
                texture ArUcoTag)" << tag_id << R"(.jpg
            }
        }
    }
})";
}

void writeImageTexture(const std::string& model_folder, const int tag_id, const int tag_resolution)
{
  cv::Mat markerImage;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::aruco::drawMarker(dictionary, tag_id, tag_resolution, markerImage, 1);
  const auto image_path = model_folder + "/materials/textures/ArUcoTag" + std::to_string(tag_id) + ".jpg";
  cv::imwrite(image_path, markerImage);
}

void createFolderStructure(const std::string& output_folder, const int tag_id)
{
  const auto model_name = "aruco_tag_" + std::to_string(tag_id);
  std::filesystem::create_directories(output_folder + '/' + model_name);
  std::filesystem::create_directories(output_folder + '/' + model_name + "/materials/scripts");
  std::filesystem::create_directories(output_folder + '/' + model_name + "/materials/textures");
}

void printUsage()
{
  std::cout << R"(tag_model_generator <tag_id> <tag_resolution> <tag_size> <output_folder_path>
  tag_id: ID number for the generated tag
  tag_resolution: pixel size of tag texture
  tag_size: physical size (meters) of tag model
  output_folder_path: folder in which the model folder will be generated        
)" << std::endl;
}

int main(int argc, char** argv)
{
    if(argc < 4 || (argc >= 2 && argv[1] == "-h")) {
        printUsage();
        return 0;
    }

    const auto tag_id = std::atoi(argv[1]);
    const auto tag_resolution = std::atoi(argv[2]);
    const auto tag_size = std::atof(argv[3]);
    const auto output_folder = std::string(argv[4]);

    createFolderStructure(output_folder, tag_id);

    const auto model_folder = output_folder + "/aruco_tag_" + std::to_string(tag_id);
    writeModelConfig(model_folder, tag_id);
    writeSDF(model_folder, tag_id, tag_size);
    writeMaterialScript(model_folder, tag_id);
    writeImageTexture(model_folder, tag_id, tag_resolution);

    return 0;
}
