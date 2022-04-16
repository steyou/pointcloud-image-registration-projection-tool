#include <iostream>
#include <thread>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <iterator>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
// #include <chrono>

template <typename Out>
void split(const std::string &s, char delim, Out result) {
    std::istringstream iss(s);
    std::string item;
    while (std::getline(iss, item, delim)) {
        *result++ = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

/**
 * Returns the contents of a point cloud .dat file
 * @param abspath The absolute path of the file you want.
 * @param sep A common string used before the group identifier string. Used to identify the beginning of a new group.
 */
std::unordered_map<std::string, std::vector<std::vector<std::string>>> readFile(const std::string abspath, const std::string sep) {

    //TODO last line bug

    std::ifstream file(abspath);

    std::vector<std::vector<std::string>> lineset;
    std::unordered_map<std::string, std::vector<std::vector<std::string>>> finalMap;

    std::string currentKey;

    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            // using printf() in all tests for consistency
            // printf("%s", line.c_str());
            // lines.push_back(line.c_str());

            //if the current line begins with a separator...
            if (line.rfind(sep, 0) == 0) {
                
                //add to the map if there were lines before this separator.
                if (lineset.size() > 0) {
                    finalMap[currentKey] = lineset;
                    lineset.clear();
                }
                
                //setting a new key indicates to add data under a new 'group'
                currentKey = line.substr(line.find(sep) + sep.length());
                currentKey = currentKey.erase(currentKey.find("\r"));

                //now continue to next line in the file!
            }
            else {
                //Nothing else to do except add the current line to the lineset. It is under the current separator.
                //TODO space delimit?
                lineset.push_back(
                    split(line, '\t')
                );
            }
        }

        //After iterating the last line we must add the lineset to the map manually because the loop does not do this.
        if (!currentKey.empty() && lineset.size() > 0) {
            finalMap[currentKey] = lineset;
            //lineset.clear() redundant because garbage collection is about to happen.
        }
        
        file.close();
    }
    else {
        throw std::runtime_error("File " + abspath + " not found.");
    }
    return finalMap;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFromData(const std::unordered_map<std::string, std::vector<std::vector<std::string>>> groups) {
    
    
    //These are references to the xyz headers in the original file.
    const std::string xCoordSep = "Calibrated xVector";
    const std::string yCoordSep = "Calibrated yVector";
    const std::string zCoordSep = "Calibrated Distance";


    if (groups.at(xCoordSep).size() != groups.at(yCoordSep).size() &&
        groups.at(yCoordSep).size() != groups.at(zCoordSep).size()) {

            throw std::runtime_error("There are not an equal amount of points.");

    }

    //set loop ends

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (int y = 0; y < 143; y++) {
        for (int x = 0; x < 175; x++) {
            pcl::PointXYZ point;
            point.x = std::stod(groups.at(xCoordSep).at(y).at(x));
            point.y = std::stod(groups.at(yCoordSep).at(y).at(x));
            point.z = std::stod(groups.at(zCoordSep).at(y).at(x));
            // point.y = 
            // point.z = 
            cloud->points.push_back(point);
        }
    }

    cloud->width = cloud->size();
    cloud->height = 1;

    return cloud;
    
}

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (0.01);
    viewer->initCameraParameters();
//   return (nullptr);
    return (viewer);
}

int main(int argc, char* argv[]) {

    //Handle command line arguments
    // if (argc < 2) {
    //     std::cout << "this the help text" << std::endl;
    //     return 0;
    // }
    // // exit if the image and point cloud arguments are not provided.
    // else if (argc == 2)
    // {
    //     std::cout << "Error: missing point cloud argument" << std::endl;
    //     return 0;
    // }
    // else if (argc > 3) {
    //     std::cout << "Error: expected exactly 2 arguments"<<std::endl;
    //     return 0;
    // }
    // for (int i = 0; i < argc; ++i)
    //     std::cout << argv[i] << std::endl;
    
    // std::cout << "sdlkfjhsdklf" << std::endl;
    // std::vector<std::string> rawpcdata = readFile(argv[1]);
    const std::string debugpath = "/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/data/scene_1/depth_camera/depth_camera_1.dat";
    std::unordered_map<std::string, std::vector<std::vector<std::string>>> asd = readFile(debugpath, "% ");

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = pointCloudFromData(asd);
    pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(pointcloud);

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    while (!viewer->wasStopped()) {
        viewer->spin();
        // std::this_thread::sleep_for(100ms);
    }

    return 1;
}

