#include <iostream>
#include <thread>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <iterator>
#include <sys/types.h>
#include <sys/dir.h>
#include <memory>
#include <string>
#include <stdexcept>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

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
std::unordered_map<std::string, std::vector<std::vector<std::string>>> readDatFile(const std::string abspath, const std::string sep) {

    std::ifstream file(abspath);

    std::vector<std::vector<std::string>> lineset;
    std::unordered_map<std::string, std::vector<std::vector<std::string>>> finalMap;

    std::string currentKey;
    bool nosep = true;

    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {

            //if the current line begins with a separator...
            if (line.rfind(sep, 0) == 0) {
                
                //add to the map if there were lines before this separator.
                if (lineset.size() > 0) {
                    finalMap[currentKey] = lineset;
                    lineset.clear();
                }
                
                //Setting a new key indicates to add data under a new 'group'. We use the separator (current line) to index this group.
                //The line has regex characters and spaces so they need to be removed before the line is usable enough as an index.
                currentKey = line.substr(line.find(sep) + sep.length());
                currentKey = currentKey.erase(currentKey.find("\r"));

                nosep = false;
                //now continue to next line in the file!
            }
            else if (!nosep) {
                //Nothing else to do except add the current line to the lineset. It is under the current separator.
                //Also separate all the numbers into their own array values. I don't like this because it is potentially more memory consuming but it is more programmatically convenient.
                line = line.erase(line.find("\r"));
                lineset.push_back(
                    split(line, '\t')
                );
            }
        }

        //After iterating the last line we must add the lineset to the map manually because the loop does not do this.
        if (!currentKey.empty() && lineset.size() > 0 && !nosep) {
            finalMap[currentKey] = lineset;
            //lineset.clear() is redundant because garbage collection is about to happen.
        }
        
        file.close();
    }
    else {
        throw std::runtime_error("File " + abspath + " not found.");
    }
    return finalMap;
}


/**
 * 
 */


/**
 * Creates a point cloud from the data points
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFromData(const std::unordered_map<std::string, std::vector<std::vector<std::string>>> groups) {
    
    
    //These are references to the xyz headers in the original file.
    const std::string xCoordSep = "Calibrated xVector";
    const std::string yCoordSep = "Calibrated yVector";
    const std::string zCoordSep = "Calibrated Distance";

    //Are there an equal amount of xyz points? Every valid point must have a xyz value.
    //x implies z via y so an explicit check is unecessary.
    if (groups.at(xCoordSep).size() != groups.at(yCoordSep).size() &&
        groups.at(yCoordSep).size() != groups.at(zCoordSep).size() &&
        groups.at(xCoordSep).at(0).size() != groups.at(yCoordSep).at(0).size() &&
        groups.at(yCoordSep).at(0).size() != groups.at(zCoordSep).at(0).size()) {

            throw std::runtime_error("There are not an equal amount of points.");

    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (int y = 0; y < groups.at(xCoordSep).size(); y++) {
        for (int x = 0; x < groups.at(xCoordSep).at(0).size(); x++) {
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudRGBFromData(const std::unordered_map<std::string, std::vector<std::vector<std::string>>> groups, cv::Mat texture) {
    
    //These are references to the xyz headers in the original file.
    const std::string xCoordSep = "Calibrated xVector";
    const std::string yCoordSep = "Calibrated yVector";
    const std::string zCoordSep = "Calibrated Distance";

    //Are there an equal amount of xyz points? Every valid point must have a xyz value.
    //x implies z via y so an explicit check is unecessary.
    if (groups.at(xCoordSep).size() != groups.at(yCoordSep).size() &&
        groups.at(yCoordSep).size() != groups.at(zCoordSep).size() &&
        groups.at(xCoordSep).at(0).size() != groups.at(yCoordSep).at(0).size() &&
        groups.at(yCoordSep).at(0).size() != groups.at(zCoordSep).at(0).size()) {

            throw std::runtime_error("There are not an equal amount of points.");

    }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int y = 0; y < groups.at(xCoordSep).size(); y++) {
        for (int x = 0; x < groups.at(xCoordSep).at(0).size(); x++) {
            pcl::PointXYZRGB point;
            point.x = std::stod(groups.at(xCoordSep).at(y).at(x));
            point.y = std::stod(groups.at(yCoordSep).at(y).at(x));
            point.z = std::stod(groups.at(zCoordSep).at(y).at(x));
            point.r = texture.at<cv::Vec3b>(y, x)[0];
            point.g = texture.at<cv::Vec3b>(y, x)[1];
            point.b = texture.at<cv::Vec3b>(y, x)[2];
            // texture.at(x, y);

            // point.g = texture.at<uint8_t>(x, y)[1];
            // point.b = texture.at<uint8_t>(x, y)[2];
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

pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0.5, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.01);
  viewer->initCameraParameters ();
  return (viewer);
}

// int findmax(std::unordered_map<std::string, std::vector<std::vector<std::string>>> thing) {
    
//     const std::string xCoordSep = "Amplitude";
//     int max = INT_MIN;
    
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

//     for (int y = 0; y < thing.at(xCoordSep).size(); y++) {
//         for (int x = 0; x < thing.at(xCoordSep).at(0).size(); x++) {
//             int current = std::stod(thing.at(xCoordSep).at(y).at(x));
//             if (current > max) {
//                 max = current;
//             }
//         }
//     }
//     return max;
// }

// int findmin(std::unordered_map<std::string, std::vector<std::vector<std::string>>> thing) {
    
//     const std::string xCoordSep = "Amplitude";
//     int min = INT_MAX;
    
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

//     for (int y = 0; y < thing.at(xCoordSep).size(); y++) {
//         for (int x = 0; x < thing.at(xCoordSep).at(0).size(); x++) {
//             int current = std::stod(thing.at(xCoordSep).at(y).at(x));
//             if (current < min) {
//                 min = current;
//             }
//         }
//     }
//     return min;
// }

int countFiles(const std::string path) {

    DIR *dp;
    int i = 0;
    struct dirent *ep;
    const char * p = path.c_str();
    dp = opendir(p);

    if (dp != NULL) {
        while (ep = readdir (dp)) {
            i++;
        }
        (void)closedir (dp);
    }

    //temporary fix for UNIX
    if (i > 2) {
        i -= 2;
    }

    return i;
}

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args ) {
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

cv::Mat createGrayImageFrom2dVector(std::vector<std::vector<std::string>> amplitudes) {
    // cv::Mat result = cv::Mat::zeros(cv::Size(amplitudes.size(), amplitudes.at(0).size()));
    cv::Mat result = cv::Mat(cv::Size(amplitudes.size(), amplitudes.at(0).size()), CV_8UC1);
    uchar pixValue;
    for (int i = 0; i < result.cols; i++) {
        for (int j = 0; j < result.rows; j++) {
            cv::Vec3b &intensity = result.at<cv::Vec3b>(i, j);
            for(int k = 0; k < result.channels(); k++) {
                // calculate pixValue
                intensity.val[k] = std::stoi(amplitudes.at(i).at(j)) / 16382 * 255;
            }
        }
    }

    return result;
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


    // const std::string debugpcpath = "/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/calib/scene_1/depth_camera/depth_camera_1.dat";
    const std::string debugpcpath = "/home/steven/Desktop/ulcerdatabase/patients/case_2/day_1/data/scene_1/depth_camera/depth_camera_1.dat";
    const std::string debugimgpath = "/home/steven/Desktop/ulcerdatabase/patients/case_2/day_1/data/scene_1/mask.png";
    
    const std::string debugbaserecurse = "/home/steven/Desktop/ulcerdatabase/patients";
    const std::string debugtargetrecurse = "/case_%d/day_%d/calib/scene_1/depth_camera/";
    
    //"In UNIX everything is a file". Count the number of folders.
    int casefolders = countFiles(debugbaserecurse);

    // std::string debugpcpath = "";
    // std::unordered_map<std::string, std::vector<std::vector<std::string>>> asd = readDatFile(debugpcpath, "% ");
    

    std::unordered_map<std::string, std::vector<std::vector<std::string>>> depthdata;
    for (int patient = 1; patient <= casefolders; patient++) {
        int dayfolders = countFiles(
            debugbaserecurse + string_format("/case_%d", patient)
        );
        for (int day = 1; day <= dayfolders; day++) {
            std::string currentpath = debugbaserecurse + string_format(debugtargetrecurse, patient, dayfolders);

            int depthcameracount = countFiles(currentpath);

            //I will discard the files that have multiple .dat files.
            if (depthcameracount == 1) {
                depthdata = readDatFile(currentpath + "depth_camera_1.dat", "% ");
                cv::Mat grayimage = createGrayImageFrom2dVector(depthdata.at("Amplitude"));
                cv::namedWindow("sldkfjsfd", cv::WINDOW_NORMAL);
                cv::imshow("sldkfj", grayimage);
                cv::waitKey(0);
            }
        }

    }
    
    // for (int i = 1; i < 48; i++) {
    //     debugpcpath = "/home/steven/Desktop/ulcerdatabase/patients/case_" + std::to_string(i) + "/day_1/calib/scene_1/depth_camera/depth_camera_1.dat";
    //     asd = readDatFile(debugpcpath, "% ");
    //     std::cout << std::to_string(findmin(asd)) + ", " + std::to_string(findmax(asd)) << std::endl;
    // }
    // const std::string debugimgpath = "/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/data/scene_1/mask.png";

    //Get the data
    // std::cout << findmax(asd) << std::endl;
    cv::Mat image = cv::imread(debugimgpath, 1);
    // cv::Mat im2;
    cv::resize(image, image, cv::Size(176, 144), cv::INTER_LINEAR);
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    //Create the point cloud
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = pointCloudFromData(asd);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = pointCloudRGBFromData(depthdata, image);

    //Create the visualiser and render it.
    // pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(pointcloud);
    pcl::visualization::PCLVisualizer::Ptr viewer = rgbVis(pointcloud);
    while (!viewer->wasStopped()) {
        viewer->spin();
        // std::this_thread::sleep_for(100ms);
    }

    // cv::namedWindow("sldkfjsfd", cv::WINDOW_NORMAL);
    // cv::imshow("sldkfj", image);
    // cv::waitKey(0);

    return 1;
}

