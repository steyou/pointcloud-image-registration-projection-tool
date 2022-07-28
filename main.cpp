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
#include <filesystem>
#include <iostream>
#include <regex>
// #include <float.h>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/regex.hpp>
// #include <boost/algorithm/string/predicate.hpp>

#include "libcbdetect/boards_from_corners.h"
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/plot_boards.h"
#include "libcbdetect/plot_corners.h"

// namespace fs = std::filesystem;
namespace boostfs = boost::filesystem;
namespace chrono = std::chrono;

using std::chrono_literals::operator""ms;


std::string inputDir = "";

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

    //Did we reach the end without a sep? If yes, it is a bad file. It is not this function's responsibility to fix it so we throw an exception.
    if (nosep) {
        throw std::runtime_error("No sep found.");
    }

    return finalMap;
}


// double fminAny(double nums[]) {
//     int length = sizeof(nums)/sizeof(*nums);
//     if (length < 2) {
//         throw std::runtime_error("Not enough arguments.");
//     }
//     if (length == 2) {
//         return fmin(nums[0], nums[1]);
//     }
//     for (int i = 0; i < length; i++) {
//         return fminAny(nums)
//     }
// }

// double fminAny(double nums[]) {
//     int length = sizeof(nums)/sizeof(*nums);
//     if (length < 2) {
//         throw std::invalid_argument("Need at least 2 arguments.");
//     }

//     double min = DBL_MAX;
//     for (int i = 0; i < length; i++) {
//         if (nums[i] < min) {
//             min = nums[i];
//         }
//     }
// }


uint size2uint(size_t val) {
    return (val <= INT_MAX) ? (uint)((ssize_t)val) : -1;
}

uint minOfThree(const uint n1, const uint n2, const uint n3) {
    // if (nums.size() < 2) {
    //     throw std::invalid_argument("Need at least 2 arguments.");
    // }
    // int min = INT_MAX;
    // for (int i = 0; i < nums.size(); i++) {
    //     if (nums[i] < min) {
    //         min = nums[i];
    //     }
    // }
    // return min;

    if (n1 < n3 && n1 < n2) {
        return n1;
    }
    if (n2 < n1 && n2 < n3) {
        return n2;
    }
    if (n3 < n1 && n3 < n2) {
        return n3;
    }

    // They are equal.
    return n1;

}


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
    if (
        groups.at(xCoordSep).size() != groups.at(yCoordSep).size() ||
        groups.at(yCoordSep).size() != groups.at(zCoordSep).size() ||
        groups.at(xCoordSep).at(0).size() != groups.at(yCoordSep).at(0).size() ||
        groups.at(yCoordSep).at(0).size() != groups.at(zCoordSep).at(0).size()
    ) {
        throw new std::runtime_error("Unequal XYZ point dimensions.");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (int y = 0; y < groups.at(xCoordSep).size(); y++) {
        for (int x = 0; x < groups.at(xCoordSep).at(0).size(); x++) {
            pcl::PointXYZ point;
            point.x = std::stod(groups.at(xCoordSep).at(y).at(x));
            point.y = std::stod(groups.at(yCoordSep).at(y).at(x));
            point.z = std::stod(groups.at(zCoordSep).at(y).at(x));
        }
    }

    cloud->width = cloud->size();
    cloud->height = 1;

    return cloud;
    
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudFromData(const std::unordered_map<std::string, std::vector<std::vector<std::string>>> groups, cv::Mat texture) {
    
    //These are references to the xyz headers in the original file.
    const std::string xCoordSep = "Calibrated xVector";
    const std::string yCoordSep = "Calibrated yVector";
    const std::string zCoordSep = "Calibrated Distance";

    //Are there an equal amount of xyz points? Every valid point must have a xyz value.
    //x implies z via y so an explicit check for equality with z and the others is unecessary.
    // if (
    //     groups.at(xCoordSep).size() != groups.at(yCoordSep).size() ||
    //     groups.at(yCoordSep).size() != groups.at(zCoordSep).size() ||
    //     groups.at(xCoordSep).at(0).size() != groups.at(yCoordSep).at(0).size() ||
    //     groups.at(yCoordSep).at(0).size() != groups.at(zCoordSep).at(0).size()
    // ) {
    //     throw new std::runtime_error("Unequal XYZ point dimensions.");
    // }

    auto yx = groups.at(xCoordSep).size();
    auto yy = groups.at(yCoordSep).size();
    auto yz = groups.at(zCoordSep).size();

    auto xx = groups.at(xCoordSep).at(0).size();
    auto xy = groups.at(yCoordSep).at(0).size();
    auto xz = groups.at(zCoordSep).at(0).size();

    uint yMax = minOfThree(yx, yy, yz);;
    uint xMax = minOfThree(xx, xy, xz);;

    // If the above lines cause the point cloud to be cropped (as a result of dimensions not matching), the image we are about to project must be cropped too.
    // cv::Rect cropArea = cv::Rect(0, 0, xMax, yMax);
    // cv::Mat croppedTexture = texture(cropArea);

    // cv::cvtColor(croppedTexture, croppedTexture, cv::COLOR_BGR2RGB);

    // cv::imshow("skldjflsd", croppedTexture);
    // cv::waitKey();
    // exit(0);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int y = 0; y < yMax; y++) {
        for (int x = 0; x < xMax; x++) {
            pcl::PointXYZRGB point;
            point.x = std::stod(groups.at(xCoordSep).at(y).at(x));
            point.y = std::stod(groups.at(yCoordSep).at(y).at(x));
            point.z = std::stod(groups.at(zCoordSep).at(y).at(x));
            point.r = texture.at<cv::Vec3b>(y, x)[0];
            point.g = texture.at<cv::Vec3b>(y, x)[1];
            point.b = texture.at<cv::Vec3b>(y, x)[2];
            
            cloud->points.push_back(point);
        }
    }

    cloud->width = cloud->size();
    cloud->height = 1;

    return cloud;
}


pcl::visualization::PCLVisualizer::Ptr visualizePointCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(0.01);
    viewer->initCameraParameters();
    return (viewer);
}


pcl::visualization::PCLVisualizer::Ptr visualizePointCloud (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (0.01);
    viewer->initCameraParameters ();
    return (viewer);
}


cv::Mat depthImageFromAmplitudes(std::vector<std::vector<std::string>> amplitudes) {
    // cv::Mat result = cv::Mat::zeros(cv::Size(amplitudes.size(), amplitudes.at(0).size()));
    cv::Mat result = cv::Mat(cv::Size(amplitudes.at(0).size(), amplitudes.size()), CV_8UC1);
    uchar pixValue;
    int i = 0;
    for (i; i < result.cols; i++) {
        for (int j = 0; j < result.rows; j++) {
            cv::Vec3b &intensity = result.at<cv::Vec3b>(j, i);
            //note this variable could be typed uint8_t because all possible values range 0-255. I wont do this because I have no idea if that introduces any problems.
            int color = std::round(std::stod(amplitudes.at(j).at(i)) / 16382 * 255);
            result.at<uchar>(j,i) = color;
            
        }
    }

    return result;
}

/**
 * 
 */
void detectChessCorners(cv::Mat img, cbdetect::Params cbParams) {

    cbdetect::Corner corners;
    std::vector<cbdetect::Board> boards;

    // auto t1 = chrono::high_resolution_clock::now();
    cbdetect::find_corners(img, corners, cbParams);
    // auto t2 = chrono::high_resolution_clock::now();
    cbdetect::plot_corners(img, corners);
    // auto t3 = chrono::high_resolution_clock::now();
    cbdetect::boards_from_corners(img, corners, boards, cbParams);
    // auto t4 = chrono::high_resolution_clock::now();

    // printf("Find corners took: %.3f ms\n", chrono::duration_cast<chrono::microseconds>(t2 - t1).count() / 1000.0);
    // printf("Find boards took: %.3f ms\n", chrono::duration_cast<chrono::microseconds>(t4 - t3).count() / 1000.0);
    // printf("Total took: %.3f ms\n", chrono::duration_cast<chrono::microseconds>(t2 - t1).count() / 1000.0 + chrono::duration_cast<chrono::microseconds>(t4 - t3).count() / 1000.0);
    cbdetect::plot_boards(img, corners, boards, cbParams);
}

void detectChessCorners() {

    std::cout << inputDir << std::endl;
    // const std::string debugimgpath = "/home/steven/Desktop/ulcerdatabase/patients/case_25/day_1/calib/scene_1";
    // inputDir = debugimgpath;
    std::string depthFile = "/depth_camera/depth_camera_1.dat";
    std::unordered_map<std::string, std::vector<std::vector<std::string>>> depthdata;
    bool nullness = true;
    try {
        depthdata = readDatFile(inputDir + depthFile, "% ");
        nullness = false;
    }
    catch(const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
    }  
    if (!nullness) {
        cv::Mat amplitudeImage = depthImageFromAmplitudes(depthdata.at("Amplitude"));
        // cv::Mat photoImage = cv::imread(debugimgpath);
        // cv::resize(photoImage, photoImage, cv::Size(photoImage.cols/2, photoImage.rows/2));

        // cv::imshow("sldkfj", amplitudeImage);
        // cv::waitKey(0);

        // cv::equalizeHist(amplitudeImage, amplitudeImage);
        cv::resize(amplitudeImage, amplitudeImage, cv::Size(amplitudeImage.cols*2, amplitudeImage.rows*2));
        // cv::medianBlur(amplitudeImage, amplitudeImage, 5);

        // cv::Mat sharpeningKernel = (
        //     cv::Mat_<double>(3, 3) <<
        //     -1, -1, -1,
        //     -1, 9, -1,
        //     -1, -1, -1
        // );
        // cv::filter2D(amplitudeImage, amplitudeImage, -1, sharpeningKernel);

        // cv::threshold(amplitudeImage, amplitudeImage, 190, 90, cv::THRESH_TRIANGLE);

        // cv::imshow("sdfssd", amplitudeImage);
        // cv::waitKey();

        cbdetect::Params amplitudeParams;
        amplitudeParams.corner_type = cbdetect::SaddlePoint;
        // amplitudeParams.init_loc_thr = 0.001;
        // amplitudeParams.score_thr = 0.001;
        amplitudeParams.radius = {1,3};

        detectChessCorners(amplitudeImage, amplitudeParams);
    }

}

/**
 * Verifies if a directory contains a matching sub-directory, as defined in `regexCriteria`, and if so, executes an arbitrary function, for each matching sub-directory.
*/
bool searchdir(std::string path, std::deque<std::string> regexCriteria, void (&doWhenMet)()) {
    // This function exists for a few reassons:
    //      1 - I needed to use boostfs::recursive_directory_iterator but I do not need the same amount of search depth.
    //      2 - Looping through directories and executing arbitrary functions is a nice-to-have if I convert this into a terminal app
    //      3 - Minimise code re-use. if I need to search the (tedious) dataset folder, I can do so just with this function.

    bool customFunctionExecuted = false;

    boostfs::path boostpath = boostfs::path(path);
    boostfs::directory_iterator itr(boostpath);
    for (; itr != boostfs::directory_iterator(); itr++) {

        std::string currentPath = itr->path().string();
        std::string first = regexCriteria.front();
        boost::regex searchThisDirectory(first.c_str());
        
        // This area has all the criteria so execute the function
        // We stop at one criterion left because it is the last. Directory names can be treated exclusive here.
        // TODO write wrapper function that asserts > 1
        if (regexCriteria.size() == 1) {
            inputDir = currentPath;
            doWhenMet();
            customFunctionExecuted = true;
        }
        // Is this inside the root directory?
        // this is an else-if because it must be mutually exclusive from the first if.
        else if (boost::regex_search(currentPath, searchThisDirectory)) {
            // Remove the first criteria so we can check the rest in the next search deeper.
            regexCriteria.pop_front();
            
            // Recursion recursion recursion recursion
            searchdir(currentPath, regexCriteria, doWhenMet);

            // Adding back the criteria may seem counterintuitive but it must be done in case there are multiple folders in this current path (remember we're also in a for loop).
            regexCriteria.push_front(first);
        }
        // If no, do not search deeper.
    }
    return customFunctionExecuted;
};

/**
 * Given the path to a wound capture scene, takes a point cloud .dat and rgb .jpg and saves it somewhere.
*/
void exportScenePlyRgb() {
    // std::cout << inputDir << std::endl;
    
    // This is a relative path
    std::string depthFile = "/depth_camera/depth_camera_1.dat";
    std::unordered_map<std::string, std::vector<std::vector<std::string>>> depthdata;
    bool nullness = true;
    try {
        depthdata = readDatFile(inputDir + depthFile, "% ");
        nullness = false;
    }
    //Empty catch may seem dubious but the only time exceptions should only happen when the file is wrong. In that case there is nothing the program can do but move on.
    catch(const std::runtime_error& e) {
        // std::cout << e.what() << std::endl;
    }                
    if (!nullness) {
        // From here we can do multiple things. We can retrieve the amplitude data for every depth image. Or we can export the dataset with ground truths.

        // std::vector<std::vector<std::string>> amplitudes = depthdata.at("Amplitude");
        // cv::Mat depthimage = depthImageFromAmplitudes(amplitudes);

        // Load the images
        cv::Mat rgbImage = cv::imread(inputDir + "/photo.jpg");
        cv::Mat maskImage = cv::imread(inputDir + "/mask.png");

        // 176x144 are the depth camera's dimensions
        cv::Size depthCameraNormalSize = cv::Size(176, 144);
        cv::resize(rgbImage, rgbImage, depthCameraNormalSize);
        cv::resize(maskImage, maskImage, depthCameraNormalSize);

        cv::cvtColor(rgbImage, rgbImage, cv::COLOR_BGR2RGB);
        cv::cvtColor(maskImage, maskImage, cv::COLOR_BGR2RGB);
        // cv::cvtColor(maskImage, maskImage, cv::COLOR_BGR2GRAY);

        // cv::imshow("lorem", maskImage);
        // cv::waitKey();
        // exit(0);

        // Create the point clouds from the depth data and images.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud = pointCloudFromData(depthdata, rgbImage);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr maskCloud = pointCloudFromData(depthdata, maskImage);

        // The processed point clouds are scaled 1000 times. Do the same here.
        Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Identity();
        scaleMatrix.block<4,3>(0,0) *= 1000;

        pcl::transformPointCloud(*rgbCloud, *rgbCloud, scaleMatrix);
        pcl::transformPointCloud(*maskCloud, *maskCloud, scaleMatrix);


        //Create the visualiser and render it.
        // pcl::visualization::PCLVisualizer::Ptr viewer = visualizePointCloud(rgbCloud);
        // while (!viewer->wasStopped()) {
        //     viewer->spin();
        //     std::this_thread::sleep_for(100ms);
        // }
        // exit(0);

        // This removes the base directory from the current path so we can use the export base path instead. A strange solution, yes, but it works.
        // TODO Separate the base dir more elegantly. Currently it is built into the directory iterators so it isn't so straightforward.
        const std::string debugbasedir = "/home/steven/Desktop/ulcerdatabase/patients";
        const std::string debugexportbase = "/home/steven/Desktop/rgbwounds";
        std::string currentTarget = inputDir.substr(
            inputDir.find(debugbasedir) + debugbasedir.length()
        );
        std::string fullExportPath = debugexportbase + currentTarget;
        std::cout << fullExportPath << std::endl;

        pcl::PLYWriter writer = pcl::PLYWriter();

        boostfs::create_directories(fullExportPath);
        writer.write(fullExportPath + "/rgb.ply", *rgbCloud);

        boostfs::create_directories(fullExportPath);
        writer.write(fullExportPath + "/mask.ply", *maskCloud);

        // It seems you do not need to close() the writer to prevent memory leaks.
    }
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
    
    // Top two vars are used for example clouds/images
    const std::string debugpcpath = "/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/calib/scene_1/depth_camera/depth_camera_1.dat";
    const std::string debugimgpath = "/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/calib/scene_1/photo.jpg";
    
    const std::string debugbasedir = "/home/steven/Desktop/ulcerdatabase/patients";
    const std::string debugexportbase = "/home/steven/Desktop/rgbwounds";
    const bool debuguseregistration = true;

    // Fill a deque (I do it this way for the curly brace syntactic sugar that deques do not support)
    // std::vector<std::string> a = {"case_\\d*$", "day_\\d*$", "data$", "scene_\\d*$"};
    // std::deque<std::string> b;
    // for (std::string thing : a) {
    //     b.push_back(thing);
    // }
    // searchdir(debugbasedir, b, exportScenePlyRgb);
    
    if (debuguseregistration) {
        
        std::vector<std::string> a = {"case_\\d*$", "day_\\d*$", "calib$", "scene_\\d*$"};
        std::deque<std::string> b;
        for (std::string thing : a) {
            b.push_back(thing);
        }
        searchdir(debugbasedir, b, detectChessCorners);

        // std::unordered_map<std::string, std::vector<std::vector<std::string>>> depthdata = readDatFile(debugpcpath, "% ");
        // cv::Mat amplitudeImage = depthImageFromAmplitudes(depthdata.at("Amplitude"));
        // cv::Mat photoImage = cv::imread(debugimgpath);
        // cv::resize(photoImage, photoImage, cv::Size(photoImage.cols/2, photoImage.rows/2));

        // cv::imshow("sldkfj", amplitudeImage);
        // cv::waitKey(0);

        // cv::equalizeHist(amplitudeImage, amplitudeImage);
        // cv::resize(amplitudeImage, amplitudeImage, cv::Size(amplitudeImage.cols*2, amplitudeImage.rows*2));

        // cbdetect::Params amplitudeParams;
        // amplitudeParams.corner_type = cbdetect::SaddlePoint;
        // amplitudeParams.init_loc_thr = 0.001;
        // amplitudeParams.score_thr = 0.001;
        // amplitudeParams.radius = {1,3};
        // amplitudeParams.detect_method = cbdetect::DetectMethod::LocalizedRadonTransform;
        // amplitudeParams.

        // cbdetect::Params rgbParams;
        // rgbParams.corner_type = cbdetect::SaddlePoint;
        // rgbParams

        // detectChessCorners(amplitudeImage, amplitudeParams);
        // detectChessCorners(photoImage, rgbParams);

        // cbdetect::Corner amplitudeCorners;
        // cbdetect::Corner photoCorners;

        // cbdetect::find_corners(amplitudeImage, amplitudeCorners, amplitudeParams);
        // cbdetect::find_corners(photoImage, photoCorners, rgbParams);

        // photoCorners.p.pop_back();

        // cv::Mat homography = cv::findHomography(photoCorners.p, amplitudeCorners.p);
        // // std::cout << homography << std::endl;

        // cv::Mat actualrgb = cv::imread("/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/data/scene_1/photo.jpg");
        // cv::cvtColor(actualrgb, actualrgb, cv::COLOR_BGR2RGB);
        
        // cv::Mat warped;
        // cv::warpPerspective(actualrgb, warped, homography, actualrgb.size());

        // // cv::Rect r2 = cv::Rect(0, 0, amplitudeImage.cols, amplitudeImage.rows);
        // // warped = warped(r2);
        
        // // cv::cvtColor(warped,warped,cv::COLOR_GRAY2BGR);
        // cv::cvtColor(warped,warped,cv::COLOR_BGR2RGB);
        // cv::imshow("dsfjlksdf", warped);
        // cv::waitKey();

    }

    

    //Create the point cloud
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = pointCloudFromData(depthdata, image);

    //Create the visualiser and render it.
    // pcl::visualization::PCLVisualizer::Ptr viewer = visualizePointCloud(pointcloud);
    // while (!viewer->wasStopped()) {
        // viewer->spin();
        // std::this_thread::sleep_for(100ms);
    // }

    // cv::imshow("sldkfj", depthimage);
    // cv::imshow("sldkfj", dst_norm_scaled);
    // cv::waitKey(0);

    return 1;
}

