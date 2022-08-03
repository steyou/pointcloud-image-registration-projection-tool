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
#include <boost/algorithm/string/find.hpp>
#include <boost/regex.hpp>

namespace boostfs = boost::filesystem;
namespace chrono = std::chrono;

using std::chrono_literals::operator""ms;

const std::string CWD = boostfs::current_path().string();
const std::string EXPORT_LIST_PATH = CWD + "/exportlist.txt";

const cv::Size DEPTH_BLOCK_SIZE = cv::Size(176, 144);

enum Block { Distance, X, Y, Amplitude, Confidence };

bool exportListLoaded = false;
std::set<std::string> exportList;

/**
 * Converts a bool to a "True" or "False" string representation.
 * @param a The boolean to convert.
 */
std::string bolst(bool a) {
    return a ? "True" : "False";
}


uint minOfThree(const uint n1, const uint n2, const uint n3) {
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
 * Returns the contents of a point cloud .dat file
 * @param abspath The absolute path of the file you want.
 * @param block The section of the file you want to read. These files have the same structure, as ordered by Block.
 */
std::vector<double> readDatFile(const std::string abspath, const Block block) {
    // Instead of reading each section of the file by its separator, we use blcoks of fixed width and height, ignoring lines beginnign with '%'.

    std::ifstream file(abspath);
    if (!file.is_open()) {
        throw std::runtime_error("File " + abspath + " not found.");
    }

    // Do not need width because we can find out the end of a line.
    const uint startAt = block * DEPTH_BLOCK_SIZE.height;
    const uint endAt = startAt + DEPTH_BLOCK_SIZE.height;

    // Line validation is simple: check if the line begins in a number. 
    std::string match = "^-?\\d+(\\.\\d+)?";
    boost::regex validLine(match.c_str());
    
    std::vector<double> lineset;
    std::string line;
    int lineIndex = 0;

    while (std::getline(file, line) && lineIndex < endAt) {

        // If the line does not start with a number or we are observering an unecessary line, skip this line.
        // The order of the conditions are significant. If the data is invalid we do not increment lineIndex. This magic is how we maintain a correct count the lines within the start and end line range regardless of where the 'comment' lines are.
        if (!boost::regex_search(line, validLine) || lineIndex++ < startAt) {
            continue;
        }

        // Split the line into an array of numbers. I choose double because there are decimal numbers in the file, even if the numbers we are actually reading are integers. Coding the data type dynamically has no benefit.
        std::stringstream in(line);
        double k;
        while(in >> k) {
            lineset.push_back(k);
        }

    }    
    file.close();    
    return lineset;
}


pcl::PointXYZRGB pointFromVectorsAndPixel(double x, double y, double z, cv::Vec3b rgb) {
    pcl::PointXYZRGB point;

    point.x = x;
    point.y = y;
    point.z = z;

    point.r = rgb[0];
    point.g = rgb[1];
    point.b = rgb[2];

    return point;
}


/**
 * Accepts a path to a .dat file and creates a point cloud, texturing it with a given cv::Mat.
 * @param abspath A file or folder path.
 * @param texture An image to colour the point cloud with.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPointCloudFile(const std::string abspath, const cv::Mat texture) {
    // This function unequally mixes file IO with processing something else. Could be bad practice, yes, but I couldn't enable image preprocessing with a string to the image path.

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Note I could reimplement this function to get the data as it is being read. However, readDatFile is not designed to read multiple parts of a file at once, so I would rather not.
    const std::vector<double> x = readDatFile(abspath, X);
    const std::vector<double> y = readDatFile(abspath, Y);
    const std::vector<double> z = readDatFile(abspath, Distance);

    const uint minlength = minOfThree(x.size(), y.size(), z.size());
    const int numPixels = texture.cols * texture.rows;

    // int width;
    // int height;

    // bool widthIsSet, heightIsSet = false;

    assert(minlength <= numPixels);

    for (uint i = 0; i < minlength; i++) {
        pcl::PointXYZRGB point = pointFromVectorsAndPixel(
            x.at(i),
            y.at(i),
            z.at(i),
            texture.at<cv::Vec3b>(i / numPixels, i % numPixels)
        );
        result->points.push_back(point);
    }

    // result->width = result->size();
    // result->height = 1;

    result->width = DEPTH_BLOCK_SIZE.width;
    result->height = DEPTH_BLOCK_SIZE.height;

    return result;

}


/**
 * Accepts a path to a .dat file and creates a point cloud, texturing it with a given cv::Mat.
 * @param abspath A file or folder path.
 * @param texture An image to colour the point cloud with.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPointCloudDir(const std::string absdir, cv::Mat texture) {
    // This function unequally mixes file IO with processing something else. Could be bad practice, yes, but I couldn't enable image preprocessing with a string to the image path.

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);

    const int numPixels = texture.cols * texture.rows;
    cv::cvtColor(texture, texture, cv::COLOR_BGR2RGB);

    // const char* matchString = std::string("depth_camera_\\d+\\.dat").c_str();
    // boost::regex searchThisDirectory(matchString);

    std::string matchString = "\\/depth_camera_\\d+\\.dat";
    boost::regex searchThisDirectory(matchString.c_str());
    
    int filesFound = 0;
    boostfs::path boostpath = boostfs::path(absdir);
    boostfs::directory_iterator itr(boostpath);
    for (; itr != boostfs::directory_iterator(); itr++) {
        std::string currentFile = itr->path().string();
        if (!boost::regex_search(currentFile, searchThisDirectory)) {
            continue;
        }

        const std::vector<double> x = readDatFile(currentFile, X);
        const std::vector<double> y = readDatFile(currentFile, Y);
        const std::vector<double> z = readDatFile(currentFile, Distance);

        const uint minlength = minOfThree(x.size(), y.size(), z.size());

        // Initialise the points vector and index the points within thereafter.
        if (filesFound > 0) {
            if (x.size() != result->points.size()) {
                throw std::runtime_error("Different number of X points across .dat files");
            }
            if (y.size() != result->points.size()) {
                throw std::runtime_error("Different number of Y points across .dat files");
            }
            if (z.size() != result->points.size()) {
                throw std::runtime_error("Different number of Distance points across .dat files");
            }

            for (int i = 0; i < minlength; i++) {
                // Multiply the last average to retrieve the sum, add on top of the sum, and divide it again + 1.
                result->points.at(i).x += x.at(i);
                result->points.at(i).y += y.at(i);
                result->points.at(i).z += z.at(i);

                // Rgb unecessary because there is nothing new to do.
            }            
                
        }        
        else {
            for (int i = 0; i < minlength; i++) {
                pcl::PointXYZRGB point = pointFromVectorsAndPixel(
                    x.at(i),
                    y.at(i),
                    z.at(i),
                    texture.at<cv::Vec3b>(i / numPixels, i % numPixels)
                );
                result->points.push_back(point);
            }
        }
        filesFound++;
    }

    if (filesFound == 0) {
        throw std::runtime_error("No depth file found in given path " + absdir);
    }
    else if (filesFound > 1) {
        for (int i = 0; i < result->points.size(); i++) {
            // Finally convert the sums to averages.
            result->points.at(i).x /= filesFound;
            result->points.at(i).y /= filesFound;
            result->points.at(i).z /= filesFound;
        }
    }

    // result->width = result->size();
    // result->height = 1;

    result->width = DEPTH_BLOCK_SIZE.width;
    result->height = DEPTH_BLOCK_SIZE.height;

    // result->width = DEPTH_BLOCK_SIZE.area();
    // result->height = 1;

    std::cout << result.get()->size() << std::endl;

    return result;
}


cv::Mat readAmplitudeImage(const std::string abspath) {
    const std::vector<double> amplitudes = readDatFile(abspath, Amplitude);
    cv::Mat result = cv::Mat(DEPTH_BLOCK_SIZE, CV_8UC1);
    const int numPixels = result.cols * result.rows;
    for (int i = 0; i < numPixels; i++) {
        int color = std::round(amplitudes[i] / 16382 * 255);
        result.at<uchar>(i / numPixels, i % numPixels) = color;
    }
    return result;
}


cv::Mat readAmplitudeImageDir(const std::string absdir) {
/*
    const std::vector<double> amplitudes = readDatFile(abspath, Amplitude);
    cv::Mat result = cv::Mat(DEPTH_BLOCK_SIZE, CV_8UC1);
    const int numPixels = result.cols * result.rows;
    for (int i = 0; i < numPixels; i++) {
        int color = std::round(amplitudes[i] / 16382 * 255);
        result.at<uchar>(i / numPixels, i % numPixels) = color;
    }
    return result;
*/
    cv::Mat result = cv::Mat(DEPTH_BLOCK_SIZE, CV_8UC1);
    std::vector<double> sumAmplitudes;

    const int numPixels = result.cols * result.rows;

    std::string matchString = "\\/depth_camera_\\d+\\.dat";
    boost::regex searchThisDirectory(matchString.c_str());

    int filesFound = 0;
    boostfs::path boostpath = boostfs::path(absdir);
    boostfs::directory_iterator itr(boostpath);
    for (; itr != boostfs::directory_iterator(); itr++) {
        std::string currentFile = itr->path().string();
        if (!boost::regex_search(currentFile, searchThisDirectory)) {
            continue;
        }
        const std::vector<double> amplitudes = readDatFile(currentFile, Amplitude);
        if (filesFound > 0) {
            if (amplitudes.size() != numPixels) {
                throw std::runtime_error("Unequal number of pixels in image.");
            }
            for (int i = 0; i < numPixels; i++) {
                sumAmplitudes[i] += amplitudes[i];
            }
        }
        else {
            sumAmplitudes = amplitudes;
        } 
        filesFound++;
    }

    if (filesFound == 0) {
        throw std::runtime_error("No depth file found in given path " + absdir);
    }

    for (int i = 0; i < sumAmplitudes.size(); i++) {
        // Finally convert the sums to averages.
        int calc = std::round(sumAmplitudes[i] / (16382 * filesFound) * 255);
        uchar colour = calc;
        result.at<uchar>(i / numPixels, i % numPixels) = colour;
    }
    return result;
}


/**
 * Calibrates a rgb image to a depth image given a path to a calibration folder.
 */
cv::Mat calibrateImage(std::string inputDir) {
    // const std::string debugimgpath = "/home/steven/Desktop/ulcerdatabase/patients/case_25/day_1/calib/scene_1";
    // inputDir = debugimgpath;
    std::string depthFile = "/depth_camera/depth_camera_1.dat";
    std::string photoFile = "/photo.jpg";

    cv::Mat amplitudeImage = readAmplitudeImageDir(inputDir + "/depth_camera/");
    cv::Mat photoImage = cv::imread(inputDir + photoFile);

    // cv::imshow("sdfsd", amplitudeImage);
    // cv::waitKey();

    cv::resize(amplitudeImage, amplitudeImage, DEPTH_BLOCK_SIZE*2);
    cv::resize(photoImage, photoImage, cv::Size(photoImage.cols/2, photoImage.rows/2));

    std::vector<cv::Point2f> depthcorners, rgbcorners;
    bool depthfound = cv::findChessboardCorners(amplitudeImage, cv::Size(8, 7), depthcorners);
    bool rgbfound = cv::findChessboardCorners(photoImage, cv::Size(8, 7), rgbcorners);


    if (!depthfound || !rgbfound) {
        throw std::runtime_error("Corners detection mismatch. depth: " + bolst(depthfound) + " rgb: " + bolst(rgbfound));
    }

    // Because we upscaled the images to find the corners better, we now must downscale the point coordinates back before continuing.
    if (depthcorners.size() == rgbcorners.size()) {
        for (int i = 0; i < depthcorners.size(); i++) {
            depthcorners.at(i).x /= 2;
            depthcorners.at(i).y /= 2;
            rgbcorners.at(i).x *= 2;
            rgbcorners.at(i).y *= 2;
        }
    }
    else {
        // Note I believe image registration inaccuracies are likely if we enter here.
        for (int i = 0; i < depthcorners.size(); i++) {
            depthcorners.at(i).x /= 2;
            depthcorners.at(i).y /= 2;
        }
        for (int i = 0; i < rgbcorners.size(); i++) {
            rgbcorners.at(i).x *= 2;
            rgbcorners.at(i).y *= 2;
        }
    }

    // We want to project rgb -> depth, so the order of the arguments are correct.
    cv::Mat homography = cv::findHomography(rgbcorners, depthcorners);
    // std::cout << homography << std::endl;
    return homography;
}


// int countFileLines(std::string abspath) {
//     std::ifstream file(abspath);
//     if (!file.is_open()) {
//         throw std::runtime_error("File " + abspath + " not found.");
//     }

//     int lines = 0;

//     // We do not actually need this variable other than to call the getline function.
//     std::string line;

//     while (std::getline(file, line)) {
//         lines++;
//     }

//     return lines;
// }

/**
 * Reads an entire text file. Calling this is likely slower than processing data within the same function.
 */
std::set<std::string> readFile(std::string abspath) {
    std::ifstream file(abspath);
    if (!file.is_open()) {
        throw std::runtime_error("File " + abspath + " not found.");
    }

    std::set<std::string> lines;
    std::string line;

    while (std::getline(file, line)) {
        lines.emplace(line);
    }

    return lines;
}



bool searchExportList(std::string caseDayDataScene) {

    if (!exportListLoaded) {
        exportList = readFile(EXPORT_LIST_PATH);
    }

    return exportList.find(caseDayDataScene) != exportList.end();
}



void writePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, std::string exportTo, std::string fname) {
    pcl::PLYWriter writer = pcl::PLYWriter();
    boostfs::create_directories(exportTo);
    writer.write(exportTo + fname, *pointcloud);    
}


int getStrIndex(std::string searchString, std::string searchFor, int nth) {
    boost::iterator_range<std::string::iterator> r = boost::find_nth(searchString, searchFor, nth);
    int index = std::distance(searchString.begin(), r.begin());
    return index;

}


std::string substrFromIndex(std::string searchString, std::string searchFor, int nth) {
    int index = getStrIndex(searchString, searchFor, nth);
    std::string s = searchString.substr(index);
    return s;
}


/**
 * Given a calibration folder, calibrates the depth and rgb images, then applies that calibration to the actual rgb image from the same scene in the data folder.
 * The result is exported to a specified base location mirroring the input directory structure.
 */
void calibrateAndExport(std::string inputDir, std::string outputBase) {
    
    cv::Mat homography;
    try {
        homography = calibrateImage(inputDir);
    }
    catch (std::runtime_error e) {
        std::cout << inputDir << std::endl;
        return;
    }

    // std::string matchString = "\\/depth_camera_\\d+\\.dat";
    std::string matchString = "\\/scene_\\d+";
    boost::regex searchThisDirectory(matchString.c_str());

    // Build some strings that will be used to save the point cloud.
    std::string pathTwoBack = boostfs::canonical(inputDir + "/../../").string();
    pathTwoBack.pop_back();
    const std::string caseAndDay = substrFromIndex(pathTwoBack, "/", -2);
    const std::string calibSceneName = substrFromIndex(inputDir, "/", -1);

    // const std::string caseName = caseAndDay.substr(0, getStrIndex(caseAndDay, "/", 1));
    // const int caseNum = std::stoi(
    //     caseName.substr(
    //         getStrIndex(caseName, "_", 0) + 1
    //     )
    // );    
    
    boostfs::path boostpath = boostfs::path(inputDir + "/../../data");
    boostfs::directory_iterator itr(boostpath);
    for (; itr != boostfs::directory_iterator(); itr++) {
        const std::string currentDir = itr->path().string();
        if (!boost::regex_search(currentDir, searchThisDirectory)) {
            continue;
        }

        const std::string dataSceneName = substrFromIndex(currentDir, "/", -1);
        const std::string outputTarget = outputBase + caseAndDay + dataSceneName;

        if (!searchExportList(caseAndDay + dataSceneName + calibSceneName)) {
            continue;
        }

        cv::Mat actualrgb = cv::imread(currentDir + "/photo.jpg");
        cv::cvtColor(actualrgb, actualrgb, cv::COLOR_BGR2RGB);

        cv::Mat warped;
        cv::warpPerspective(actualrgb, warped, homography, DEPTH_BLOCK_SIZE);
        cv::cvtColor(warped, warped, cv::COLOR_BGR2RGB);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = readPointCloudDir(currentDir + "/depth_camera/", warped);

        // The processed point clouds are scaled 1000 times. Do the same here.
        // The fourth one in a trs matrix expresses shear. We do not want to shear, so we create a block that targets the 1s before it.
        Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Identity();
        scaleMatrix.block<4,3>(0,0) *= 1000;
        pcl::transformPointCloud(*pointcloud, *pointcloud, scaleMatrix);

        writePointCloud(pointcloud, outputTarget, calibSceneName + ".ply");
        // cv::imshow("sldkfj", amplitudeImage);
        // cv::waitKey(0);

    }

}

/**
 * Verifies if a directory contains a matching sub-directory, as defined in `regexCriteria`, and if so, executes a specified function, for each matching sub-directory.
 * @param b An optional arbitrary string to pass to the function. It is the second argument of the function passed. The first argument is always the discovered directory.
*/
bool searchDirs(std::string basePath, std::deque<std::string> regexCriteria, void (*doWhenMet)(std::string, std::string), const std::string b = "") {
    // This function exists for a few reasons:
    //      1 - I needed to use boostfs::recursive_directory_iterator but I do not need the same amount of search depth.
    //      2 - Looping through directories and executing specific functions is a nice-to-have if I convert this into a terminal app
    //      3 - Minimise code re-use. if I need to search the (tedious) dataset folder, I can do so just with this function.

    bool customFunctionExecuted = false;

    std::string first = regexCriteria.front();
    boost::regex searchThisDirectory(first.c_str());

    boostfs::path boostpath = boostfs::path(basePath);
    boostfs::directory_iterator itr(boostpath);
    for (; itr != boostfs::directory_iterator(); itr++) {

        std::string currentPath = itr->path().string();
        
        // This area has all the criteria so execute the function
        // We stop at one criterion left because it is the last. Directory names can be treated exclusive here.
        // TODO write wrapper function that asserts > 1
        if (regexCriteria.size() == 1) {
            (*doWhenMet)(currentPath, b);
            customFunctionExecuted = true;
        }
        // Is this inside the root directory?
        // this is an else-if because it must be mutually exclusive from the first if.
        else if (boost::regex_search(currentPath, searchThisDirectory)) {
            // Remove the first criteria so we can check the rest in the next search deeper.
            regexCriteria.pop_front();
            
            // Recursion recursion recursion recursion
            searchDirs(currentPath, regexCriteria, doWhenMet, b);

            // Adding back the criteria may seem counterintuitive but it must be done in case there are multiple folders in this current path (remember we're also in a for loop).
            regexCriteria.push_front(first);
        }
        // If no, do not search deeper.
    }
    return customFunctionExecuted;
};


// void searchData(const std::string baseDir, const std::string outputBase) {

//     const std::vector<std::string> a = {"case_\\d*$", "day_\\d*$", "data$", "scene_\\d*$"};
//     std::deque<std::string> b;
//     for (std::string thing : a) {
//         b.push_back(thing);
//     }
//     searchDirs(baseDir, b, relativeCalib, outputBase);

// }

// void relativeCalib(std::string dataScene, std::string outputBase) {
//     const std::vector<std::string> a = {"scene_\\d*$"};
//     std::deque<std::string> b;
//     for (std::string thing : a) {
//         b.push_back(thing);
//     }
//     searchDirs(dataScene + "/../../data", b, calibrateAndExport, outputBase);
// }




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

    // Top two vars are used for example clouds/images
    const std::string debugpcpath = "/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/calib/scene_1/depth_camera/depth_camera_1.dat";
    const std::string debugimgpath = "/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/calib/scene_1/photo.jpg";
    
    const std::string debugbasedir = "/home/steven/Desktop/ulcerdatabase/patients";
    const std::string debugexportbase = "/home/steven/Desktop/alignedwounds";
    const bool debuguseregistration = true;

    if (debuguseregistration) {
        
        std::vector<std::string> a = {"case_\\d*$", "day_\\d*$", "calib$", "scene_\\d*$"};
        std::deque<std::string> b;
        for (std::string thing : a) {
            b.push_back(thing);
        }
        searchDirs(debugbasedir, b, calibrateAndExport, debugexportbase);

    }

    

    //Create the point cloud
    // cv::Mat img = cv::imread(debugimgpath);
    // cv::resize(img, img, DEPTH_BLOCK_SIZE);
    // // std::unordered_map<std::string, std::vector<std::vector<std::string>>> depthdata = readDatFile(debugpcpath, "% ");
    // // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = pointCloudFromData(depthdata, img);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = readPointCloudFile(debugpcpath, img);

    // //Create the visualiser and render it.
    // pcl::visualization::PCLVisualizer::Ptr viewer = visualizePointCloud(pointcloud);
    // while (!viewer->wasStopped()) {
    //     viewer->spin();
    //     std::this_thread::sleep_for(100ms);
    // }

    // cv::imshow("sldkfj", depthimage);
    // cv::imshow("sldkfj", dst_norm_scaled);
    // cv::waitKey(0);

    return 1;
}

