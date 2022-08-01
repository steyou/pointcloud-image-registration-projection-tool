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

// #include "libcbdetect/boards_from_corners.h"
// #include "libcbdetect/config.h"
// #include "libcbdetect/find_corners.h"
// #include "libcbdetect/plot_boards.h"
// #include "libcbdetect/plot_corners.h"

// namespace fs = std::filesystem;
namespace boostfs = boost::filesystem;
namespace chrono = std::chrono;

using std::chrono_literals::operator""ms;

const cv::Size DEPTH_BLOCK_SIZE = cv::Size(176, 144);
// std::string inputDir = "";

enum Block { Distance, X, Y, Amplitude, Confidence };

// template <typename Out>
// void split(const std::string &s, char delim, Out result) {
//     std::istringstream iss(s);
//     std::string item;
//     while (std::getline(iss, item, delim)) {
//         *result++ = item;
//     }
// }


// std::vector<std::string> split(const std::string &s, char delim) {
//     std::vector<std::string> elems;
//     split(s, delim, std::back_inserter(elems));
//     return elems;
// }

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


// /**
//  * Returns the contents of a point cloud .dat file
//  * @param abspath The absolute path of the file you want.
//  * @param sep A common string used before the group identifier string. Used to identify the beginning of a new group.
//  */
// std::unordered_map<std::string, std::vector<std::vector<std::string>>> readDatFile(const std::string abspath, const std::string sep) {

//     std::ifstream file(abspath);

//     std::vector<std::vector<std::string>> lineset;
//     std::unordered_map<std::string, std::vector<std::vector<std::string>>> finalMap;

//     std::string currentKey;
//     bool nosep = true;

//     if (file.is_open()) {
//         std::string line;
//         while (std::getline(file, line)) {

//             //if the current line begins with a separator...
//             if (line.rfind(sep, 0) == 0) {
                
//                 //add to the map if there were lines before this separator.
//                 if (lineset.size() > 0) {
//                     finalMap[currentKey] = lineset;
//                     lineset.clear();
//                 }
                
//                 //Setting a new key indicates to add data under a new 'group'. We use the separator (current line) to index this group.
//                 //The line has regex characters and spaces so they need to be removed before the line is usable enough as an index.
//                 currentKey = line.substr(line.find(sep) + sep.length());
//                 currentKey = currentKey.erase(currentKey.find("\r"));

//                 nosep = false;
//                 //now continue to next line in the file!
//             }
//             else if (!nosep) {
//                 //Nothing else to do except add the current line to the lineset. It is under the current separator.
//                 //Also separate all the numbers into their own array values. I don't like this because it is potentially more memory consuming but it is more programmatically convenient.
//                 line = line.erase(line.find("\r"));
//                 lineset.push_back(
//                     split(line, '\t')
//                 );
//             }
//         }

//         //After iterating the last line we must add the lineset to the map manually because the loop does not do this.
//         if (!currentKey.empty() && lineset.size() > 0 && !nosep) {
//             finalMap[currentKey] = lineset;
//             //lineset.clear() is redundant because garbage collection is about to happen.
//         }
        
//         file.close();
//     }
//     else {
//         throw std::runtime_error("File " + abspath + " not found.");
//     }

//     //Did we reach the end without a sep? If yes, it is a bad file. It is not this function's responsibility to fix it so we throw an exception.
//     if (nosep) {
//         throw std::runtime_error("No sep found.");
//     }

//     return finalMap;
// }

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
        if (boost::regex_search(currentFile, searchThisDirectory)) {

            const std::vector<double> x = readDatFile(currentFile, X);
            const std::vector<double> y = readDatFile(currentFile, Y);
            const std::vector<double> z = readDatFile(currentFile, Distance);

            const uint minlength = minOfThree(x.size(), y.size(), z.size());

            // Initialise the points vector and index the points within thereafter.
            if (filesFound == 0) {
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
            else {
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

            filesFound++;

        }
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


/**
 * Creates a point cloud from the data points
 */
// pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFromData(const std::unordered_map<std::string, std::vector<std::vector<std::string>>> groups) {
    
//     //These are references to the xyz headers in the original file.
//     const std::string xCoordSep = "Calibrated xVector";
//     const std::string yCoordSep = "Calibrated yVector";
//     const std::string zCoordSep = "Calibrated Distance";

//     //Are there an equal amount of xyz points? Every valid point must have a xyz value.
//     //x implies z via y so an explicit check is unecessary.
//     if (
//         groups.at(xCoordSep).size() != groups.at(yCoordSep).size() ||
//         groups.at(yCoordSep).size() != groups.at(zCoordSep).size() ||
//         groups.at(xCoordSep).at(0).size() != groups.at(yCoordSep).at(0).size() ||
//         groups.at(yCoordSep).at(0).size() != groups.at(zCoordSep).at(0).size()
//     ) {
//         throw new std::runtime_error("Unequal XYZ point dimensions.");
//     }

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//     for (int y = 0; y < groups.at(xCoordSep).size(); y++) {
//         for (int x = 0; x < groups.at(xCoordSep).at(0).size(); x++) {
//             pcl::PointXYZ point;
//             point.x = std::stod(groups.at(xCoordSep).at(y).at(x));
//             point.y = std::stod(groups.at(yCoordSep).at(y).at(x));
//             point.z = std::stod(groups.at(zCoordSep).at(y).at(x));
//         }
//     }

//     cloud->width = cloud->size();
//     cloud->height = 1;

//     return cloud;
    
// }


// pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudFromData(const std::unordered_map<std::string, std::vector<std::vector<std::string>>> groups, cv::Mat texture) {
    
//     //These are references to the xyz headers in the original file.
//     const std::string xCoordSep = "Calibrated xVector";
//     const std::string yCoordSep = "Calibrated yVector";
//     const std::string zCoordSep = "Calibrated Distance";

//     //Are there an equal amount of xyz points? Every valid point must have a xyz value.
//     //x implies z via y so an explicit check for equality with z and the others is unecessary.
//     // if (
//     //     groups.at(xCoordSep).size() != groups.at(yCoordSep).size() ||
//     //     groups.at(yCoordSep).size() != groups.at(zCoordSep).size() ||
//     //     groups.at(xCoordSep).at(0).size() != groups.at(yCoordSep).at(0).size() ||
//     //     groups.at(yCoordSep).at(0).size() != groups.at(zCoordSep).at(0).size()
//     // ) {
//     //     throw new std::runtime_error("Unequal XYZ point dimensions.");
//     // }

//     auto yx = groups.at(xCoordSep).size();
//     auto yy = groups.at(yCoordSep).size();
//     auto yz = groups.at(zCoordSep).size();

//     auto xx = groups.at(xCoordSep).at(0).size();
//     auto xy = groups.at(yCoordSep).at(0).size();
//     auto xz = groups.at(zCoordSep).at(0).size();

//     uint yMax = minOfThree(yx, yy, yz);;
//     uint xMax = minOfThree(xx, xy, xz);;

//     // If the above lines cause the point cloud to be cropped (as a result of dimensions not matching), the image we are about to project must be cropped too.
//     // cv::Rect cropArea = cv::Rect(0, 0, xMax, yMax);
//     // cv::Mat croppedTexture = texture(cropArea);

//     cv::cvtColor(texture, texture, cv::COLOR_BGR2RGB);

//     // cv::imshow("skldjflsd", croppedTexture);
//     // cv::waitKey();
//     // exit(0);
    
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

//     for (int y = 0; y < yMax; y++) {
//         for (int x = 0; x < xMax; x++) {
//             pcl::PointXYZRGB point;
//             point.x = std::stod(groups.at(xCoordSep).at(y).at(x));
//             point.y = std::stod(groups.at(yCoordSep).at(y).at(x));
//             point.z = std::stod(groups.at(zCoordSep).at(y).at(x));
//             point.r = texture.at<cv::Vec3b>(y, x)[0];
//             point.g = texture.at<cv::Vec3b>(y, x)[1];
//             point.b = texture.at<cv::Vec3b>(y, x)[2];
            
//             cloud->points.push_back(point);
//         }
//     }

//     cloud->width = cloud->size();
//     cloud->height = 1;

//     return cloud;
// }


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
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (0.01);
    viewer->initCameraParameters ();
    return (viewer);
}


// cv::Mat depthImageFromAmplitudes(std::vector<std::vector<std::string>> amplitudes) {
//     // cv::Mat result = cv::Mat::zeros(cv::Size(amplitudes.size(), amplitudes.at(0).size()));
//     cv::Mat result = cv::Mat(cv::Size(amplitudes.at(0).size(), amplitudes.size()), CV_8UC1);
//     uchar pixValue;
//     int i = 0;
//     for (i; i < result.cols; i++) {
//         for (int j = 0; j < result.rows; j++) {
//             cv::Vec3b &intensity = result.at<cv::Vec3b>(j, i);
//             //note this variable could be typed uint8_t because all possible values range 0-255. I wont do this because I have no idea if that introduces any problems.
//             int color = std::round(std::stod(amplitudes.at(j).at(i)) / 16382 * 255);
//             result.at<uchar>(j,i) = color;
            
//         }
//     }

//     return result;
// }

// std::vector<int> extract_corners(
//     const std::vector<cv::Mat>& images,
//     std::vector<std::vector<cv::Point2d>>& image_points,
//     std::vector<std::vector<cv::Point2d>>& world_points,
//     int num_boards,
//     int pattern_size,
//     const std::vector<cbdetect::Params>& params) {

//     std::vector<int> res(images.size(), 0);
//     // params.camera1.detect_params.overlay = true;
//     for(int i = 0; i < images.size(); ++i) {
//         cbdetect::Corner chessboad_corners;
//         std::vector<cbdetect::Board> chessboards;

//         cbdetect::find_corners(images[i], chessboad_corners, params[i]);
//         cbdetect::boards_from_corners(images[i], chessboad_corners, chessboards, params[i]);
//         if(chessboards.empty()) {
//             continue;
//         }
//         // if(params.camera1.show_cornres) {
//         //     cbdetect::plot_boards(images[i], chessboad_corners, chessboards, params.camera1.detect_params);
//         // }

//         std::sort(chessboards.begin(), chessboards.end(),
//             [](const cbdetect::Board& b1, const cbdetect::Board& b2) {
//             return b1.num > b2.num;
//         });

//         res[i] = std::min(num_boards, (int)chessboards.size());
//         for(int j = 0; j < res[i]; ++j) {
//             std::vector<std::vector<int>>& chessboard = chessboards[j].idx;
//             std::vector<cv::Point2d> pi_buf, pw_buf;

//             for(int jj = 0; jj < chessboard.size(); ++jj) {
//                 for(int ii = 0; ii < chessboard[0].size(); ++ii) {
//                     if(chessboard[jj][ii] < 0) continue;
//                     pi_buf.emplace_back(chessboad_corners.p[chessboard[jj][ii]]);
//                     pw_buf.emplace_back(cv::Point2d(pattern_size * (ii - 1), pattern_size * (jj - 1)));
//                 }
//             }
//             image_points.emplace_back(pi_buf);
//             world_points.emplace_back(pw_buf);
//         }
//     }

//   return res;

// }

// /**
//  * 
//  */
// void detectChessCorners(cv::Mat img, cbdetect::Params cbParams) {

//     cbdetect::Corner corners;
//     std::vector<cbdetect::Board> boards;

//     // auto t1 = chrono::high_resolution_clock::now();
//     cbdetect::find_corners(img, corners, cbParams);
//     // auto t2 = chrono::high_resolution_clock::now();
//     cbdetect::plot_corners(img, corners);
//     // auto t3 = chrono::high_resolution_clock::now();
//     cbdetect::boards_from_corners(img, corners, boards, cbParams);
//     // auto t4 = chrono::high_resolution_clock::now();

//     // printf("Find corners took: %.3f ms\n", chrono::duration_cast<chrono::microseconds>(t2 - t1).count() / 1000.0);
//     // printf("Find boards took: %.3f ms\n", chrono::duration_cast<chrono::microseconds>(t4 - t3).count() / 1000.0);
//     // printf("Total took: %.3f ms\n", chrono::duration_cast<chrono::microseconds>(t2 - t1).count() / 1000.0 + chrono::duration_cast<chrono::microseconds>(t4 - t3).count() / 1000.0);
//     cbdetect::plot_boards(img, corners, boards, cbParams);
// }

// void detectChessCorners() {

//     std::cout << inputDir << std::endl;
//     // const std::string debugimgpath = "/home/steven/Desktop/ulcerdatabase/patients/case_25/day_1/calib/scene_1";
//     // inputDir = debugimgpath;
//     std::string depthFile = "/depth_camera/depth_camera_1.dat";
//     std::unordered_map<std::string, std::vector<std::vector<std::string>>> depthdata;
//     bool nullness = true;
//     try {
//         depthdata = readDatFile(inputDir + depthFile, "% ");
//         nullness = false;
//     }
//     catch(const std::runtime_error& e) {
//         std::cout << e.what() << std::endl;
//     }
//     if (!nullness) {
//         std::string photoFile = "/photo.jpg";
        
//         cv::Mat amplitudeImage = depthImageFromAmplitudes(depthdata.at("Amplitude"));
//         cv::Mat photoImage = cv::imread(inputDir + photoFile);
//         // cv::resize(photoImage, photoImage, cv::Size(photoImage.cols/2, photoImage.rows/2));

//         // cv::imshow("sldkfj", amplitudeImage);
//         // cv::waitKey(0);

//         // cv::equalizeHist(amplitudeImage, amplitudeImage);
//         cv::resize(amplitudeImage, amplitudeImage, cv::Size(amplitudeImage.cols*2, amplitudeImage.rows*2));
//         // cv::medianBlur(amplitudeImage, amplitudeImage, 5);

//         // cv::Mat sharpeningKernel = (
//         //     cv::Mat_<double>(3, 3) <<
//         //     -1, -1, -1,
//         //     -1, 9, -1,
//         //     -1, -1, -1
//         // );
//         // cv::filter2D(amplitudeImage, amplitudeImage, -1, sharpeningKernel);

//         // cv::threshold(amplitudeImage, amplitudeImage, 190, 90, cv::THRESH_TRIANGLE);
//         // cv::imshow("sdfssd", amplitudeImage);
//         // cv::waitKey();

//         cbdetect::Params amplitudeParams;
//         amplitudeParams.corner_type = cbdetect::SaddlePoint;
//         // amplitudeParams.init_loc_thr = 0.001;
//         // amplitudeParams.score_thr = 0.001;
//         amplitudeParams.radius = {1,3};

//         cbdetect::Params photoParams;
//         photoParams.corner_type = cbdetect::SaddlePoint;
//         // photoParams.radius = {1,3};

//         // detectChessCorners(amplitudeImage, amplitudeParams);

//         std::vector<cv::Mat> images = {amplitudeImage, photoImage};
//         std::vector<cbdetect::Params> params = {amplitudeParams, photoParams};

//         std::vector<std::vector<cv::Point2d>> image_points;
//         std::vector<std::vector<cv::Point2d>> world_points;

//         extract_corners(images, image_points, world_points, 1, 24, params);

//         // cv::Mat homography;
//         // cv::findHomography(world_points[0], image_points[0], homography);

//         std::vector<cv::Mat> homo_vec;
//         Eigen::MatrixXd mat_v = Eigen::MatrixXd::Zero(2 * world_points.size(), 6);
//         for(int i = 0; i < world_points.size(); ++i) {
//             cv::Mat homo = cv::findHomography(world_points[i], image_points[i], cv::RANSAC);
//             auto homo_at = [&homo](int x, int y) { return homo.at<double>(y - 1, x - 1); };
//             mat_v.block(2 * i, 0, 1, 6) << homo_at(1, 1) * homo_at(2, 1),
//                 homo_at(1, 1) * homo_at(2, 2) + homo_at(1, 2) * homo_at(2, 1),
//                 homo_at(1, 2) * homo_at(2, 2),
//                 homo_at(1, 3) * homo_at(2, 1) + homo_at(1, 1) * homo_at(2, 3),
//                 homo_at(1, 3) * homo_at(2, 2) + homo_at(1, 2) * homo_at(2, 3),
//                 homo_at(1, 3) * homo_at(2, 3);
//             mat_v.block(2 * i + 1, 0, 1, 6) << homo_at(1, 1) * homo_at(1, 1) - homo_at(2, 1) * homo_at(2, 1),
//                 homo_at(1, 1) * homo_at(1, 2) + homo_at(1, 2) * homo_at(1, 1) -
//                     homo_at(2, 1) * homo_at(2, 2) - homo_at(2, 2) * homo_at(2, 1),
//                 homo_at(1, 2) * homo_at(1, 2) - homo_at(2, 2) * homo_at(2, 2),
//                 homo_at(1, 3) * homo_at(1, 1) + homo_at(1, 1) * homo_at(1, 3) -
//                     homo_at(2, 3) * homo_at(2, 1) - homo_at(2, 1) * homo_at(2, 3),
//                 homo_at(1, 3) * homo_at(1, 2) + homo_at(1, 2) * homo_at(1, 3) -
//                     homo_at(2, 3) * homo_at(2, 2) - homo_at(2, 2) * homo_at(2, 3),
//                 homo_at(1, 3) * homo_at(1, 3) - homo_at(2, 3) * homo_at(2, 3);
//             homo_vec.emplace_back(homo);
//         }

//         // std::string scene_num(
//         //     inputDir[inputDir.length()-1],
//         //     1
//         // );
//         std::string scene_num = inputDir.substr(inputDir.length()-1);
//         std::string actualrgbfile = "/../../data/scene_" + scene_num + "/photo.jpg";
//         // std::string actualrgbfile = "/../../data/scene_1/photo.jpg";
//         cv::Mat actualrgb = cv::imread(inputDir + actualrgbfile);
        
//         cv::Mat warped;

//         bool arewegood = false;
//         try {

//             cv::warpPerspective(actualrgb, warped, homo_vec[0], actualrgb.size());
//             arewegood = true;


//         }
//         catch (cv::Exception e) { }
//         if (arewegood) {
//             // cv::resize(warped, warped, cv::Size(warped.cols/8, warped.rows/8));
//             // cv::imshow("sdfssd", warped);
//             // cv::waitKey();

//             cv::Rect r2 = cv::Rect(0, 0, amplitudeImage.cols, amplitudeImage.rows);
//             warped = warped(r2);

//             pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = pointCloudFromData(depthdata, warped);

//             pcl::PLYWriter writer = pcl::PLYWriter();

//             const std::string debugbasedir = "/home/steven/Desktop/ulcerdatabase/patients";
//             const std::string debugexportbase = "/home/steven/Desktop/badwounds";
//             std::string currentTarget = inputDir.substr(
//                 inputDir.find(debugbasedir) + debugbasedir.length()
//             );
//             std::string fullExportPath = debugexportbase + currentTarget;
//             std::cout << fullExportPath << std::endl;

//             boostfs::create_directories(fullExportPath);
//             writer.write(fullExportPath + "/rgb.ply", *pointcloud);

//         }
//     }

// }

void detectChessCorners(std::string inputDir, std::string outputBase) {
    // const std::string debugimgpath = "/home/steven/Desktop/ulcerdatabase/patients/case_25/day_1/calib/scene_1";
    // inputDir = debugimgpath;
    std::string depthFile = "/depth_camera/depth_camera_1.dat";
    std::string photoFile = "/photo.jpg";

    cv::Mat amplitudeImage = readAmplitudeImage(inputDir + depthFile);
    cv::Mat photoImage = cv::imread(inputDir + photoFile);

    cv::resize(amplitudeImage, amplitudeImage, DEPTH_BLOCK_SIZE*2);
    cv::resize(photoImage, photoImage, cv::Size(photoImage.cols/2, photoImage.rows/2));

    std::vector<cv::Point2f> depthcorners, rgbcorners;
    bool depthfound = cv::findChessboardCorners(amplitudeImage, cv::Size(8, 7), depthcorners);
    bool rgbfound = cv::findChessboardCorners(photoImage, cv::Size(8, 7), rgbcorners);


    if (!depthfound || !rgbfound) {
        return;
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

    std::string actualrgbfile = "/../../data/scene_" + inputDir.substr(inputDir.length()-1) + "/photo.jpg";
    if (!boostfs::exists(inputDir + actualrgbfile)) {
        std::cout << inputDir + actualrgbfile << std::endl;
        return;
    }
    cv::Mat actualrgb = cv::imread(inputDir + actualrgbfile);
    cv::cvtColor(actualrgb, actualrgb, cv::COLOR_BGR2RGB);

    cv::Mat warped;
    cv::warpPerspective(actualrgb, warped, homography, DEPTH_BLOCK_SIZE);
    cv::cvtColor(warped, warped, cv::COLOR_BGR2RGB);

    std::string actualpointcloudfile = "/../../data/scene_" + inputDir.substr(inputDir.length()-1) + "/depth_camera";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = readPointCloudDir(inputDir + actualpointcloudfile, warped);

    // The processed point clouds are scaled 1000 times. Do the same here.
    // The fourth one in a trs matrix expresses shear. We do not want to shear, so we create a block that targets the 1s before it.
    Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Identity();
    scaleMatrix.block<4,3>(0,0) *= 1000;
    pcl::transformPointCloud(*pointcloud, *pointcloud, scaleMatrix);

    pcl::PLYWriter writer = pcl::PLYWriter();

    // TODO write code that finds the intersection between the inputDir and the outputBase to calculate debugbase dynamically
    const std::string debugbasedir = "/home/steven/Desktop/ulcerdatabase/patients";
    std::string currentTarget = inputDir.substr(
        inputDir.find(debugbasedir) + debugbasedir.length()
    );
    std::string fullExportPath = outputBase + currentTarget;
    // std::cout << fullExportPath << std::endl;

    boostfs::create_directories(fullExportPath);
    writer.write(fullExportPath + "/rgb.ply", *pointcloud);    

    // cv::imshow("sldkfj", amplitudeImage);
    // cv::waitKey(0);
        
}

/**
 * Verifies if a directory contains a matching sub-directory, as defined in `regexCriteria`, and if so, executes an arbitrary function, for each matching sub-directory.
 * @param b An optional arbitrary string to pass to the function. It is the second argument of the function. The first argument is always the discovered directory.
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
            // inputDir = currentPath;
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

// /**
//  * Given the path to a wound capture scene, takes a point cloud .dat and rgb .jpg and saves it somewhere.
// */
// void exportScenePlyRgb() {
//     // std::cout << inputDir << std::endl;
    
//     // This is a relative path
//     std::string depthFile = "/depth_camera/depth_camera_1.dat";
//     std::unordered_map<std::string, std::vector<std::vector<std::string>>> depthdata;
//     bool nullness = true;
//     try {
//         depthdata = readDatFile(inputDir + depthFile, "% ");
//         nullness = false;
//     }
//     //Empty catch may seem dubious but the only time exceptions should only happen when the file is wrong. In that case there is nothing the program can do but move on.
//     catch(const std::runtime_error& e) {
//         // std::cout << e.what() << std::endl;
//     }                
//     if (!nullness) {
//         // From here we can do multiple things. We can retrieve the amplitude data for every depth image. Or we can export the dataset with ground truths.

//         // std::vector<std::vector<std::string>> amplitudes = depthdata.at("Amplitude");
//         // cv::Mat depthimage = depthImageFromAmplitudes(amplitudes);

//         // Load the images
//         cv::Mat rgbImage = cv::imread(inputDir + "/photo.jpg");
//         cv::Mat maskImage = cv::imread(inputDir + "/mask.png");

//         cv::resize(rgbImage, rgbImage, DEPTH_BLOCK_SIZE);
//         cv::resize(maskImage, maskImage, DEPTH_BLOCK_SIZE);

//         cv::cvtColor(rgbImage, rgbImage, cv::COLOR_BGR2RGB);
//         cv::cvtColor(maskImage, maskImage, cv::COLOR_BGR2RGB);
//         // cv::cvtColor(maskImage, maskImage, cv::COLOR_BGR2GRAY);

//         // cv::imshow("lorem", maskImage);
//         // cv::waitKey();
//         // exit(0);

//         // Create the point clouds from the depth data and images.
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud = pointCloudFromData(depthdata, rgbImage);
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr maskCloud = pointCloudFromData(depthdata, maskImage);

//         // The processed point clouds are scaled 1000 times. Do the same here.
//         Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Identity();
//         scaleMatrix.block<4,3>(0,0) *= 1000;

//         pcl::transformPointCloud(*rgbCloud, *rgbCloud, scaleMatrix);
//         pcl::transformPointCloud(*maskCloud, *maskCloud, scaleMatrix);


//         //Create the visualiser and render it.
//         // pcl::visualization::PCLVisualizer::Ptr viewer = visualizePointCloud(rgbCloud);
//         // while (!viewer->wasStopped()) {
//         //     viewer->spin();
//         //     std::this_thread::sleep_for(100ms);
//         // }
//         // exit(0);

//         // This removes the base directory from the current path so we can use the export base path instead. A strange solution, yes, but it works.
//         // TODO Separate the base dir more elegantly. Currently it is built into the directory iterators so it isn't so straightforward.
//         const std::string debugbasedir = "/home/steven/Desktop/ulcerdatabase/patients";
//         const std::string debugexportbase = "/home/steven/Desktop/rgbwounds";
//         std::string currentTarget = inputDir.substr(
//             inputDir.find(debugbasedir) + debugbasedir.length()
//         );
//         std::string fullExportPath = debugexportbase + currentTarget;
//         // std::cout << fullExportPath << std::endl;

//         pcl::PLYWriter writer = pcl::PLYWriter();
//         boostfs::create_directories(fullExportPath);

//         writer.write(fullExportPath + "/rgb.ply", *rgbCloud);
//         writer.write(fullExportPath + "/mask.ply", *maskCloud);

//         // It seems you do not need to close() the writer to prevent memory leaks.
//     }
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


    // const std::string debugpcpath = "/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/calib/scene_1/depth_camera/depth_camera_1.dat";
    
    // Top two vars are used for example clouds/images
    const std::string debugpcpath = "/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/calib/scene_1/depth_camera/depth_camera_1.dat";
    const std::string debugimgpath = "/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/calib/scene_1/photo.jpg";
    
    const std::string debugbasedir = "/home/steven/Desktop/ulcerdatabase/patients";
    const std::string debugexportbase = "/home/steven/Desktop/alignedwounds";
    const bool debuguseregistration = true;

    // Fill a deque (I do it this way for the curly brace syntactic sugar that deques do not support)
    // std::vector<std::string> a = {"case_\\d*$", "day_\\d*$", "data$", "scene_\\d*$"};
    // std::deque<std::string> b;
    // for (std::string thing : a) {
    //     b.push_back(thing);
    // }
    // searchDirs(debugbasedir, b, exportScenePlyRgb);
    
    if (debuguseregistration) {
        
        std::vector<std::string> a = {"case_\\d*$", "day_\\d*$", "calib$", "scene_\\d*$"};
        std::deque<std::string> b;
        for (std::string thing : a) {
            b.push_back(thing);
        }
        searchDirs(debugbasedir, b, detectChessCorners, debugexportbase);

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

