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
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// namespace fs = std::filesystem;

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
            
            cv::Vec3b texrgb = texture.at<cv::Vec3b>(cv::Point(x,y));
            
            point.r = texrgb[2];
            point.g = texrgb[1];
            point.b = texrgb[0];
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


pcl::visualization::PCLVisualizer::Ptr visualizePointCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (0.01);
    viewer->initCameraParameters();
    return (viewer);
}


pcl::visualization::PCLVisualizer::Ptr visualizePointCloud (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0.1, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (0.01);
    viewer->initCameraParameters ();
    return (viewer);
}


cv::Mat createDepthImageFromAmplitudes(std::vector<std::vector<std::string>> amplitudes) {
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
    const std::string debugpcpath = "/home/steven/Desktop/ulcerdatabase/patients/case_2/day_1/calib/scene_1/depth_camera/depth_camera_1.dat";
    const std::string debugimgpath = "/home/steven/Desktop/ulcerdatabase/patients/case_1/day_1/calib/scene_1/depth_camera/depth_map_1.png";
    
    const std::string debugbaserecurse = "/home/steven/Desktop/ulcerdatabase/patients";
    const std::string debugtargetrecurse = "/calib/scene_1/depth_camera/";
    
    //"In UNIX everything is a file". Count the number of folders.
    // int casefolders = countFiles(debugbaserecurse);

    // std::string debugpcpath = "";
    std::unordered_map<std::string, std::vector<std::vector<std::string>>> asd = readDatFile(debugpcpath, "% ");
    
    //Note: accessing the folders this way does not respect alphanumeric order. I think this is fortunately irrelevant for what I am trying to do.
    // for (const auto & patient : fs::directory_iterator(debugbaserecurse)) {
    //     for (const auto & day : fs::directory_iterator(patient.path())) {
    //         std::string currentpath = (std::string)day.path() + debugtargetrecurse;
    //         std::string pathwithfile = currentpath + "depth_camera_1.dat"; 
    //         // int depthcameracount = countFiles(currentpath);

    //         std::unordered_map<std::string, std::vector<std::vector<std::string>>> depthdata;
    //         bool nullness = true;
    //         try {
    //             depthdata = readDatFile(pathwithfile, "% ");
    //             nullness = false;
    //         }
    //         //Empty catch may seem dubious but the only time exceptions should only happen when the file is wrong. In that case there is nothing the program can do but move on.
    //         catch(const std::runtime_error& e) {/*nothing*/}                
    //         if (!nullness) {
                cv::Mat depthimage = createDepthImageFromAmplitudes(asd.at("Amplitude"));
                // cv::Mat depthimage = createDepthImageFromAmplitudes(depthdata.at("Amplitude"));
    //             // cv::namedWindow("sldkfjsfd", cv::WINDOW_NORMAL);
    //             // cv::resize(depthimage, depthimage, cv::Size(720,880), 0, 0, cv::INTER_NEAREST);
    //             std::cout << pathwithfile << std::endl;
    //             cv::imshow("cvwindow", depthimage);
    //             cv::waitKey(0);
    //             cv::imwrite(currentpath + "depth_map_1.png", depthimage);
    //         }
    //     }
    // }
    
    // std::unordered_map<std::string, std::vector<std::vector<std::string>>> depthdata = readDatFile(debugpcpath, "% ");

    //Get the data
    // cv::Mat image = cv::imread(debugimgpath, 1);

    // cv::Mat im2;
    // cv::resize(image, image, cv::Size(176, 144), cv::INTER_LINEAR);
    // cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    // cv::resize(image, image, cv::Size(720, 880), 0, 0, cv::INTER_NEAREST);


    cv::Mat rgbimage = cv::imread("/home/steven/Desktop/ulcerdatabase/patients/case_2/day_1/calib/scene_1/photo.jpg", cv::IMREAD_GRAYSCALE);
    std::cout << rgbimage.size << std::endl;
    // cv::cvtColor(rgbimage,rgbimage,cv::COLOR_BGR2GRAY);
    
    // cv::resize(depthimage, depthimage, rgbimage.size(), cv::INTER_LINEAR);
    // cv::resize(rgbimage, rgbimage, cv::Size(1920,1080), cv::INTER_LINEAR);
    // cv::resize(rgbimage, rgbimage, cv::Size(rgbimage.cols/2,rgbimage.rows/2), cv::INTER_LINEAR);
    std::cout << rgbimage.size << std::endl;

    // cv::rectangle(rgbimage, cv::Point(200,200), cv::Point(800,800), cv::Scalar(255,0,0));

    // //Centre crop the image.
    // const float cropscalar = 1.0;
    // cv::Rect r = cv::Rect(
    //     (float)rgbimage.cols/2.0-(float)depthimage.cols*cropscalar/2.0,
    //     (float)rgbimage.rows/2.0-(float)depthimage.rows*cropscalar/2.0,
    //     depthimage.cols*cropscalar,
    //     depthimage.rows*cropscalar
    // );

    // std::cout << (float)r.width/(float)r.height << std::endl;
    // std::cout << (float)depthimage.cols/(float)depthimage.rows << std::endl;
    
    // rgbimage = rgbimage(r);
    // cv::rectangle(rgbimage,r,cv::Scalar(255,0,0),10);
    // cv::resize(rgbimage, rgbimage, depthimage.size(), cv::INTER_LINEAR);

    // cv::imshow("sldkfj", rgbimage);
    // cv::waitKey(0);

    // Registration code from https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html
    std::vector<cv::Point2f> depthcorners, rgbcorners;
    bool depthfound = cv::findChessboardCorners(depthimage, cv::Size(8, 7), depthcorners);
    bool rgbfound = cv::findChessboardCorners(rgbimage, cv::Size(8, 7), rgbcorners);
    
    if (depthfound && rgbfound) {
        cv::Mat homography = cv::findHomography(rgbcorners,depthcorners);
        std::cout << homography << std::endl;

        cv::Mat actualrgb = cv::imread("/home/steven/Desktop/ulcerdatabase/patients/case_2/day_1/data/scene_1/photo.jpg");
        cv::cvtColor(actualrgb, actualrgb, cv::COLOR_BGR2RGB);
        
        cv::Mat warped;
        cv::warpPerspective(actualrgb, warped, homography, actualrgb.size());

        // cv::imshow("", warped);
        // cv::waitKey(0);

        //If you imshow the warped image you will see that there are unneeded pixels beyond the borders of the depth camera. These two lines crops the image back to the borders.
        cv::Rect r2 = cv::Rect(0,0,depthimage.cols,depthimage.rows);
        warped = warped(r2);
        
        // cv::cvtColor(warped,warped,cv::COLOR_GRAY2BGR);
        cv::cvtColor(warped,warped,cv::COLOR_BGR2RGB);
        // cv::imwrite("/home/steven/Desktop/alignedimage.jpg",warped);
        // cv::resize(warped, warped, depthimage.size(), cv::INTER_LINEAR);

        //Create the point cloud
        const std::string thefootpath = "/home/steven/Desktop/ulcerdatabase/patients/case_2/day_1/data/scene_1/depth_camera/depth_camera_1.dat";
        std::unordered_map<std::string, std::vector<std::vector<std::string>>> rawDat = readDatFile(thefootpath, "% ");

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = pointCloudFromData(rawDat, warped);
        // std::cout << pointcloud.get() << endl;

        // Create the visualiser and render it.
        pcl::visualization::PCLVisualizer::Ptr viewer = visualizePointCloud(pointcloud);
        while (!viewer->wasStopped()) {
            viewer->spin();
        }

    }
    else {
        std::cout << "Registration Error." << std::endl;
        std::cout << "Found depth chessboard: " << depthfound << std::endl;
        std::cout << "Found rgb chessboard: " << rgbfound << std::endl;
    }


    

    return 1;
}

