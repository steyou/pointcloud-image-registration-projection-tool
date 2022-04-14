#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <unordered_map>
/**
 * Returns the contents of a text file
 * @param abspath The absolute path of the file you want.
 */
std::vector<std::string> readFile(std::string abspath) {
    std::vector<std::string> lines;
    std::ifstream file(abspath);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            // using printf() in all tests for consistency
            // printf("%s", line.c_str());
            lines.push_back(line.c_str());
        }
        file.close();
    }
    else {
        throw std::runtime_error("File " + abspath + " not found.");
    }
    return lines;
}

std::unordered_map<std::string, std::vector<std::string>> groupData(const std::vector<std::string> &raw, std::string sep) {
    //TODO: throw error if there is no sep on the first line

    //tempContainer stores each section before they are added to the dictionary
    std::vector<std::string> tempContainer;
    std::unordered_map<std::string, std::vector<std::string>> finalMap;

    std::string currentKey;

    for (int i = 0; i < raw.size(); i++) {
        //if the current line begins with a separator...
        if (raw.at(i).rfind(sep, 0) == 0) {
            
            //add to the map if this line is not the first.
            if (tempContainer.size() > 0) {
                finalMap[currentKey] = tempContainer;
                tempContainer.clear();
            }

            //setting a new key indicates to add data under a new 'group'
            currentKey = raw.at(i).substr(raw.at(i).find(sep) + 1);

            //now go to next line in the file!
        }
        else {
            //add the current line to the tempContainer
            tempContainer.push_back(raw.at(i));
            std::cout << raw.at(i) << std::endl;
        }
    }

    //the last iteration does not insert into the map so it must be done manually.
    //this if leaves no assumption that the raw array was empty.
    if (!currentKey.empty() && tempContainer.size() > 0) {
        finalMap[currentKey] = tempContainer;
    }

    return finalMap;

}

// pcl::PointCloud<pcl::PointXYZ> pointCloudFromData(std::vector<std::string> rawData) {
//     //Read the data file
//     // std::vector<std::string> rawpoints = readFile(abspath);

//     //Parse the file and create the points.

// }

int main(int argc, char* argv[]) {

    //Handle command line arguments
    if (argc < 2) {
        std::cout << "this the help text" << std::endl;
        return 0;
    }
    // exit if the image and point cloud arguments are not provided.
    else if (argc == 2)
    {
        std::cout << "Error: missing point cloud argument" << std::endl;
        return 0;
    }
    else if (argc > 3) {
        std::cout << "Error: expected exactly 2 arguments"<<std::endl;
        return 0;
    }
    // for (int i = 0; i < argc; ++i)
    //     std::cout << argv[i] << std::endl;
    
    // std::cout << "sdlkfjhsdklf" << std::endl;
    // std::vector<std::string> rawpcdata = readFile(argv[1]);
    std::unordered_map<std::string, std::vector<std::string>> asd = groupData(readFile(argv[1]), "% ");

    //pcl::PointCloud<pcl::PointXYZ> pointcloud = pointCloudFromData(rawpcdata);

    return 1;
}

