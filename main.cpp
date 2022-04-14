#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <unordered_map>
/**
 * Returns the contents of a point cloud .dat file
 * @param abspath The absolute path of the file you want.
 * @param sep A common string used before the group identifier string. Used to identify the beginning of a new group.
 */
std::unordered_map<std::string, std::vector<std::string>> readFile(std::string abspath, std::string sep) {

    std::ifstream file(abspath);

    std::vector<std::string> lineset;
    std::unordered_map<std::string, std::vector<std::string>> finalMap;

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
                currentKey = line.substr(line.find(sep) + 1);

                //now continue -- go to next line in the file!
            }
            else {
                //Nothing else to do except add the current line to the lineset. It is under the current separator.
                lineset.push_back(line);
            }
        }
        file.close();
    }
    else {
        throw std::runtime_error("File " + abspath + " not found.");
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
    // std::unordered_map<std::string, std::vector<std::string>> asd = groupData(readFile(argv[1]), "% ");

    //pcl::PointCloud<pcl::PointXYZ> pointcloud = pointCloudFromData(rawpcdata);

    return 1;
}

