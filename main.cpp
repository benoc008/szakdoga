#include <string>
#include "CloudHelper.hpp"


void findGhosts(std::string inputFile, std::string outputDir);

int main(int argc, char *argv[]) {

    std::string inputFile = "/home/bence/ClionProjects/szakdoga/old/resources/gellert4a_bin.pcd";
    std::string outputDir = "/home/bence/ClionProjects/szakdoga/old/resources/output/";

    if (argc == 3) {
        inputFile = argv[1];
        outputDir = argv[2];
    }


    findGhosts(inputFile, outputDir);

    return 0;
}

void findGhosts(std::string inputFile, std::string outputDir) {
    pcl::ScopeTime timer("whole process");
    CloudHelper<pcl::PointXYZRGB> inputCloud;
    inputCloud.readCloudFromPcd(inputFile);
    inputCloud.removeGround(800.0f, 500.0f);

    std::cout << inputCloud.getCloud()->size() << std::endl;

    std::vector<CloudHelper<pcl::PointXYZRGB>> cloudClusters = inputCloud.euclideanClustering(50);
    int index = 0;
    for (auto& cluster : cloudClusters) {
        cluster.clearIfHeightDoesNotFit(1000.0f, 2000.0f);
        if (cluster.getCloud()->size() != 0) {
            if (cluster.calculateDensity(5) > 500.0f) { // <--- mert ez igazabol az atlag ponttavolsag
                cluster.writeCloudBinary(outputDir + "human_" + std::to_string(index) + ".pcd");
            } else {
                cluster.writeCloudBinary(outputDir + "not_human_" + std::to_string(index) + ".pcd");
            }
            index++;
        }

    }

}