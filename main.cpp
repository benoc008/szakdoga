#include <string>
#include "CloudHelper.hpp"

/**
 * notes
 *
 * magic numberekkel nem jo dolgozni, mert inputonkent erosen elteroek a parameterek
 * mas merkator, mas lidar, eltero pontsuruseg, tavoli objektumok surusege nyilvan kisebb.
 *
 * kerdes:
 *  lehet-e hasznalni egyeb informaciot a feldolgozaskor (timestamp, meres helye)
 *  mi az elvart futasido (/terulet v /pontmennyiseg)
 *
 *  outlier-eket clusterezni, aztan a bb-on beluli elemeket is outlierre tenni
 *
 * @param inputFile
 * @param outputDir
 */

void findGhosts(std::string inputFile, std::string outputDir);
void removeGhosts(std::string inputFile, std::string outputDir);

int main(int argc, char *argv[]) {

    std::string inputFile = "/home/bence/ClionProjects/szakdoga/old/resources/down_3_13.laz";
    std::string outputDir = "/home/bence/ClionProjects/szakdoga/old/resources/output/";

    if (argc == 3) {
        inputFile = argv[1];
        outputDir = argv[2];
    }


    removeGhosts(inputFile, outputDir);

    return 0;
}

auto highThings = [](CloudHelper<pcl::PointXYZRGB> cloud){
    cloud.sortByZAsc();
    return cloud.getCloud()->at(cloud.getCloud()->size() / 10 * 9).z - cloud.getCloud()->at(0).z < 2000;
};

void removeGhosts(std::string inputFile, std::string outputDir){
    pcl::ScopeTime timer("whole process");
    CloudHelper<pcl::PointXYZRGB> inputCloud;
//    inputCloud.readCloudFromPcd(inputFile);
    inputCloud.readCloudFromLas(inputFile);
    CloudHelper<pcl::PointXYZRGB> groundCloud = inputCloud.removeGround(0.8f, 0.5f);
    groundCloud.calculateBoundingBox();

    ClusteringResults<pcl::PointXYZRGB> clusteringResults = inputCloud.euclideanClustering(0.03, 1500, 80000000);

//    std::vector<CloudHelper<pcl::PointXYZRGB>> remainingOutliers = clusteringResults.refineOutliers(150.0, 1000, 80000000);
//    clusteringResults.filterOutliers(highThings);

    CloudHelper<pcl::PointXYZRGB> cloudObjects = clusteringResults.getMergedCloudObjects();
    cloudObjects.addToCloud(groundCloud.getCloud());
    CloudHelper<pcl::PointXYZRGB> outliers = clusteringResults.getMergedOutliers();
//    outliers.addCloudsToCloud(remainingOutliers);

    cloudObjects.writeCloudBinary(outputDir + "objects_down_3.pcd");
    outliers.writeCloudBinary(outputDir + "outliers_down_3.pcd");
}

//void findGhosts(std::string inputFile, std::string outputDir) {
//    pcl::ScopeTime timer("whole process");
//    CloudHelper<pcl::PointXYZRGB> inputCloud;
//    inputCloud.readCloudFromPcd(inputFile);
//    inputCloud.removeGround(800.0f, 500.0f);
//
//    std::vector<CloudHelper<pcl::PointXYZRGB>> cloudClusters = inputCloud.euclideanClustering(50, 100, 5000000);
//    int index = 0;
//    for (auto& cluster : cloudClusters) {
//        cluster.clearIfHeightDoesNotFit(1000.0f, 2000.0f);
//        if (cluster.getCloud()->size() != 0) {
//            if (cluster.calculateDensity(5) > 500.0f) { // <--- mert ez igazabol az atlag ponttavolsag
//                cluster.writeCloudBinary(outputDir + "human_" + std::to_string(index) + ".pcd");
//            } else {
//                cluster.writeCloudBinary(outputDir + "not_human_" + std::to_string(index) + ".pcd");
//            }
//            index++;
//        }
//
//    }

//}