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

void removeGhosts(std::string inputFile, std::string outputDir, std::string outputName, double groundWidth, double groundHeight, double clusterResolution, double highResolution, double lowResolution);

int main(int argc, char *argv[]) {

    std::string inputFile = "/home/bence/ClionProjects/szakdoga/old/resources/pcd/Deak22.laz.pcd";
    std::string outputDir = "/home/bence/ClionProjects/szakdoga/old/resources/output/";

    if (argc == 3) {
        inputFile = argv[1];
        outputDir = argv[2];
    }

    {
        pcl::ScopeTime timer1("whole process");
//        {
//            pcl::ScopeTime timer2("astoria fast ");
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Astoria11z.laz.pcd", outputDir, "Astoria11_fast", 0.8, 0.5, 0.03, 0.20, 0.06);
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Astoria12z.laz.pcd", outputDir, "Astoria12_fast", 0.8, 0.5, 0.03, 0.20, 0.06);
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Astoria21z.laz.pcd", outputDir, "Astoria21_fast", 0.8, 0.5, 0.03, 0.20, 0.06);
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Astoria22z.laz.pcd", outputDir, "Astoria22_fast", 0.8, 0.5, 0.03, 0.20, 0.06);
//        }
        {
            pcl::ScopeTime timer2("astoria slow ");
            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Astoria11z.laz.pcd", outputDir, "Astoria11_slow", 0.8, 0.5, 0.02, 0.20, 0.06);
            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Astoria12z.laz.pcd", outputDir, "Astoria12_slow", 0.8, 0.5, 0.02, 0.20, 0.06);
            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Astoria21z.laz.pcd", outputDir, "Astoria21_slow", 0.8, 0.5, 0.02, 0.20, 0.06);
            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Astoria22z.laz.pcd", outputDir, "Astoria22_slow", 0.8, 0.5, 0.02, 0.20, 0.06);
        }
//        {
//            pcl::ScopeTime timer2("deak fast ");
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Deak11.laz.pcd", outputDir, "Deak11_fast", 0.8, 0.5, 0.03, 0.20, 0.06);
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Deak12.laz.pcd", outputDir, "Deak12_fast", 0.8, 0.5, 0.03, 0.20, 0.06);
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Deak21.laz.pcd", outputDir, "Deak21_fast", 0.8, 0.5, 0.03, 0.20, 0.06);
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Deak22.laz.pcd", outputDir, "Deak22_fast", 0.8, 0.5, 0.03, 0.20, 0.06);
//        }
//        {
//            pcl::ScopeTime timer2("deak slow ");
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Deak11.laz.pcd", outputDir, "Deak11_slow", 0.8, 0.5, 0.021, 0.20, 0.06);
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Deak12.laz.pcd", outputDir, "Deak12_slow", 0.8, 0.5, 0.021, 0.20, 0.06);
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Deak21.laz.pcd", outputDir, "Deak21_slow", 0.8, 0.5, 0.021, 0.20, 0.06);
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/pcd/Deak22.laz.pcd", outputDir, "Deak22_slow", 0.8, 0.5, 0.021, 0.20, 0.06);
//        }
//        {
//            pcl::ScopeTime timer2("gellert ");
//            removeGhosts("/home/bence/ClionProjects/szakdoga/old/resources/gellert4a_bin.laz.pcd", outputDir, "gellert", 800, 500, 3, 20, 6);
//        }
    }

    return 0;
}

auto highThings = [](CloudHelper<pcl::PointXYZRGB> cloud){
    cloud.sortByZAsc();
    return cloud.getCloud()->at(cloud.getCloud()->size() / 100 * 93).z - cloud.getCloud()->at(0).z < 2.5; //1.6 volt
};

auto lowThings = [](CloudHelper<pcl::PointXYZRGB> cloud){
    cloud.sortByZAsc();
    return cloud.getCloud()->at(cloud.getCloud()->size() / 10 * 9).z - cloud.getCloud()->at(0).z > 0.6; //.6 volt
};

void removeGhosts(std::string inputFile, std::string outputDir, std::string outputName, double groundWidth, double groundHeight, double clusterResolution, double highResolution, double lowResolution){

    CloudHelper<pcl::PointXYZRGB> inputCloud;
//    inputCloud.readCloudFromLas(inputFile);
    inputCloud.readCloudFromPcd(inputFile);

    CloudHelper<pcl::PointXYZRGB> groundCloud = inputCloud.removeGround(groundWidth, groundHeight);

    groundCloud.calculateBoundingBox();

    ClusteringResults<pcl::PointXYZRGB> clusteringResults = inputCloud.euclideanClustering(clusterResolution, 1500, 80000000);

    pcl::ScopeTime timer1("high ");
    std::vector<CloudHelper<pcl::PointXYZRGB>> remainingOutliersHigh = clusteringResults.refineOutliers(highResolution, 500, 80000000);
    clusteringResults.filterOutliers(highThings);

    pcl::ScopeTime timer2("low ");
    std::vector<CloudHelper<pcl::PointXYZRGB>> remainingOutliersLow = clusteringResults.refineOutliers(lowResolution, 500, 80000000);
    clusteringResults.filterOutliers(lowThings);

    CloudHelper<pcl::PointXYZRGB> cloudObjects = clusteringResults.getMergedCloudObjects();
    cloudObjects.addToCloud(groundCloud.getCloud());
    CloudHelper<pcl::PointXYZRGB> outliers = clusteringResults.getMergedOutliers();

    outliers.addCloudsToCloud(remainingOutliersLow);
    outliers.addCloudsToCloud(remainingOutliersHigh);

    cloudObjects.writeCloudBinary(outputDir + "objects_" + outputName + ".pcd");
    outliers.writeCloudBinary(outputDir + "outliers_" + outputName + ".pcd");
}
