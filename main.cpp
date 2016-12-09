#include <string>
#include "CloudHelper.hpp"
#include "ParamBuilder.hpp"
#include "FileUtil.hpp"

void removeGhosts(ParamBuilder& paramBuilder, std::string inputFile, std::string outputFile);

int main(int argc, char *argv[]) {

    ParamBuilder paramBuilder(argc, argv);
    if(!paramBuilder.isValid()){
        return 1;
    }

    if(isDirectory(paramBuilder.getInputFile())){
        std::vector<std::string> inputFiles = getFilesInDir(paramBuilder.getInputFile());
        int counter = 1;
        for(std::string inputFile : inputFiles){
            std::cout << counter++ << "/" << inputFiles.size() << std::endl;
            boost::filesystem::path inputFileBoost(inputFile);
            removeGhosts(paramBuilder, inputFile, paramBuilder.getOutputFile() + "out_" + inputFileBoost.filename().string());
        }
    } else {
        removeGhosts(paramBuilder, paramBuilder.getInputFile(), paramBuilder.getOutputFile());
    }

    return 0;
}

auto highThings = [](CloudHelper<pcl::PointXYZRGB> cloud, double minHeight){
    cloud.sortByZAsc();
    return cloud.getCloud()->at(cloud.getCloud()->size() / 100 * 90).z - cloud.getCloud()->at(0).z < minHeight; //1.6 volt, astoria 2.5
};

auto lowThings = [](CloudHelper<pcl::PointXYZRGB> cloud, double maxHeight){
    cloud.sortByZAsc();
    return cloud.getCloud()->at(cloud.getCloud()->size() / 10 * 9).z - cloud.getCloud()->at(0).z > maxHeight; //.6 volt
};

void removeGhosts(ParamBuilder& paramBuilder, std::string inputFile, std::string outputFile){
    pcl::StopWatch mainWatch;
    CloudHelper<pcl::PointXYZRGB> inputCloud;
    boost::filesystem::path inputFileBoost(outputFile);
    if(inputFileBoost.extension().compare(".las") == 0 || inputFileBoost.extension().compare(".laz") == 0 ){
        inputCloud.readCloudFromLas(inputFile);
    } else {
        inputCloud.readCloudFromPcd(inputFile);
    }

    pcl::StopWatch removeGroundWatch;
    CloudHelper<pcl::PointXYZRGB> groundCloud = inputCloud.removeGround(paramBuilder.getGroundWidth(), paramBuilder.getGroundHeight(), paramBuilder.getGroundHeight() / 5.0);
    if(paramBuilder.isDebug()){
        std::cout << "Talaj eltávolítása: " << removeGroundWatch.getTime() << "ms." << std::endl;
    }

    groundCloud.calculateBoundingBox();

    pcl::StopWatch euclideanClusteringWatch;
    ClusteringResults<pcl::PointXYZRGB> clusteringResults = inputCloud.euclideanClustering(paramBuilder.getClusterResolution(), 1500, 80000000);
    if(paramBuilder.isDebug()){
        std::cout << "Kezdeti csoportosítás: " << euclideanClusteringWatch.getTime() << "ms." << std::endl;
    }

    pcl::StopWatch filterHighWatch;
    std::vector<CloudHelper<pcl::PointXYZRGB>> remainingOutliersHigh = clusteringResults.refineOutliers(paramBuilder.getHighResolution(), 1500, 80000000);
    clusteringResults.filterOutliers(highThings, paramBuilder.getMinHeight());
    if(paramBuilder.isDebug()){
        std::cout << "Magas objektumok szűrése: " << filterHighWatch.getTime() << "ms." << std::endl;
    }

    pcl::StopWatch filterLowWatch;
    std::vector<CloudHelper<pcl::PointXYZRGB>> remainingOutliersLow = clusteringResults.refineOutliers(paramBuilder.getLowResolution(), 500, 80000000);
    clusteringResults.filterOutliers(lowThings, paramBuilder.getMaxHeight());
    if(paramBuilder.isDebug()){
        std::cout << "Alacsony objektumok szűrése: " << filterLowWatch.getTime() << "ms." << std::endl;
    }

    CloudHelper<pcl::PointXYZRGB> cloudObjects = clusteringResults.getMergedCloudObjects();
    cloudObjects.addToCloud(groundCloud.getCloud());
    CloudHelper<pcl::PointXYZRGB> outliers = clusteringResults.getMergedOutliers();

    outliers.addCloudsToCloud(remainingOutliersLow);
    outliers.addCloudsToCloud(remainingOutliersHigh);


    cloudObjects.writeCloudBinary(outputFile);
    boost::filesystem::path outputFileBoost(outputFile);
    if(paramBuilder.isGhosts()){
        std::string filename = outputFileBoost.filename().string();
        char separator = boost::filesystem::path::preferred_separator;
        outliers.writeCloudBinary(outputFileBoost.remove_filename().string() + separator + "szellem_" + filename);
    }
    std::cout << "Szellemek eltávolítása: " << mainWatch.getTime() << "ms." << std::endl;
}
