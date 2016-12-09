#ifndef OLD_CLUSTERINGRESULTS_HPP
#define OLD_CLUSTERINGRESULTS_HPP

#include "CloudHelper.hpp"

template<typename PointT>
class CloudHelper;

template<typename PointT>
class ClusteringResults {
public:

    std::vector<CloudHelper<PointT> > cloudObjects;
    std::vector<CloudHelper<PointT> > outliersCluster;

    ClusteringResults(std::vector<CloudHelper<PointT> > cloudObjects, std::vector<CloudHelper<PointT> > outliersCluster)
            :
            cloudObjects{std::move(cloudObjects)}, outliersCluster{std::move(outliersCluster)} {}

    CloudHelper<PointT> getMergedCloudObjects() {
        return mergeCloudHelperVector(cloudObjects);
    }

    CloudHelper<PointT> getMergedOutliers() {
        return mergeCloudHelperVector(outliersCluster);
    }

    void filterOutliers(std::function<bool(CloudHelper<PointT>, double)> filterFunction, double param) {

        auto cloudIterator = std::begin(outliersCluster);

        while (cloudIterator != std::end(outliersCluster)) {
            if (!filterFunction(*cloudIterator, param)) {
                cloudObjects.push_back(*cloudIterator);
                cloudIterator = outliersCluster.erase(cloudIterator);
            } else {
                ++cloudIterator;
            }
        }

    }

    std::vector<CloudHelper<PointT>> refineOutliers(double clusterTolerance, int minClusterSize, int maxClusterSize) {
        CloudHelper<PointT> mergedOutliers = getMergedOutliers();
//        std::cout << "merged outliers size: " << mergedOutliers.getCloud()->size() << std::endl;
        ClusteringResults<PointT> clusteringResults = mergedOutliers.euclideanClustering(clusterTolerance,
                                                                                         minClusterSize,
                                                                                         maxClusterSize);
        outliersCluster = clusteringResults.cloudObjects;
        return clusteringResults.outliersCluster;
    }


private:

    CloudHelper<PointT> mergeCloudHelperVector(std::vector<CloudHelper<PointT>> cloudHelperVector) {

        CloudHelper<PointT> merged;

        for (auto cloudHelper : cloudHelperVector) {
            merged.addToCloud(cloudHelper.getCloud());
        }

        return merged;
    }

};


#endif //OLD_CLUSTERINGRESULTS_HPP
