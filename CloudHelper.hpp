#ifndef CLOUD_HELPER_H
#define CLOUD_HELPER_H

#include <iostream>
#include <string>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#ifdef _OPENMP

# include <omp.h>

#endif

#include "CloudUtil.hpp"
#include "ClusteringResults.hpp"

template<typename PointT>
class CloudHelper : public CloudUtil<PointT> {

public:
    CloudHelper() : CloudUtil<PointT>() {}

    CloudHelper(typename pcl::PointCloud<PointT>::Ptr cloud) : CloudUtil<PointT>(cloud) {}

    void addCloudsToCloud(std::vector<CloudHelper<PointT>> clouds){
        for(auto cloud : clouds){
            this->addToCloud(cloud.getCloud());
        }
    }

    CloudHelper<PointT> removeGround(double width = 0.8, double height = 0.5, double feet = 0.1) {
//        pcl::ScopeTime timer("remove ground");
        this->calculateBoundingBox();
        int rows = static_cast<int>(floor((this->boundingBox.maxY - this->boundingBox.minY) / width)) + 1;
        int columns = static_cast<int>(floor((this->boundingBox.maxX - this->boundingBox.minX) / width)) + 1;

        std::vector<CloudHelper<PointT>> cloudsGrid;
        for (int i = 0; i < rows * columns; i++) {
            cloudsGrid.push_back(CloudHelper<PointT>());
        }

        for (typename pcl::PointCloud<PointT>::iterator it = this->cloud->begin(); it != this->cloud->end(); it++) {
            int x = (it->x - this->boundingBox.minX) / width;
            int y = (it->y - this->boundingBox.minY) / width;
            cloudsGrid.at(y * columns + x).getCloud()->push_back(*it);
        }

        typename pcl::PointCloud<PointT>::Ptr groundCloud(new pcl::PointCloud<PointT>);
        this->cloud->clear();
        for (auto& cloudInGrid : cloudsGrid) {
            cloudInGrid.sortByZAsc();
            auto pclCloud = cloudInGrid.getCloud();
            if (pclCloud->empty()) {
                continue;
            }
            size_t cloudSize = pclCloud->size();
            if (pclCloud->at(cloudSize - 1).z - pclCloud->at(cloudSize / 10).z > height) {
                for(auto pointInCloudInGrid = pclCloud->begin(); pointInCloudInGrid != pclCloud->end(); pointInCloudInGrid++) {
                    if (pointInCloudInGrid->z < pclCloud->at(0).z + feet) { // <-- also 10 centi
                        groundCloud->insert(groundCloud->end(), *pointInCloudInGrid);
                    } else {
                        this->cloud->insert(this->cloud->end(), pointInCloudInGrid, pclCloud->end());
                        break;
                    }
                }
            } else {
                groundCloud->insert(groundCloud->end(), pclCloud->begin(), pclCloud->end());
            }
        }
        return CloudHelper<PointT>(groundCloud);
    }

    pcl::ModelCoefficients::Ptr calculatePlaneWithRansac() {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(1.5);

        seg.setInputCloud(this->cloud);
        seg.segment(*inliers, *coefficients);

        std::cout << "Model coefficients: " << coefficients->values[0] << " "
                  << coefficients->values[1] << " "
                  << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(this->cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*(this->cloud));

        return coefficients;
    }

    ClusteringResults<PointT> euclideanClustering(double clusterTolerance, int minClusterSize, int maxClusterSize) {
//        pcl::ScopeTime timer("euclidean clustering");
        typename pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>);
        kdTree->setInputCloud(this->cloud);

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<PointT> euclideanClusterExtraction;
        euclideanClusterExtraction.setClusterTolerance(clusterTolerance);
        euclideanClusterExtraction.setMinClusterSize(minClusterSize);
        euclideanClusterExtraction.setMaxClusterSize(maxClusterSize);
        euclideanClusterExtraction.setSearchMethod(kdTree);
        euclideanClusterExtraction.setInputCloud(this->cloud);
        euclideanClusterExtraction.extract(clusterIndices);

        pcl::ExtractIndices<PointT> extract;
        pcl::IndicesPtr allIndicesPtr(new std::vector<int>);
        std::vector<CloudHelper<PointT>> cloudObjects;

        for (std::vector<pcl::PointIndices>::const_iterator clusterIndicesIterator = clusterIndices.begin(); clusterIndicesIterator != clusterIndices.end(); ++clusterIndicesIterator) {
            pcl::IndicesPtr indicesPtr(new std::vector<int>);
            indicesPtr->insert(indicesPtr->end(), clusterIndicesIterator->indices.begin(), clusterIndicesIterator->indices.end());
            allIndicesPtr->insert(allIndicesPtr->end(), clusterIndicesIterator->indices.begin(), clusterIndicesIterator->indices.end());
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

            extract.setInputCloud (this->cloud);
            extract.setIndices (indicesPtr);
            extract.setNegative (false);
            extract.filter (*cloudCluster);

//            std::cout << "PointCloud representing the Cluster: " << cloudCluster->points.size() << " data points."
//                      << std::endl;
            cloudObjects.push_back(CloudHelper<PointT>(cloudCluster));
        }

        typename pcl::PointCloud<PointT>::Ptr outliersCluster(new pcl::PointCloud<PointT>);

        extract.setInputCloud (this->cloud);
        extract.setIndices (allIndicesPtr);
        extract.setNegative (true);
        extract.filter (*outliersCluster);

//        std::cout << cloudObjects.size() << " objects extracted" << std::endl;

        std::vector<CloudHelper<PointT>> outliersClusters;
        outliersClusters.push_back(CloudHelper<PointT>(outliersCluster));

        return ClusteringResults<PointT>(cloudObjects, outliersClusters);
    }
};

#endif //CLOUD_HELPER_H
