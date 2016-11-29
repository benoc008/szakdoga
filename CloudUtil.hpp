#ifndef OLD_CLOUDUTIL_HPP
#define OLD_CLOUDUTIL_HPP

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>
#include <stdlib.h>
#include "LasToPcdConverter.hpp"

struct BoundingBox {
    float minZ = FLT_MAX, maxZ = FLT_MIN, minX = FLT_MAX, maxX = FLT_MIN, minY = FLT_MAX, maxY = FLT_MIN;

    void reset() {
        minZ = FLT_MAX, maxZ = FLT_MIN, minX = FLT_MAX, maxX = FLT_MIN, minY = FLT_MAX, maxY = FLT_MIN;
    }
};

template <typename PointT>
class CloudUtil {
public:

    CloudUtil() { cloud = (new pcl::PointCloud<PointT>)->makeShared(); }

    CloudUtil(typename pcl::PointCloud<PointT>::Ptr cloud) : CloudUtil() {
        pcl::copyPointCloud(*cloud, *(this->cloud));
    };

    void colorCloud(uint8_t r, uint8_t g, uint8_t b);

    void generateRandomCloud(int numPoints, int size) {
        cloud->clear();
        cloud->resize(numPoints);
        for (int i = 0; i < numPoints; i++) {
            double randMax = static_cast<double>(RAND_MAX);
            cloud->at(i).x = (rand() / randMax) * size;
            cloud->at(i).y = (rand() / randMax) * size;
            cloud->at(i).z = (rand() / randMax) * size;
        }
    }

    void translateCloud(float x, float y, float z) {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform(0, 3) = x;
        transform(1, 3) = y;
        transform(2, 3) = z;
        pcl::transformPointCloud(*cloud, *cloud, transform);
    }

    void takeScreenshot(std::string fileName){
        pcl::visualization::PCLVisualizer visu("Alignment");
        visu.addPointCloud (cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (cloud, 0.0, 255.0, 0.0), "cloud");
        visu.setShowFPS(false);
        visu.spin ();
        visu.saveScreenshot(fileName);
    }

    void readCloudFromPcd(std::string fileName) {
        pcl::PCDReader reader;
        cloud->clear();
        reader.read(fileName, *cloud);
        std::cout << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
    }

    void readCloudFromLas(std::string fileName){
        LasToPcdConverter<pcl::PointCloud<PointT> > converter;
        liblas::Header header;
        converter.convertLasToPcd(fileName, header, *cloud);
        offset = converter.getOffset();
        std::cout << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
    }

    void writeCloudBinary(std::string fileName) {
        pcl::PCDWriter writer;
        writer.write(fileName, *cloud, true);
        std::cout << "File written to: " << fileName << std::endl;
    }

    void writeCloudAscii(std::string fileName) {
        pcl::PCDWriter writer;
        writer.write(fileName, *cloud, false);
        std::cout << "File written to: " << fileName << std::endl;
    }

    void addToCloud(typename pcl::PointCloud<PointT>::Ptr cloudToAdd){
        *cloud += *cloudToAdd;
    }

    typename pcl::PointCloud<PointT>::Ptr getCloud() {
        return cloud;
    }

    void sortByZAsc(){
        std::sort(cloud->begin(), cloud->end(), [](PointT a, PointT b) { return a.z < b.z; });
    }

    void calculateBoundingBox() {
        for (typename pcl::PointCloud<PointT>::iterator it = cloud->begin(); it != cloud->end(); it++) {
            if (it->z < boundingBox.minZ) {
                boundingBox.minZ = it->z;
            }
            if (it->z > boundingBox.maxZ) {
                boundingBox.maxZ = it->z;
            }
            if (it->x < boundingBox.minX) {
                boundingBox.minX = it->x;
            }
            if (it->x > boundingBox.maxX) {
                boundingBox.maxX = it->x;
            }
            if (it->y < boundingBox.minY) {
                boundingBox.minY = it->y;
            }
            if (it->y > boundingBox.maxY) {
                boundingBox.maxY = it->y;
            }
        }
    }

    CloudUtil<PointT> intersectWith(CloudUtil& targetCloudUtil){
        calculateBoundingBox();
        targetCloudUtil.calculateBoundingBox(); // todo csak egyszer
        CloudUtil<PointT> resultCloud;
        for (typename pcl::PointCloud<PointT>::iterator it = cloud->begin(); it != cloud->end(); it++) {
            if(it->x <= boundingBox.maxX && it->x <= targetCloudUtil.boundingBox.maxX &&
               it->x >= boundingBox.minX && it->x >= targetCloudUtil.boundingBox.minX &&

               it->y <= boundingBox.maxY && it->y <= targetCloudUtil.boundingBox.maxY &&
               it->y >= boundingBox.minY && it->y >= targetCloudUtil.boundingBox.minY &&

               it->z <= boundingBox.maxZ && it->z <= targetCloudUtil.boundingBox.maxZ &&
               it->z >= boundingBox.minZ && it->z >= targetCloudUtil.boundingBox.minZ ){

                PointT point(*it);
                resultCloud.cloud->push_back(point);
            }
        }
        return resultCloud;
    }

protected:
    typename pcl::PointCloud<PointT>::Ptr cloud;
    BoundingBox boundingBox;
    Offset offset;
};

#endif //OLD_CLOUDUTIL_HPP
