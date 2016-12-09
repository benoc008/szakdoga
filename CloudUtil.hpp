#ifndef CLOUDUTIL_HPP
#define CLOUDUTIL_HPP

#include <liblas/liblas.hpp>

#include <string>
#include <fstream>  // std::ofstream
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

    void readCloudFromPcd(std::string fileName) {
        pcl::PCDReader reader;
        cloud->clear();
        reader.read(fileName, *cloud);
//        std::cout << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
    }

    void readCloudFromLas(std::string fileName){
        LasToPcdConverter<pcl::PointCloud<PointT> > converter;
        liblas::Header header;
        converter.convertLasToPcd(fileName, header, *cloud);
        offset = converter.getOffset();
//        std::cout << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
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

    void writeCloudLas(std::string fileName){
        std::ofstream ofs;
        ofs.open(fileName, ios::out | ios::binary);

        liblas::Header header;
        header.SetScale( 0.0001, 0.0001, 0.0001 );

        liblas::Writer writer(ofs, header);

        for (typename pcl::PointCloud<PointT>::iterator it = cloud->begin(); it != cloud->end(); it++) {
            liblas::Point point(&header);
            point.SetCoordinates(it->x, it->y, it->z);
            const liblas::Color color(it->r, it->g, it->b);
            point.SetColor(color);

            writer.WritePoint(point);
        }

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

protected:
    typename pcl::PointCloud<PointT>::Ptr cloud;
    BoundingBox boundingBox;
    Offset offset;
};

#endif //CLOUDUTIL_HPP
