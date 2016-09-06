#include "CloudUtil.hpp"

template<>
void CloudUtil<pcl::PointXYZ>::colorCloud(uint8_t r, uint8_t g, uint8_t b){
    throw "wrong template class";
}

template<>
void CloudUtil<pcl::PointXYZRGB>::colorCloud(uint8_t r, uint8_t g, uint8_t b){
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++) {
        it->r = r;
        it->g = g;
        it->b = b;
    }
}

template<>
void CloudUtil<pcl::PointXYZRGBNormal>::colorCloud(uint8_t r, uint8_t g, uint8_t b){
    for (pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it = cloud->begin(); it != cloud->end(); it++) {
        it->r = r;
        it->g = g;
        it->b = b;
    }
}

