#ifndef INCLUDED_LASTOPCD_HPP
#define INCLUDED_LASTOPCD_HPP

#include <fstream>
#include <string>

#include <liblas/liblas.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>

class Offset {
public:
    double x, y, z;

	Offset(){
		reset();
	}

    void reset(){
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    bool isNulls(){
        return (x == 0 && y == 0 && z == 0);
    }
};

template<typename CloudT>
class LasToPcdConverter {
public:

	LasToPcdConverter(){
		_calculateOffset = false;
	}

    void convertLasToPcdWithOffset(const std::string &input, liblas::Header &header, CloudT &cloud){
        _calculateOffset = _offset.isNulls();
        LAStoPCD(input, header, cloud);
    }

    void convertLasToPcd(const std::string &input, liblas::Header &header, CloudT &cloud){
        _offset.reset();
        _calculateOffset = true;
        LAStoPCD(input, header, cloud);
    }

    void setOffset(Offset offset){
        _offset.x = offset.x;
        _offset.y = offset.y;
        _offset.z = offset.z;
    }

    Offset getOffset(){
        return _offset;
    }

private:
    Offset _offset;
    bool _calculateOffset;

    void LAStoPCD(const std::string &input, liblas::Header &header, CloudT &cloud) {
        std::ifstream ifs;

        if (!liblas::Open(ifs, input.c_str())) {
            std::cerr << "Cannot open " << input << " for read. Exiting..." << std::endl;
            return;
        }

        liblas::ReaderFactory f;
        liblas::Reader reader = f.CreateWithStream(ifs);

        typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

        header = reader.GetHeader();

        cloud.width = header.GetPointRecordsCount();
        cloud.height = 1;  // unorganized point cloud
        cloud.is_dense = false;
        cloud.points.resize(cloud.width);

        typename CloudT::PointType testPoint = cloud.points[0];

        bool has_x = false;
        bool has_y = false;
        bool has_z = false;
        float x_val = 0.0f;
        float y_val = 0.0f;
        float z_val = 0.0f;
        pcl::for_each_type<FieldList>(
                pcl::CopyIfFieldExists<typename CloudT::PointType, float>(testPoint, "x", has_x, x_val));
        pcl::for_each_type<FieldList>(
                pcl::CopyIfFieldExists<typename CloudT::PointType, float>(testPoint, "y", has_y, y_val));
        pcl::for_each_type<FieldList>(
                pcl::CopyIfFieldExists<typename CloudT::PointType, float>(testPoint, "z", has_z, z_val));

        if (has_x && has_y && has_z) {
            for (size_t i = 0; i < cloud.points.size(); ++i) {
                reader.ReadNextPoint();
                liblas::Point const &q = reader.GetPoint();
                if (_calculateOffset && _offset.isNulls()) {
                    _offset.x = q.GetX();
                    _offset.y = q.GetY();
                    _offset.z = q.GetZ();
                }

                typename CloudT::PointType p = cloud.points[i];
                pcl::for_each_type<FieldList>(
                        pcl::SetIfFieldExists<typename CloudT::PointType, float>(p, "x", static_cast<float>(q.GetX() - _offset.x)));
                pcl::for_each_type<FieldList>(
                        pcl::SetIfFieldExists<typename CloudT::PointType, float>(p, "y", static_cast<float>(q.GetY() - _offset.y)));
                pcl::for_each_type<FieldList>(
                        pcl::SetIfFieldExists<typename CloudT::PointType, float>(p, "z", static_cast<float>(q.GetZ() - _offset.z)));
                cloud.points[i] = p;
            }
        }

        bool has_i = false;
        float i_val = 0.0f;
        pcl::for_each_type<FieldList>(
                pcl::CopyIfFieldExists<typename CloudT::PointType, float>(testPoint, "intensity", has_i, i_val));

        if (has_i) {
            reader.Reset();

            for (size_t i = 0; i < cloud.points.size(); ++i) {
                reader.ReadNextPoint();
                liblas::Point const &q = reader.GetPoint();

                typename CloudT::PointType p = cloud.points[i];
                pcl::for_each_type<FieldList>(pcl::SetIfFieldExists<typename CloudT::PointType, float>(p, "intensity",
                                                                                                       static_cast<float>(q.GetIntensity())));
                cloud.points[i] = p;
            }
        }
    }
};

#endif  // INCLUDED_LASTOPCD_HPP
