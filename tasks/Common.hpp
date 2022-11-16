#ifndef SLAM3D_COMMON_HPP
#define SLAM3D_COMMON_HPP

#include <sys/time.h>
#include <base/Pose.hpp>
#include <base/Time.hpp>
#include <base/samples/Pointcloud.hpp>

#include <slam3d/core/Types.hpp>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

namespace slam3d
{
	long timevaldiff(const timeval& start, const timeval& end);
	
	base::Time timeval2time(const timeval& tv);
	
	Transform pose2transform(const base::Pose& pose);

	PointCloud::Ptr createFromRockMessage(const base::samples::Pointcloud& cloud);
	void createFromPcl(slam3d::PointCloud::ConstPtr pcl_cloud, base::samples::Pointcloud& base_cloud);
	PointCloudMeasurement::Ptr castToPointcloud(Measurement::Ptr m);
}

#endif
