#ifndef SLAM3D_ROCK_TYPES_HPP
#define SLAM3D_ROCK_TYPES_HPP

#include <base/Time.hpp>
#include <base/Pose.hpp>
#include <base/samples/Pointcloud.hpp>

#include <string>

namespace slam3d
{
	struct Localized
	{
		base::Time stamp;
		std::string robot_name;
		std::string sensor_name;
		std::string unique_id;
		base::Pose sensor_pose;
	};
	
	struct LocalizedPointcloud : public Localized
	{
		base::samples::Pointcloud point_cloud;
	};
	
	struct SpatialConstraint
	{
		std::string source_unique_id;
		std::string target_unique_id;
		std::string sensor_name;
		base::Pose relative_pose;
		base::Matrix6d covariance;
	};
}

#endif
