#ifndef SLAM3D_COMMON_HPP
#define SLAM3D_COMMON_HPP

#include <sys/time.h>
#include <base/Pose.hpp>
#include <slam3d/Sensor.hpp>

namespace slam3d
{
	long timevaldiff(const timeval& start, const timeval& end);
	
	Transform pose2transform(const base::Pose& pose);
}

#endif
