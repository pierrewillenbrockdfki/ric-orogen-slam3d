#ifndef SLAM3D_COMMON_HPP
#define SLAM3D_COMMON_HPP

#include <sys/time.h>

namespace slam3d
{
	long timevaldiff(const timeval& start, const timeval& end);
}

#endif
