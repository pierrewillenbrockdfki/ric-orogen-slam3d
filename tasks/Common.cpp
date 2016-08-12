#include "Common.hpp"

using namespace slam3d;

long slam3d::timevaldiff(const timeval& start, const timeval& end)
{
	long msec;
	msec=(end.tv_sec - start.tv_sec)*1000;
	msec+=(end.tv_usec - start.tv_usec)/1000;
	if(msec > 0)
		return msec;
	else
		return -msec;
}

Transform slam3d::pose2transform(const base::Pose& pose)
{
	Eigen::Affine3d affine = pose.toTransform();
	Transform tf;
	tf.linear() = affine.linear();
	tf.translation() = affine.translation();
	return tf;
}
