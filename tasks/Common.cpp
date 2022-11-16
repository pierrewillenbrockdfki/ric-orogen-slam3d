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

base::Time slam3d::timeval2time(const timeval& tv)
{
	uint64_t usec = (tv.tv_sec * base::Time::UsecPerSec) + tv.tv_usec;
	return base::Time::fromMicroseconds(usec);
}

Transform slam3d::pose2transform(const base::Pose& pose)
{
	Eigen::Affine3d affine = pose.toTransform();
	Transform tf;
	tf.linear() = affine.linear();
	tf.translation() = affine.translation();
	return tf;
}

PointCloud::Ptr slam3d::createFromRockMessage(const base::samples::Pointcloud& cloud_in)
{
	PointCloud::Ptr cloud_out(new PointCloud);
	cloud_out->header.stamp = cloud_in.time.toMicroseconds();
	cloud_out->reserve(cloud_in.points.size());
	
	unsigned numPoints = cloud_in.points.size();
	for(unsigned i = 0; i < numPoints; ++i)
	{
		PointType p;
		p.x = cloud_in.points[i][0];
		p.y = cloud_in.points[i][1];
		p.z = cloud_in.points[i][2];
		cloud_out->push_back(p);
	}
	return cloud_out;
}

void slam3d::createFromPcl(PointCloud::ConstPtr pcl_cloud, base::samples::Pointcloud& base_cloud)
{
	base_cloud.time.fromMicroseconds(pcl_cloud->header.stamp);
	base_cloud.points.reserve(pcl_cloud->size());
	for(PointCloud::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
	{
		base::Point p;
		p[0] = it->x;
		p[1] = it->y;
		p[2] = it->z;
		base_cloud.points.push_back(p);
	}
}

PointCloudMeasurement::Ptr slam3d::castToPointcloud(Measurement::Ptr m)
{
	PointCloudMeasurement::Ptr pcm = boost::dynamic_pointer_cast<PointCloudMeasurement>(m);
	if(!pcm)
	{
		throw BadMeasurementType();
	}
	return pcm;
}
