/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DistributedPointcloudMapper.hpp"

#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

using namespace slam3d;

slam::Transform pose2eigen(base::Pose pose)
{
	Eigen::Affine3d tmp = pose.toTransform();
	slam::Transform tf;
	tf.linear() = tmp.linear();
	tf.translation() = tmp.translation();
	return tf;
}

DistributedPointcloudMapper::DistributedPointcloudMapper(std::string const& name)
	: DistributedPointcloudMapperBase(name)
{
}

DistributedPointcloudMapper::DistributedPointcloudMapper(std::string const& name, RTT::ExecutionEngine* engine)
	: DistributedPointcloudMapperBase(name, engine)
{
}

DistributedPointcloudMapper::~DistributedPointcloudMapper()
{
}

bool DistributedPointcloudMapper::configureHook()
{
	if (! DistributedPointcloudMapperBase::configureHook())
		return false;
	return true;
}

bool DistributedPointcloudMapper::startHook()
{
	if (! DistributedPointcloudMapperBase::startHook())
		return false;
	return true;
}

void DistributedPointcloudMapper::updateHook()
{
	DistributedPointcloudMapperBase::updateHook();
	
	// Send all new nodes to other robots
	
	// Add readings from other robots to own map
	slam3d::LocalizedPointcloud lc;
	slam::PointCloudMeasurement* m;
	slam::Transform sensor_pose;
	slam::Transform robot_pose;
	boost::uuids::uuid id;
	while(_external_in.read(lc, false) == RTT::NewData)
	{
		id = boost::lexical_cast<boost::uuids::uuid>(lc.unique_id);
		sensor_pose = pose2eigen(lc.sensor_pose);
		robot_pose = pose2eigen(lc.corrected_pose);
		slam::PointCloud::Ptr cloud = createFromRockMessage(lc.point_cloud);
		m = new slam::PointCloudMeasurement(cloud, lc.robot_name, lc.sensor_name, sensor_pose, id);
		mMapper->addExternalReading(m, robot_pose);
	}
}

void DistributedPointcloudMapper::errorHook()
{
	DistributedPointcloudMapperBase::errorHook();
}

void DistributedPointcloudMapper::stopHook()
{
	DistributedPointcloudMapperBase::stopHook();
}

void DistributedPointcloudMapper::cleanupHook()
{
	DistributedPointcloudMapperBase::cleanupHook();
}
