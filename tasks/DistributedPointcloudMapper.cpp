/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DistributedPointcloudMapper.hpp"

#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

#include <base/Logging.hpp>

using namespace slam3d;

slam3d::Transform pose2eigen(const base::Pose& pose)
{
	Eigen::Affine3d tmp = pose.toTransform();
	slam3d::Transform tf;
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
	while(!mNewVertices.empty())
	{
		slam3d::VertexObject v = mNewVertices.front();
		mNewVertices.pop();
		slam3d::PointCloudMeasurement::Ptr m = boost::dynamic_pointer_cast<slam3d::PointCloudMeasurement>(v.measurement);
		slam3d::LocalizedPointcloud loc_cloud;
		loc_cloud.robot_name = mRobotName;
		loc_cloud.sensor_name = mPclSensor->getName();
		loc_cloud.stamp.fromMicroseconds(m->getPointCloud()->header.stamp);
		loc_cloud.sensor_pose = base::Pose(m->getSensorPose());
		loc_cloud.corrected_pose = base::Pose(v.corrected_pose);
		loc_cloud.unique_id = boost::uuids::to_string(m->getUniqueId());
		createFromPcl(m->getPointCloud(), loc_cloud.point_cloud);
		_external_out.write(loc_cloud);
	}
	
	// Add readings from other robots to own map
	slam3d::LocalizedPointcloud lc;
	slam3d::Transform sensor_pose;
	slam3d::Transform robot_pose;
	boost::uuids::uuid id;
	while(_external_in.read(lc, false) == RTT::NewData)
	{
		id = boost::lexical_cast<boost::uuids::uuid>(lc.unique_id);
		sensor_pose = pose2eigen(lc.sensor_pose);
		robot_pose = pose2eigen(lc.corrected_pose);
		slam3d::PointCloud::Ptr cloud = createFromRockMessage(lc.point_cloud);
		slam3d::PointCloudMeasurement::Ptr m(new slam3d::PointCloudMeasurement(cloud, lc.robot_name, lc.sensor_name, sensor_pose, id));
		mMapper->addExternalReading(m, robot_pose);
		mScansAdded++;
	}
	
	if(mMapPublishRate > 0 && mScansAdded % mMapPublishRate == 0)
	{
		optimize();
		generate_map();
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
