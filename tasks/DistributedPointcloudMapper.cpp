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

bool DistributedPointcloudMapper::hasVertex(boost::uuids::uuid id) const
{
	if(mExternalMeasurements.find(id) != mExternalMeasurements.end())
		return true;
	try
	{
		mMapper->getVertex(id);
		return true;
	}catch(std::out_of_range &e)
	{
		return false;
	}
}

bool DistributedPointcloudMapper::hasEdge(const std::string& source,
                                          const std::string& target,
                                          const std::string& sensor) const
{	
	for(ConstraintList::const_iterator c = mExternalConstraints.begin(); c != mExternalConstraints.end(); ++c)
	{
		if(c->sensor_name != sensor)
			continue;
		if(c->source_unique_id == source && c->target_unique_id == target)
			return true;
		if(c->source_unique_id == target && c->target_unique_id == source)
			return true;
	}
	
	boost::uuids::uuid s_uuid = boost::lexical_cast<boost::uuids::uuid>(source);
	boost::uuids::uuid t_uuid = boost::lexical_cast<boost::uuids::uuid>(target);
	try
	{
		IdType s_id = mMapper->getVertex(s_uuid).index;
		IdType t_id = mMapper->getVertex(t_uuid).index;
		mMapper->getEdge(s_id, t_id, sensor);
		return true;
	}catch(std::out_of_range &e)
	{
		return false;
	}catch(InvalidEdge &e)
	{
		return false;
	}
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
		loc_cloud.unique_id = boost::uuids::to_string(m->getUniqueId());
		createFromPcl(m->getPointCloud(), loc_cloud.point_cloud);
		_vertex_out.write(loc_cloud);
		
		EdgeObjectList edges = mMapper->getOutEdges(v.index);
		if(edges.empty())
		{
			SpatialConstraint constr;
			constr.source_unique_id = boost::uuids::to_string(boost::uuids::nil_uuid());
			constr.target_unique_id = boost::uuids::to_string(v.measurement->getUniqueId());
			constr.sensor_name = "none";
			constr.relative_pose = base::Pose(v.corrected_pose);
			constr.covariance = base::Matrix6d::Identity();
			_edge_out.write(constr);
			continue;
		}
		
		for(EdgeObjectList::iterator e = edges.begin(); e != edges.end(); ++e)
		{
			SpatialConstraint constr;
			constr.source_unique_id = boost::uuids::to_string(mMapper->getVertex(e->source).measurement->getUniqueId());
			constr.target_unique_id = boost::uuids::to_string(mMapper->getVertex(e->target).measurement->getUniqueId());
			constr.sensor_name = e->sensor;
			constr.relative_pose = base::Pose(e->transform);
			constr.covariance = base::Matrix6d(e->covariance);
			_edge_out.write(constr);
		}
	}
	
	// Get external pointclouds
	slam3d::LocalizedPointcloud lc;
	while(_vertex_in.read(lc, false) == RTT::NewData)
	{
		boost::uuids::uuid id = boost::lexical_cast<boost::uuids::uuid>(lc.unique_id);
		if(hasVertex(id))
		{
			mLogger->message(DEBUG, (boost::format("Received '%1%:%2%', which is already present.") % lc.robot_name % lc.sensor_name).str());
			continue;
		}
		
		slam3d::Transform sensor_pose = pose2eigen(lc.sensor_pose);
		slam3d::PointCloud::Ptr cloud = createFromRockMessage(lc.point_cloud);
		slam3d::PointCloudMeasurement::Ptr m(new slam3d::PointCloudMeasurement(cloud, lc.robot_name, lc.sensor_name, sensor_pose, id));
		mExternalMeasurements.insert(MeasurementMap::value_type(m->getUniqueId(), m));
	}
	
	// Get external constraints
	slam3d::SpatialConstraint c;
	while(_edge_in.read(c, false) == RTT::NewData)
	{
		if(hasEdge(c.source_unique_id, c.target_unique_id, c.sensor_name))
		{
			mLogger->message(DEBUG, (boost::format("Received %1%-edge, which is already present.") % c.sensor_name).str());
			continue;
		}
		mExternalConstraints.push_back(c);
	}

	try
	{
		addExternals();
	}
	catch(std::exception &e)
	{
		mLogger->message(ERROR, e.what());
	}
}

void DistributedPointcloudMapper::addExternals()
{
	// Try to add all external measurements and constraints collected so far
	bool update = false;
	for(ConstraintList::iterator c = mExternalConstraints.begin(); c != mExternalConstraints.end();)
	{
		boost::uuids::uuid s_id = boost::lexical_cast<boost::uuids::uuid>(c->source_unique_id);
		boost::uuids::uuid t_id = boost::lexical_cast<boost::uuids::uuid>(c->target_unique_id);
		slam3d::Transform relative_pose = pose2eigen(c->relative_pose);
		
		try
		{
			// Case 1: Graph -> Graph
			mMapper->addExternalConstraint(s_id, t_id, relative_pose, c->covariance, c->sensor_name);
			c = mExternalConstraints.erase(c);
		}catch(DuplicateEdge &de)
		{
			mLogger->message(DEBUG, de.what());
			c = mExternalConstraints.erase(c);
		}catch (std::out_of_range &e)
		{
			try
			{
				// Case 2: Graph -> External
				PointCloudMeasurement::Ptr m = mExternalMeasurements.at(t_id);
				mMapper->addExternalReading(m, s_id, relative_pose, c->covariance, c->sensor_name);
				mExternalMeasurements.erase(t_id);
				c = mExternalConstraints.erase(c);
				if((++mScansAdded % mMapPublishRate) == 0) update = true;
			}catch(DuplicateMeasurement &dm)
			{
				mLogger->message(DEBUG, dm.what());
				c = mExternalConstraints.erase(c);
			}catch (std::out_of_range &e)
			{
				try
				{
					// Case 3: External -> Graph
					PointCloudMeasurement::Ptr m = mExternalMeasurements.at(s_id);
					mMapper->addExternalReading(m, t_id, relative_pose.inverse(), c->covariance, c->sensor_name);
					mExternalMeasurements.erase(s_id);
					c = mExternalConstraints.erase(c);
					if((++mScansAdded % mMapPublishRate) == 0) update = true;
					
				}catch(DuplicateMeasurement &dm)
				{
					mLogger->message(DEBUG, dm.what());
					c = mExternalConstraints.erase(c);
				}catch (std::out_of_range &e)
				{
					// Case 4: Both not in graph, leave this edge for now
					++c;
				}
			}
		}
	}
	
	int left = mExternalConstraints.size();
	if(left > 0)
	{
		mLogger->message(DEBUG, (boost::format("There are %1% unmatched external constraints left.") % left).str());
	}
	
	if((mMapPublishRate > 0) && update)
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
