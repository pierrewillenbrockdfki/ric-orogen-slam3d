/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SLAM3D_DISTRIBUTEDPOINTCLOUDMAPPER_TASK_HPP
#define SLAM3D_DISTRIBUTEDPOINTCLOUDMAPPER_TASK_HPP

#include "slam3d/DistributedPointcloudMapperBase.hpp"

namespace slam3d
{
	typedef std::map<boost::uuids::uuid, PointCloudMeasurement::Ptr> MeasurementMap;
	typedef std::vector<SpatialConstraint> ConstraintList;
	
	class DistributedPointcloudMapper : public DistributedPointcloudMapperBase
	{
	friend class DistributedPointcloudMapperBase;

	protected:
		MeasurementMap mExternalMeasurements;
		ConstraintList mExternalConstraints;
		
	protected:
		bool hasVertex(boost::uuids::uuid id) const;
		bool hasEdge(const std::string& source, const std::string& target, const std::string& sensor) const;

	public:
		DistributedPointcloudMapper(std::string const& name = "slam3d::DistributedPointcloudMapper");
		DistributedPointcloudMapper(std::string const& name, RTT::ExecutionEngine* engine);
		~DistributedPointcloudMapper();
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif
