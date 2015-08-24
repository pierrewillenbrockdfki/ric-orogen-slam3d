/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SLAM3D_DISTRIBUTEDPOINTCLOUDMAPPER_TASK_HPP
#define SLAM3D_DISTRIBUTEDPOINTCLOUDMAPPER_TASK_HPP

#include "slam3d/DistributedPointcloudMapperBase.hpp"

namespace slam3d
{
	class DistributedPointcloudMapper : public DistributedPointcloudMapperBase
	{
	friend class DistributedPointcloudMapperBase;
	protected:

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
