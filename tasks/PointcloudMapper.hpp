/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SLAM3D_POINTCLOUDMAPPER_TASK_HPP
#define SLAM3D_POINTCLOUDMAPPER_TASK_HPP

#include "slam3d/PointcloudMapperBase.hpp"

#include <slam3d/include/GraphMapper.hpp>
#include <slam3d/include/PointCloudSensor.hpp>
#include <slam3d/include/Solver.hpp>

namespace slam3d
{
	class PointcloudMapper : public PointcloudMapperBase
	{
	friend class PointcloudMapperBase;
	protected:
	
		bool processPointcloud(const base::samples::Pointcloud& cloud);
	
		slam::Clock* mClock;
		slam::Logger* mLogger;
		slam::GraphMapper* mMapper;
		slam::PointCloudSensor* mPclSensor;
		slam::Solver* mSolver;

		int mScansReceived;
		int mScansAdded;
		
		double mScanResolution;

	public:
		PointcloudMapper(std::string const& name = "slam3d::PointcloudMapper");
		PointcloudMapper(std::string const& name, RTT::ExecutionEngine* engine);
		~PointcloudMapper();

		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

