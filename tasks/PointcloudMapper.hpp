/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SLAM3D_POINTCLOUDMAPPER_TASK_HPP
#define SLAM3D_POINTCLOUDMAPPER_TASK_HPP

#include "slam3d/PointcloudMapperBase.hpp"

#include <slam3d/include/GraphMapper.hpp>
#include <slam3d/include/PointCloudSensor.hpp>
#include <slam3d/include/Solver.hpp>

namespace slam3d
{
	class RockOdometry;
	
	class PointcloudMapper : public PointcloudMapperBase
	{
	friend class PointcloudMapperBase;
	protected:

		// Operations
		virtual bool optimize();
		virtual bool generate_map();
		
		// Callbacks
		virtual void scanTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &scan_sample);

		// Internal methods
		bool processPointcloud(const base::samples::Pointcloud& cloud);
		void sendRobotPose();
		void sendOdometryDrift();
	
		// Members
		slam::Clock* mClock;
		slam::Logger* mLogger;
		slam::GraphMapper* mMapper;
		slam::PointCloudSensor* mPclSensor;
		slam::Solver* mSolver;
		RockOdometry* mOdometry;
		
		base::samples::RigidBodyState mOdometryPose;

		int mScansReceived;
		int mScansAdded;
		
		// Parameters for creation of map-pcl
		double mScanResolution;
		double mMapResolution;
		
		// Parameters for outlier rejection in map-pcl
		double   mMapOutlierRadius;
		unsigned mMapOutlierNeighbors;

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

