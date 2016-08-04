#ifndef SLAM3D_POINTCLOUDFILTER_TASK_HPP
#define SLAM3D_POINTCLOUDFILTER_TASK_HPP

#include "slam3d/PointcloudFilterBase.hpp"

#include <base/samples/Pointcloud.hpp>
#include <octomap/OcTree.h>
#include <slam3d/PointCloudSensor.hpp>

#include "OctoMapConfiguration.hpp"

namespace slam3d
{
	class PointcloudFilter : public PointcloudFilterBase
	{
	friend class PointcloudFilterBase;
	
	protected:
	
		// Callbacks
		virtual void inputTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &scan_sample);

		// Members
		slam3d::PointCloud mPointcloud;
		octomap::OcTree* mOcTree;
		OctoMapConfiguration mOctoConfig;
		unsigned mScanCount;

		// Parameters
		double mMinHeight;
		double mMaxHeight;
		double mSqMinDistance;
		double mSqMaxDistance;
		double mResolution;

	public:
		PointcloudFilter(std::string const& name = "slam3d::PointcloudFilter");
		PointcloudFilter(std::string const& name, RTT::ExecutionEngine* engine);
		~PointcloudFilter();
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

