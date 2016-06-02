#ifndef SLAM3D_POINTCLOUDMAPPER_TASK_HPP
#define SLAM3D_POINTCLOUDMAPPER_TASK_HPP

#include <slam3d/PointcloudMapperBase.hpp>
#include <slam3d/GraphMapper.hpp>
#include <slam3d/PointCloudSensor.hpp>
#include <slam3d/Solver.hpp>

#include <envire/maps/MLSGrid.hpp>
#include <envire/operators/MLSProjection.hpp>

#include <queue>
#include <boost/thread/shared_mutex.hpp>

namespace slam3d
{	
	class RockOdometry;
	
	class PointcloudMapper : public PointcloudMapperBase
	{
	friend class PointcloudMapperBase;
	protected:

		// Operations
		virtual bool generate_cloud();
		virtual bool generate_map();
		virtual bool optimize();
		virtual bool write_graph();
		
		// Callbacks
		virtual void scanTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &scan_sample);
		virtual bool setLog_level(boost::int32_t value);

		// Internal methods
		slam3d::PointCloud::Ptr createFromRockMessage(const base::samples::Pointcloud& cloud);
		void createFromPcl(slam3d::PointCloud::ConstPtr pcl_cloud, base::samples::Pointcloud& base_cloud);
		void sendRobotPose();
		PointCloud::Ptr buildPointcloud(const VertexObjectList& vertices);
		void sendPointcloud(const VertexObjectList& vertices);
		virtual void handleNewScan(const VertexObject& scan);
		virtual void addScanToMap(const VertexObject& scan);
		virtual void rebuildMap(const VertexObjectList& vertices);
		virtual void sendMap();
	
		// Members
		slam3d::Clock* mClock;
		slam3d::Logger* mLogger;
		slam3d::GraphMapper* mMapper;
		slam3d::PointCloudSensor* mPclSensor;
		slam3d::Solver* mSolver;
		RockOdometry* mOdometry;
		boost::shared_mutex mGraphMutex;

		std::string mRobotName;
		std::string mRobotFrame;
		std::string mOdometryFrame;
		std::string mMapFrame;
		
		bool mUseColorsAsViewpoints;
		
		int mScansReceived;
		int mScansAdded;
		int mOptimizationRate;
		int mMapPublishRate;
		
		// Parameters for creation of map-pcl
		double mScanResolution;
		double mMapResolution;
		
		// Parameters for outlier rejection in map-pcl
		double   mMapOutlierRadius;
		unsigned mMapOutlierNeighbors;
		
		// Parameters for creation of MLS
		envire::Environment mEnvironment;
		envire::MLSGrid* mMultiLayerMap;
		
		double mGridSizeX;
		double mGridSizeY;
		double mGridOffsetX;
		double mGridOffsetY;
		double mGridMinZ;
		double mGridMaxZ;
		double mGridResolution;
		
		bool mRebuildMap;
		
		// Current state of transformations
		base::Time mCurrentTime;
		Eigen::Affine3d mCurrentPose;
		Eigen::Affine3d mCurrentOdometry;
		Eigen::Affine3d mLastOdometry;

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
