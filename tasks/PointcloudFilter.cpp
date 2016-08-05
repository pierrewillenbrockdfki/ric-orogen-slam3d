#include "PointcloudFilter.hpp"
#include "BaseLogger.hpp"
#include "Common.hpp"

#include <base/samples/Pointcloud.hpp>
#include <slam3d/FileLogger.hpp>

#include <pcl/filters/voxel_grid.h>

using namespace slam3d;

PointcloudFilter::PointcloudFilter(std::string const& name)
 : PointcloudFilterBase(name)
{
}

PointcloudFilter::PointcloudFilter(std::string const& name, RTT::ExecutionEngine* engine)
 : PointcloudFilterBase(name, engine)
{
}

PointcloudFilter::~PointcloudFilter()
{
}

bool PointcloudFilter::setLog_level(boost::int32_t value)
{
	switch(value)
	{
	case 4:
		mLogger->setLogLevel(FATAL);
		break;
	case 3:
		mLogger->setLogLevel(ERROR);
		break;
	case 2:
		mLogger->setLogLevel(WARNING);
		break;
	case 0:
		mLogger->setLogLevel(DEBUG);
		break;
	default:
		mLogger->setLogLevel(INFO);
	}
	return true;
}

PointCloud::Ptr PointcloudFilter::downsample(PointCloud::Ptr source, float leaf_size)
{
	PointCloud::Ptr downsampled(new PointCloud);
	pcl::VoxelGrid<PointType> grid;
	grid.setLeafSize (leaf_size, leaf_size, leaf_size);
	grid.setInputCloud(source);
	grid.filter(*downsampled);
	return downsampled;
}

void PointcloudFilter::inputTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &input_sample)
{
	// Get laser pose
	timeval start = mClock->now();
	Eigen::Affine3d laser2odom;
	try
	{
		_laser2odometry.get(ts, laser2odom, true);
	}catch(std::exception &e)
	{
		mLogger->message(WARNING, e.what());
	}
	
	// Add to accumulated cloud and octree
	octomap::Pointcloud octoCloud;
	PointCloud::Ptr t1(new PointCloud);
	for(std::vector<base::Vector3d>::const_iterator it = input_sample.points.begin(); it < input_sample.points.end(); ++it)
	{
		double sq_dist = ((*it)[0] * (*it)[0]) + ((*it)[1] * (*it)[1]) + ((*it)[2] * (*it)[2]);
		if((*it)[2] < mMaxHeight && (*it)[2] > mMinHeight && sq_dist < mSqMaxDistance && sq_dist > mSqMinDistance)
		{
			t1->push_back(PointType((*it)[0], (*it)[1], (*it)[2]));
		}
	}
	
	PointCloud::Ptr t2 = downsample(t1, mResolution);
	for(PointCloud::iterator p = t2->begin(); p != t2->end(); ++p)
	{
		base::Vector3d v = laser2odom * base::Vector3d(p->x, p->y, p->z);
		mPointcloud.push_back(PointType(v[0], v[1], v[2]));
		octoCloud.push_back(octomap::point3d(v[0], v[1], v[2]));
	}
	
	mOcTree->insertPointCloud(octoCloud, octomap::point3d(), mOctoConfig.rangeMax, true, true);
	
	// Accumulate scans before updating
	mScanCount++;
	if(mScanCount < mPassRate)
	{
		mLogger->message(DEBUG, (boost::format("Added scan in %1% ms.") % timevaldiff(start, mClock->now())).str());
		return;
	}
	mScanCount = 0;
	
	// Check each point, if it is in free OctoMap voxel
	PointCloud::Ptr filtered_cloud(new PointCloud);
	for(PointCloud::iterator p = mPointcloud.begin(); p != mPointcloud.end(); ++p)
	{
		octomap::OcTreeNode* node = mOcTree->search(p->x, p->y, p->z);
		if(!node || mOcTree->isNodeOccupied(node))
		{
			filtered_cloud->push_back(*p);
		}
	}
	
	// Downsample
	PointCloud::Ptr downsampled = downsample(filtered_cloud, mResolution);
	
	// Transform to current pose
	base::samples::Pointcloud result;
	for(PointCloud::iterator p = downsampled->begin(); p != downsampled->end(); ++p)
	{
		base::Vector3d vec = laser2odom.inverse() * base::Vector3d(p->x, p->y, p->z);
		result.points.push_back(vec);
	}
	result.time = ts;
	_output.write(result);

	// Write octree binary to view with 'octovis'
	if(_log_level.get() == 0)
	{
		mOcTree->writeBinary("slam3d_filter_result.bt");
	}
	
	// Reset everything
	delete mOcTree;
	mOcTree = new octomap::OcTree(mResolution);
	mOcTree->setOccupancyThres(mOctoConfig.occupancyThres);
	mOcTree->setProbHit(mOctoConfig.probHit);
	mOcTree->setProbMiss(mOctoConfig.probMiss);
	mOcTree->setClampingThresMin(mOctoConfig.clampingThresMin);
	mOcTree->setClampingThresMax(mOctoConfig.clampingThresMax);
	
	mPointcloud.clear();
	mLogger->message(DEBUG, (boost::format("Created output in %1% ms.") % timevaldiff(start, mClock->now())).str());
}

bool PointcloudFilter::configureHook()
{
	if (! PointcloudFilterBase::configureHook())
		return false;
	
	mClock = new Clock();
	switch(_log_type)
	{
	case 0:
		mLogger = new Logger(*mClock);
		break;
	case 1:
		mLogger = new BaseLogger();
		break;
	case 2:
		mLogger = new FileLogger(*mClock, "slam3d_filter.log");
		break;
	default:
		mLogger = new Logger(*mClock);
		mLogger->message(WARNING, "Invalid logger type, using standard logger.");
	}

	setLog_level(_log_level);
	
	mMinHeight = _min_height.get();
	mMaxHeight = _max_height.get();
	mSqMinDistance = _min_distance.get() * _min_distance.get();
	mSqMaxDistance = _max_distance.get() * _max_distance.get();
	mOctoConfig = _octo_map_config.get();
	mResolution = _resolution.get();
	mPassRate = _pass_rate.get();
	
	mOcTree = new octomap::OcTree(mResolution);
	mOcTree->setOccupancyThres(mOctoConfig.occupancyThres);
	mOcTree->setProbHit(mOctoConfig.probHit);
	mOcTree->setProbMiss(mOctoConfig.probMiss);
	mOcTree->setClampingThresMin(mOctoConfig.clampingThresMin);
	mOcTree->setClampingThresMax(mOctoConfig.clampingThresMax);
	
	mScanCount = 0;
	return true;
}

bool PointcloudFilter::startHook()
{
	if (! PointcloudFilterBase::startHook())
		return false;
	return true;
}

void PointcloudFilter::updateHook()
{
	PointcloudFilterBase::updateHook();
}

void PointcloudFilter::errorHook()
{
	PointcloudFilterBase::errorHook();
}

void PointcloudFilter::stopHook()
{
	PointcloudFilterBase::stopHook();
}

void PointcloudFilter::cleanupHook()
{
	delete mLogger;
	delete mClock;
	PointcloudFilterBase::cleanupHook();
}
