#include "PointcloudFilter.hpp"

#include <base-logging/Logging.hpp>
#include <base/samples/Pointcloud.hpp>

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

void PointcloudFilter::inputTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &input_sample)
{
	// Get laser pose
	Eigen::Affine3d laser2odom;
	try
	{
		_laser2odometry.get(ts, laser2odom, true);
	}catch(std::exception &e)
	{
		LOG_ERROR("%s", e.what());
	}
	
	// Add to accumulated cloud and octree
	mScanCount++;
	octomap::Pointcloud octoCloud;
	for(std::vector<base::Vector3d>::const_iterator it = input_sample.points.begin(); it < input_sample.points.end(); ++it)
	{
		double sq_dist = ((*it)[0] * (*it)[0]) + ((*it)[1] * (*it)[1]) + ((*it)[2] * (*it)[2]);
		if((*it)[2] < mMaxHeight && (*it)[2] > mMinHeight && sq_dist < mSqMaxDistance && sq_dist > mSqMinDistance)
		{
			base::Vector3d v = laser2odom * (*it);
			mPointcloud.push_back(PointType(v[0], v[1], v[2]));
			octoCloud.push_back(octomap::point3d(v[0], v[1], v[2]));
		}
	}
	mOcTree->insertPointCloud(octoCloud, octomap::point3d(), mOctoConfig.rangeMax, true, true);
	
	if(mScanCount < 20)
		return;
	
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
	float leaf_size = mResolution;
	PointCloud::Ptr downsampled(new PointCloud);
	pcl::VoxelGrid<PointType> grid;
	grid.setLeafSize (leaf_size, leaf_size, leaf_size);
	grid.setInputCloud(filtered_cloud);
	grid.filter(*downsampled);
	
	// Transform to current pose
	base::samples::Pointcloud result;
	for(PointCloud::iterator p = downsampled->begin(); p != downsampled->end(); ++p)
	{
		base::Vector3d vec = laser2odom.inverse() * base::Vector3d(p->x, p->y, p->z);
		result.points.push_back(vec);
	}
	result.time = ts;
	_output.write(result);
	mOcTree->writeBinary("slam3d_filter_result.bt");
	
	// Reset everything
	delete mOcTree;
	mOcTree = new octomap::OcTree(mResolution);
	mOcTree->setOccupancyThres(mOctoConfig.occupancyThres);
	mOcTree->setProbHit(mOctoConfig.probHit);
	mOcTree->setProbMiss(mOctoConfig.probMiss);
	mOcTree->setClampingThresMin(mOctoConfig.clampingThresMin);
	mOcTree->setClampingThresMax(mOctoConfig.clampingThresMax);
}

bool PointcloudFilter::configureHook()
{
	if (! PointcloudFilterBase::configureHook())
		return false;
	
	mMinHeight = _min_height.get();
	mMaxHeight = _max_height.get();
	mSqMinDistance = _min_distance.get() * _min_distance.get();
	mSqMaxDistance = _max_distance.get() * _max_distance.get();
	mOctoConfig = _octo_map_config.get();
	mResolution = _resolution.get();
	
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
	PointcloudFilterBase::cleanupHook();
}
