/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PointcloudMapper.hpp"
#include "rock-common.hpp"

#include <base/samples/Pointcloud.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <slam3d/include/FileLogger.hpp>
#include <slam3d/include/G2oSolver.hpp>

using namespace slam3d;

PointcloudMapper::PointcloudMapper(std::string const& name)
    : PointcloudMapperBase(name)
{
}

PointcloudMapper::PointcloudMapper(std::string const& name, RTT::ExecutionEngine* engine)
    : PointcloudMapperBase(name, engine)
{
}

PointcloudMapper::~PointcloudMapper()
{
}

bool PointcloudMapper::configureHook()
{
	if (! PointcloudMapperBase::configureHook())
		return false;
		
	mClock = new slam::Clock();
	mLogger = new BaseLogger();
	mLogger->setLogLevel(slam::DEBUG);

	mPclSensor = new slam::PointCloudSensor("pointcloud", mLogger);
	mPclSensor->setConfiguaration(_gicp_config.get());

	mSolver = new slam::G2oSolver(mLogger);

	mMapper = new slam::GraphMapper(mLogger);
	mMapper->setNeighborRadius(_neighbor_radius.get());
	mMapper->registerSensor(mPclSensor);
	mMapper->setSolver(mSolver);
	
	mScansReceived = 0;
	mScansAdded = 0;
	
	// Get parameters
	mScanResolution = _scan_resolution.get();
	
	return true;
}

bool PointcloudMapper::startHook()
{
    if (! PointcloudMapperBase::startHook())
        return false;
    return true;
}

bool PointcloudMapper::processPointcloud(const base::samples::Pointcloud& cloud_in)
{
	// Transform base::samples::Pointcloud --> slam::Pointcloud
	slam::PointCloud::Ptr cloud(new slam::PointCloud);
	cloud->header.stamp = cloud_in.time.toMicroseconds();
	for(std::vector<base::Vector3d>::const_iterator it = cloud_in.points.begin(); it < cloud_in.points.end(); ++it)
	{
		slam::PointType p;
		p.x = (*it)[0];
		p.y = (*it)[1];
		p.z = (*it)[2];
		cloud->push_back(p);
	}
	
	// Downsample and add to map
	try
	{
		slam::PointCloud::ConstPtr downsampled_cloud = mPclSensor->downsample(cloud, mScanResolution);
		LOG_DEBUG("Downsampled cloud has %d points.", downsampled_cloud->size());
		slam::PointCloudMeasurement* measurement = new slam::PointCloudMeasurement(cloud, mPclSensor->getName());
		mMapper->addReading(measurement);
	}catch(std::exception& e)
	{
		LOG_ERROR("Downsampling failed: %s", e.what());
		return false;
	}
	return true;
}

void PointcloudMapper::updateHook()
{
	LOG_DEBUG("=== updateHook ===");
    PointcloudMapperBase::updateHook();
	
	// Read the scan from the port
	base::samples::Pointcloud cloud;
	while(_scan.read(cloud, false) == RTT::NewData)
	{
		++mScansReceived;
		try
		{
			if(processPointcloud(cloud))
			{
				mScansAdded++;
			}else
			{
				LOG_WARN("Scan was not added to map!");
			}
		}catch (std::exception &e)
		{
			LOG_ERROR("Could not add scan: %s", e.what());
		}
	}
	
	// Publish the robot pose in map
	base::samples::RigidBodyState rbs;
	Eigen::Affine3d aff = mMapper->getCurrentPose();
	rbs.setTransform(aff);
	rbs.invalidateCovariances();
	rbs.sourceFrame = "map";
	rbs.targetFrame = "robot";
	rbs.time = cloud.time;
	_map2robot.write(rbs);
	
	// Optimize
//	mMapper->optimize();

	// Publish accumulated cloud
	slam::VertexList vertices = mMapper->getVerticesFromSensor(mPclSensor->getName());
	slam::PointCloud::Ptr accCloud = mPclSensor->getAccumulatedCloud(vertices, 0.05);
	
	base::samples::Pointcloud mapCloud;
	for(slam::PointCloud::iterator it = accCloud->begin(); it < accCloud->end(); ++it)
	{
		base::Vector3d vec;
		vec[0] = it->x;
		vec[1] = it->y;
		vec[2] = it->z;
		mapCloud.points.push_back(vec);
	}
	mapCloud.time = base::Time::fromMicroseconds(accCloud->header.stamp);
	_cloud.write(mapCloud);
}

void PointcloudMapper::errorHook()
{
    PointcloudMapperBase::errorHook();
}

void PointcloudMapper::stopHook()
{
    PointcloudMapperBase::stopHook();
}

void PointcloudMapper::cleanupHook()
{
    PointcloudMapperBase::cleanupHook();
	delete mMapper;
	delete mPclSensor;
	delete mSolver;
	delete mLogger;
	delete mClock;
}
