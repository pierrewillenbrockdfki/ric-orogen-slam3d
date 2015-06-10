/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Mapper.hpp"
#include "rock-common.hpp"

#include <velodyne_lidar/MultilevelLaserScan.h>
#include <velodyne_lidar/pointcloudConvertHelper.hpp>

#include <base/samples/Pointcloud.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <slam3d/include/FileLogger.hpp>
#include <slam3d/include/G2oSolver.hpp>

using namespace slam3d;

Mapper::Mapper(std::string const& name)
    : MapperBase(name)
{
}

Mapper::Mapper(std::string const& name, RTT::ExecutionEngine* engine)
    : MapperBase(name, engine)
{
}

Mapper::~Mapper()
{
}

bool Mapper::configureHook()
{
	if (! MapperBase::configureHook())
		return false;
		
	mClock = new slam::Clock();
	mLogger = new BaseLogger();
//	mLogger = new slam::Logger(*mClock);
//	mLogger = new slam::FileLogger(*mClock, "slam3d.log");
	mLogger->setLogLevel(slam::DEBUG);

	mMapper = new slam::GraphMapper(mLogger);
	mPclSensor = new slam::PointCloudSensor("pointcloud", mLogger);
	slam::GICPConfiguration conf;
	conf.max_correspondence_distance = 4.0;
	conf.max_fitness_score = 20;
	conf.maximum_iterations = 200;
	mPclSensor->setConfiguaration(conf);
	mMapper->registerSensor(mPclSensor);
	mMapper->setNeighborRadius(2.0);
	
	mSolver = new slam::G2oSolver(mLogger);
	mMapper->setSolver(mSolver);
	
	mScansReceived = 0;
	mScansAdded = 0;
	
	return true;
}

bool Mapper::startHook()
{
	if (! MapperBase::startHook())
		return false;
	return true;
}

void Mapper::updateHook()
{
	LOG_DEBUG("=== updateHook ===");
	MapperBase::updateHook();
	
	// Read the scan from the port
	velodyne_lidar::MultilevelLaserScan scan;
	while(_scan.read(scan, false) == RTT::NewData)
	{
		++mScansReceived;
//		if(mScansReceived < 600 || mScansReceived % 4 == 0)
//			continue;
		
		try
		{
			if(processScan(scan))
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

	// TODO: check timestamping from scan
	rbs.time = base::Time::now();
	_map2robot.write(rbs);
	

	if(mScansAdded % 10 != 0)
	{
		return;
	}

	// Optimize
	mMapper->optimize();

	// Publish accumulated cloud
	slam::VertexList vertices = mMapper->getVerticesFromSensor(mPclSensor->getName());
	slam::PointCloud::Ptr accCloud = mPclSensor->getAccumulatedCloud(vertices, 0.25);
	
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

bool Mapper::processScan(const velodyne_lidar::MultilevelLaserScan& scan)
{	
	// Convert to PointCloud
	std::vector<Eigen::Vector3d> points;
	velodyne_lidar::ConvertHelper::convertScanToPointCloud(scan, points);
	if(points.size() == 0)
	{
		LOG_ERROR("Convertion to pointcloud returned no points!");
		return false;
	}else
	{
		LOG_DEBUG("Converted to pointcloud with %d points.", points.size());
	}
	slam::PointCloud::Ptr cloud(new slam::PointCloud);
	cloud->header.stamp = scan.time.toMicroseconds();
	for(std::vector<Eigen::Vector3d>::iterator it = points.begin(); it < points.end(); ++it)
	{
		slam::PointType p;
		p.x = (*it)[0];
		p.y = (*it)[1];
		p.z = (*it)[2];
		cloud->push_back(p);
	}
	
	try
	{
		slam::PointCloud::ConstPtr downsampled_cloud = mPclSensor->downsample(cloud, 0.5);
		LOG_DEBUG("Downsampled cloud has %d points.", downsampled_cloud->size());
		slam::PointCloudMeasurement* measurement = new slam::PointCloudMeasurement(downsampled_cloud, mPclSensor->getName());
		mMapper->addReading(measurement);
	}catch(std::exception& e)
	{
		LOG_ERROR("Downsampling failed: %s", e.what());
		return false;
	}
	return true;
}

void Mapper::errorHook()
{
	MapperBase::errorHook();
}

void Mapper::stopHook()
{
	MapperBase::stopHook();
}

void Mapper::cleanupHook()
{
	MapperBase::cleanupHook();
	delete mMapper;
	delete mPclSensor;
	delete mSolver;
	delete mLogger;
	delete mClock;
}
