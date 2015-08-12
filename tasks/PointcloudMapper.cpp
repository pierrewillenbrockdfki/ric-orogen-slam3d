/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PointcloudMapper.hpp"
#include "rock-common.hpp"

#include <base/samples/Pointcloud.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <slam3d/include/FileLogger.hpp>
#include <slam3d/include/G2oSolver.hpp>

#include <boost/format.hpp>

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

bool PointcloudMapper::optimize()
{
	mLogger->message(slam::INFO, "Requested global optimization.");
	if(mMapper->optimize())
	{
		sendOdometryDrift();
		sendRobotPose();
		return true;
	}else
	{
		return false;
	}
}

bool PointcloudMapper::generate_map()
{
	// Publish accumulated cloud
	mLogger->message(slam::INFO, "Requested map generation.");
	slam::VertexList vertices = mMapper->getVerticesFromSensor(mPclSensor->getName());
	slam::PointCloud::Ptr accumulated = mPclSensor->getAccumulatedCloud(vertices);
	slam::PointCloud::Ptr downsampled = mPclSensor->downsample(accumulated, mMapResolution);
	slam::PointCloud::Ptr accCloud = mPclSensor->removeOutliers(downsampled, mMapOutlierRadius, mMapOutlierNeighbors);
	
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
	return true;
}

bool PointcloudMapper::configureHook()
{	
	if (! PointcloudMapperBase::configureHook())
		return false;
		
	mClock = new slam::Clock();
	mLogger = new slam::Logger(*mClock);

//	mLogger = new BaseLogger();
	switch(_log_level.get())
	{
	case 4:
		mLogger->setLogLevel(slam::FATAL);
		break;
	case 3:
		mLogger->setLogLevel(slam::ERROR);
		break;
	case 2:
		mLogger->setLogLevel(slam::WARNING);
		break;
	case 0:
		mLogger->setLogLevel(slam::DEBUG);
		break;
	default:
		mLogger->setLogLevel(slam::INFO);
	}

	mLogger->message(slam::INFO, "=== Configure PointCloudMapper ===");

	mPclSensor = new slam::PointCloudSensor("pointcloud", mLogger, slam::Transform::Identity());
	slam::GICPConfiguration conf = _gicp_config.get();
	mPclSensor->setConfiguaration(conf);
	mLogger->message(slam::INFO, " = GICP - Parameters =");
	mLogger->message(slam::INFO, (boost::format("correspondence_randomness:    %1%") % conf.correspondence_randomness).str());
	mLogger->message(slam::INFO, (boost::format("euclidean_fitness_epsilon:    %1%") % conf.euclidean_fitness_epsilon).str());
	mLogger->message(slam::INFO, (boost::format("max_correspondence_distance:  %1%") % conf.max_correspondence_distance).str());
	mLogger->message(slam::INFO, (boost::format("max_fitness_score:            %1%") % conf.max_fitness_score).str());
	mLogger->message(slam::INFO, (boost::format("max_sensor_distance:          %1%") % conf.max_sensor_distance).str());
	mLogger->message(slam::INFO, (boost::format("maximum_iterations:           %1%") % conf.maximum_iterations).str());
	mLogger->message(slam::INFO, (boost::format("maximum_optimizer_iterations: %1%") % conf.maximum_optimizer_iterations).str());
	mLogger->message(slam::INFO, (boost::format("orientation_sigma:            %1%") % conf.orientation_sigma).str());
	mLogger->message(slam::INFO, (boost::format("point_cloud_density:          %1%") % conf.point_cloud_density).str());
	mLogger->message(slam::INFO, (boost::format("position_sigma:               %1%") % conf.position_sigma).str());
	mLogger->message(slam::INFO, (boost::format("rotation_epsilon:             %1%") % conf.rotation_epsilon).str());
	mLogger->message(slam::INFO, (boost::format("transformation_epsilon:       %1%") % conf.transformation_epsilon).str());

	mSolver = new slam::G2oSolver(mLogger);
	mMapper = new slam::GraphMapper(mLogger);
	
	if(_use_odometry.get())
	{
		mOdometry = new RockOdometry(mLogger, _odometry_time_tolerance.get());
		mMapper->setOdometry(mOdometry, _add_odometry_edges.get());
		mOdometryPose.setTransform(Eigen::Affine3d::Identity());
	}else
	{
		mOdometry = NULL;
	}
	
	mLogger->message(slam::INFO, " = GraphMapper - Parameters =");
	double min_translation = _min_translation.get();
	double min_rotation = _min_rotation.get();
	mLogger->message(slam::INFO, (boost::format("min_pose_distance:      %1% / %2%") % min_translation % min_rotation).str());
	mMapper->setMinPoseDistance(min_translation, min_rotation);
	
	double neighbor_radius = _neighbor_radius.get();
	mLogger->message(slam::INFO, (boost::format("neighbor_radius:        %1%") % neighbor_radius).str());
	mMapper->setNeighborRadius(neighbor_radius);
	
	mScanResolution = _scan_resolution.get();
	mLogger->message(slam::INFO, (boost::format("scan_resolution:        %1%") % mScanResolution).str());
	
	mMapResolution = _map_resolution.get();
	mLogger->message(slam::INFO, (boost::format("map_resolution:         %1%") % mMapResolution).str());
	
	mMapOutlierRadius = _map_outlier_radius.get();
	mLogger->message(slam::INFO, (boost::format("map_outlier_radius:     %1%") % mMapOutlierRadius).str());
	
	mMapOutlierNeighbors = _map_outlier_neighbors.get();
	mLogger->message(slam::INFO, (boost::format("map_outlier_neighbors:  %1%") % mMapOutlierNeighbors).str());
	
	mLogger->message(slam::INFO, (boost::format("use_odometry:           %1%") % _use_odometry.get()).str());
	mLogger->message(slam::INFO, (boost::format("add_odometry_edges:     %1%") % _add_odometry_edges.get()).str());
	mLogger->message(slam::INFO, (boost::format("odometry_time_tolerance:%1%") % _odometry_time_tolerance.get()).str());
	
	mMapper->registerSensor(mPclSensor);
	mMapper->setSolver(mSolver);
	
	mScansReceived = 0;
	mScansAdded = 0;
	
	return true;
}

bool PointcloudMapper::startHook()
{
	if(!PointcloudMapperBase::startHook())
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
		slam::PointCloudMeasurement* measurement;
		if(mScanResolution > 0)
		{
			slam::PointCloud::ConstPtr downsampled_cloud = mPclSensor->downsample(cloud, mScanResolution);
			mLogger->message(slam::DEBUG, (boost::format("Downsampled cloud has %1% points.") % downsampled_cloud->size()).str());
			measurement = new slam::PointCloudMeasurement(downsampled_cloud, mPclSensor->getName());
		}else
		{
			measurement = new slam::PointCloudMeasurement(cloud, mPclSensor->getName());
		}
		
		if(!mMapper->addReading(measurement))
		{
			delete measurement;
			return false;
		}
	}catch(std::exception& e)
	{
		mLogger->message(slam::ERROR, (boost::format("Downsampling failed: %1%") % e.what()).str());
		return false;
	}
	return true;
}

void PointcloudMapper::sendRobotPose()
{
	// Publish the robot pose in map
	base::samples::RigidBodyState rbs;
	Eigen::Affine3d current = mMapper->getCurrentPose();
	rbs.setTransform(current);
	rbs.invalidateCovariances();
	rbs.sourceFrame = "map";
	rbs.targetFrame = "robot";
	rbs.time = mOdometryPose.time;
	_map2robot.write(rbs);
}

void PointcloudMapper::sendOdometryDrift()
{
	// Publish the odometry drift
	base::samples::RigidBodyState rbs;
	Eigen::Affine3d current = mMapper->getCurrentPose();
	Eigen::Affine3d drift = current * mOdometryPose.getTransform().inverse();
	rbs.setTransform(drift);
	rbs.invalidateCovariances();
	rbs.sourceFrame = "map";
	rbs.targetFrame = "odometry";
	rbs.time = mOdometryPose.time;
	_map2odometry.write(rbs);
}

void PointcloudMapper::updateHook()
{
	mLogger->message(slam::DEBUG, "=== updateHook ===");
	PointcloudMapperBase::updateHook();
	
	// Read odometry data
	if(mOdometry)
	{
		int samples_read = 0;
		while(_odometry.read(mOdometryPose, false) == RTT::NewData)
		{
			++samples_read;
			mOdometry->setCurrentPose(mOdometryPose);
		}
		mLogger->message(slam::DEBUG, (boost::format("Received %1% odometry samples.") % samples_read).str());
	}
	
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
				sendOdometryDrift();
			}
		}catch (std::exception &e)
		{
			mLogger->message(slam::ERROR, (boost::format("Could not add scan: %1%") % e.what()).str());
		}
	}
	sendRobotPose();
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
