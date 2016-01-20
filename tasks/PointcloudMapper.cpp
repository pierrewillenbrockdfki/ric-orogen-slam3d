/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PointcloudMapper.hpp"
#include "rock-common.hpp"

#include <base/samples/Pointcloud.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <slam3d/FileLogger.hpp>
#include <slam3d/G2oSolver.hpp>

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
	mLogger->message(INFO, "Requested global optimization.");
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
	mLogger->message(INFO, "Requested map generation.");
	VertexList vertices = mMapper->getVerticesFromSensor(mPclSensor->getName());
	PointCloud::Ptr accumulated = mPclSensor->getAccumulatedCloud(vertices);
	PointCloud::Ptr downsampled = mPclSensor->downsample(accumulated, mMapResolution);
	PointCloud::Ptr accCloud = mPclSensor->removeOutliers(downsampled, mMapOutlierRadius, mMapOutlierNeighbors);
	
	base::samples::Pointcloud mapCloud;
	for(PointCloud::iterator it = accCloud->begin(); it < accCloud->end(); ++it)
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

bool PointcloudMapper::generate_octomap()
{
	std::string name("map_");
	name += mClock->now().tv_sec;
	name += ".bt";
	mOcTree->writeBinary(name.c_str());
	return true;
}

bool PointcloudMapper::configureHook()
{	
	if (! PointcloudMapperBase::configureHook())
		return false;
		
	mClock = new Clock();
	mLogger = new Logger(*mClock);

	switch(_log_level.get())
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

	mLogger->message(INFO, "=== Configure PointCloudMapper ===");

	mPclSensor = new PointCloudSensor("pointcloud", mLogger, Transform::Identity());
	GICPConfiguration conf = _gicp_config.get();
	mPclSensor->setConfiguaration(conf);
	mLogger->message(INFO, " = GICP - Parameters =");
	mLogger->message(INFO, (boost::format("correspondence_randomness:    %1%") % conf.correspondence_randomness).str());
	mLogger->message(INFO, (boost::format("euclidean_fitness_epsilon:    %1%") % conf.euclidean_fitness_epsilon).str());
	mLogger->message(INFO, (boost::format("max_correspondence_distance:  %1%") % conf.max_correspondence_distance).str());
	mLogger->message(INFO, (boost::format("max_fitness_score:            %1%") % conf.max_fitness_score).str());
	mLogger->message(INFO, (boost::format("max_sensor_distance:          %1%") % conf.max_sensor_distance).str());
	mLogger->message(INFO, (boost::format("maximum_iterations:           %1%") % conf.maximum_iterations).str());
	mLogger->message(INFO, (boost::format("maximum_optimizer_iterations: %1%") % conf.maximum_optimizer_iterations).str());
	mLogger->message(INFO, (boost::format("orientation_sigma:            %1%") % conf.orientation_sigma).str());
	mLogger->message(INFO, (boost::format("point_cloud_density:          %1%") % conf.point_cloud_density).str());
	mLogger->message(INFO, (boost::format("position_sigma:               %1%") % conf.position_sigma).str());
	mLogger->message(INFO, (boost::format("rotation_epsilon:             %1%") % conf.rotation_epsilon).str());
	mLogger->message(INFO, (boost::format("transformation_epsilon:       %1%") % conf.transformation_epsilon).str());

	mSolver = new G2oSolver(mLogger);
	mMapper = new GraphMapper(mLogger);

	mLogger->message(INFO, " = GraphMapper - Parameters =");
	mLogger->message(INFO, (boost::format("use_odometry:           %1%") % _use_odometry.get()).str());	
	if(_use_odometry.get())
	{
		mOdometry = new RockOdometry(mLogger, _odometry_time_tolerance.get());
		mMapper->setOdometry(mOdometry, _add_odometry_edges.get());
		mOdometryPose.setTransform(Eigen::Affine3d::Identity());
		mLogger->message(INFO, (boost::format("add_odometry_edges:     %1%") % _add_odometry_edges.get()).str());
		mLogger->message(INFO, (boost::format("odometry_time_tolerance:%1%") % _odometry_time_tolerance.get()).str());
	}else
	{
		mOdometry = NULL;
	}
	
	double min_translation = _min_translation.get();
	double min_rotation = _min_rotation.get();
	mLogger->message(INFO, (boost::format("min_pose_distance:      %1% / %2%") % min_translation % min_rotation).str());
	mMapper->setMinPoseDistance(min_translation, min_rotation);
	
	double neighbor_radius = _neighbor_radius.get();
	mLogger->message(INFO, (boost::format("neighbor_radius:        %1%") % neighbor_radius).str());
	int max_neighbor_links = _max_neighbor_links.get();
	mLogger->message(INFO, (boost::format("max_neighbor_links:     %1%") % max_neighbor_links).str());
	mMapper->setNeighborRadius(neighbor_radius, max_neighbor_links);
	
	mScanResolution = _scan_resolution.get();
	mLogger->message(INFO, (boost::format("scan_resolution:        %1%") % mScanResolution).str());
	
	mMapResolution = _map_resolution.get();
	mLogger->message(INFO, (boost::format("map_resolution:         %1%") % mMapResolution).str());
	
	mMapPublishRate = _map_publish_rate.get();
	mLogger->message(INFO, (boost::format("map_publish_rate:       %1%") % mMapPublishRate).str());
	
	mMapOutlierRadius = _map_outlier_radius.get();
	mLogger->message(INFO, (boost::format("map_outlier_radius:     %1%") % mMapOutlierRadius).str());
	
	mMapOutlierNeighbors = _map_outlier_neighbors.get();
	mLogger->message(INFO, (boost::format("map_outlier_neighbors:  %1%") % mMapOutlierNeighbors).str());
	
	mRobotName = _robot_name.get();
	mLogger->message(INFO, (boost::format("robot_name:             %1%") % mRobotName).str());
	
	mRobotFrame = _robot_frame.get();
	mLogger->message(INFO, (boost::format("robot_frame:            %1%") % mRobotFrame).str());
	
	mOdometryFrame = _odometry_frame.get();
	mLogger->message(INFO, (boost::format("odometry_frame:         %1%") % mOdometryFrame).str());

	mMapFrame = _map_frame.get();
	mLogger->message(INFO, (boost::format("map_frame:              %1%") % mMapFrame).str());

	mMapper->registerSensor(mPclSensor);
	mMapper->setSolver(mSolver);
	
	mScansReceived = 0;
	mScansAdded = 0;
	
	mOcTree = new octomap::OcTree(mMapResolution);
	return true;
}

bool PointcloudMapper::startHook()
{
	if(!PointcloudMapperBase::startHook())
		return false;
	return true;
}

PointCloud::Ptr PointcloudMapper::createFromRockMessage(const base::samples::Pointcloud& cloud_in)
{
	PointCloud::Ptr cloud_out(new PointCloud);
	cloud_out->header.stamp = cloud_in.time.toMicroseconds();
	for(std::vector<base::Vector3d>::const_iterator it = cloud_in.points.begin(); it < cloud_in.points.end(); ++it)
	{
		PointType p;
		p.x = (*it)[0];
		p.y = (*it)[1];
		p.z = (*it)[2];
		cloud_out->push_back(p);
	}
	return cloud_out;
}

void PointcloudMapper::createFromPcl(PointCloud::ConstPtr pcl_cloud, base::samples::Pointcloud& base_cloud)
{
	base_cloud.time.fromMicroseconds(pcl_cloud->header.stamp);
	for(PointCloud::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
	{
		base::Point p;
		p[0] = it->x;
		p[1] = it->y;
		p[2] = it->z;
		base_cloud.points.push_back(p);
	}
}

bool PointcloudMapper::processPointcloud(const base::samples::Pointcloud& cloud_in)
{
	// Transform base::samples::Pointcloud --> Pointcloud
	PointCloud::Ptr cloud = createFromRockMessage(cloud_in);
	
	// Downsample and add to map
	try
	{
		PointCloudMeasurement* measurement;
		if(mScanResolution > 0)
		{
			PointCloud::ConstPtr downsampled_cloud = mPclSensor->downsample(cloud, mScanResolution);
			mLogger->message(DEBUG, (boost::format("Downsampled cloud has %1% points.") % downsampled_cloud->size()).str());
			measurement = new PointCloudMeasurement(downsampled_cloud, mRobotName, mPclSensor->getName(), mPclSensor->getSensorPose());
		}else
		{
			measurement = new PointCloudMeasurement(cloud, mRobotName, mPclSensor->getName(), mPclSensor->getSensorPose());
		}
		
		if(!mMapper->addReading(measurement))
		{
			delete measurement;
			return false;
		}
		mNewVertices.push(mMapper->getLastVertex());
	}catch(std::exception& e)
	{
		mLogger->message(ERROR, (boost::format("Downsampling failed: %1%") % e.what()).str());
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
	rbs.sourceFrame = mRobotFrame;
	rbs.targetFrame = mMapFrame;
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
	rbs.sourceFrame = mOdometryFrame;
	rbs.targetFrame = mMapFrame;
	rbs.time = mOdometryPose.time;
	_map2odometry.write(rbs);
}

void PointcloudMapper::scanTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &scan_sample)
{
	++mScansReceived;
	
	// Read odometry data
	if(mOdometry)
	{
		// Rock calls "odometry -> robot" [robot2odometry]
		Eigen::Affine3d odom;
		if(!_robot2odometry.get(ts, odom, true))
		{
			mLogger->message(ERROR, "Odometry not available!");
			return;
		}
		mOdometryPose.setTransform(odom);
		mOdometryPose.time = ts;
		mOdometry->setCurrentPose(mOdometryPose);
	}

	try
	{
		if(processPointcloud(scan_sample))
		{
			mScansAdded++;
			if(mMapPublishRate > 0 && mScansAdded % mMapPublishRate == 0)
			{
				optimize();
				generate_map();
			}
		}
	}catch (std::exception &e)
	{
		mLogger->message(ERROR, (boost::format("Could not add scan: %1%") % e.what()).str());
	}
	sendOdometryDrift();
	sendRobotPose();
}

void PointcloudMapper::updateHook()
{
	PointcloudMapperBase::updateHook();
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
	delete mOcTree;
	delete mMapper;
	delete mPclSensor;
	delete mSolver;
	delete mLogger;
	delete mClock;
}
