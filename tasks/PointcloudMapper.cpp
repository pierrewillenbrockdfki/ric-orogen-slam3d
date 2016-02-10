#include "PointcloudMapper.hpp"
#include "rock-common.hpp"

#include <base/samples/Pointcloud.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <slam3d/BoostMapper.hpp>
#include <slam3d/FileLogger.hpp>
#include <slam3d/G2oSolver.hpp>

#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <octomap/octomap.h>
#include <pcl/common/transforms.h>

using namespace slam3d;

PointcloudMapper::PointcloudMapper(std::string const& name)
    : PointcloudMapperBase(name)
{
	mMapCloud = NULL;
}

PointcloudMapper::PointcloudMapper(std::string const& name, RTT::ExecutionEngine* engine)
    : PointcloudMapperBase(name, engine)
{
	mMapCloud = NULL;
	mCurrentOdometry = Eigen::Affine3d::Identity();
}

PointcloudMapper::~PointcloudMapper()
{
}

bool PointcloudMapper::pause()
{
	if(state() == RUNNING)
	{
		state(PAUSED);
		mLogger->message(INFO, "Mapping is now paused.");

		// Create a new map cloud to localize against
		PointCloud::ConstPtr cloud = buildPointcloud(mMapper->getVertexObjectsFromSensor(mPclSensor->getName()));
		delete mMapCloud;
		mMapCloud = new PointCloudMeasurement(cloud, mRobotName, mPclSensor->getName(), Transform::Identity());
		mCurrentPose = mMapper->getCurrentPose();
		return true;
	}
	mLogger->message(WARNING, "Cannot pause, mapper is not running!");
	return false;
}

bool PointcloudMapper::resume()
{
	if(state() == PAUSED)
	{
		state(RUNNING);
		mLogger->message(INFO, "Mapping will now resume.");
		return true;
	}
	mLogger->message(WARNING, "Cannot resume, mapper is not paused!");
	return false;
}

bool PointcloudMapper::optimize()
{
	mLogger->message(INFO, "Requested global optimization.");
	try
	{
		boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
		if(mMapper->optimize())
		{
			sendRobotPose();
			return true;
		}
	}catch (boost::lock_error &e)
	{
		mLogger->message(WARNING, "Could not access the pose graph for optimization! Is another operation still running?");
	}
	return false;
}

bool PointcloudMapper::generate_map()
{
	// Publish accumulated cloud
	mLogger->message(INFO, "Requested map generation.");
	VertexObjectList vertices = mMapper->getVertexObjectsFromSensor(mPclSensor->getName());
	boost::thread projThread(&PointcloudMapper::sendPointcloud, this, vertices);
	return true;
}

bool PointcloudMapper::generate_octomap()
{
	// Reset OctoMap
	delete mOcTree;
	mOcTree = new octomap::OcTree(mMapResolution);

	// Project all scans to octomap
	mLogger->message(INFO, "Requested octomap generation.");
	VertexObjectList vertices = mMapper->getVertexObjectsFromSensor(mPclSensor->getName());
	boost::thread projThread(&PointcloudMapper::buildOcTree, this, vertices);
	return true;
}

void PointcloudMapper::addScanToOctoMap(const VertexObject& scan)
{
	PointCloudMeasurement* pcl = dynamic_cast<PointCloudMeasurement*>(scan.measurement);
	if(!pcl)
	{
		mLogger->message(ERROR, "Measurement is not a point cloud!");
		throw BadMeasurementType();
	}

	PointCloud::Ptr tempCloud(new PointCloud);
	pcl::transformPointCloud(*(pcl->getPointCloud()), *tempCloud, (scan.corrected_pose * pcl->getSensorPose()).matrix());

	octomap::Pointcloud octoCloud;
	for(PointCloud::iterator it = tempCloud->begin(); it < tempCloud->end(); ++it)
	{
		octoCloud.push_back(octomap::point3d(it->x, it->y,it->z));
	}
	Vector3 origin = scan.corrected_pose.translation();
	mOcTree->insertPointCloud(octoCloud, octomap::point3d(origin(0), origin(1), origin(2)), 5, true, true);
}

PointCloud::Ptr PointcloudMapper::buildPointcloud(const VertexObjectList& vertices)
{
	timeval start = mClock->now();
	PointCloud::Ptr accumulated;

	boost::shared_lock<boost::shared_mutex> guard(mGraphMutex);
	accumulated = mPclSensor->getAccumulatedCloud(vertices);

	PointCloud::Ptr downsampled = mPclSensor->downsample(accumulated, mMapResolution);
	PointCloud::Ptr accCloud = mPclSensor->removeOutliers(downsampled, mMapOutlierRadius, mMapOutlierNeighbors);

	timeval finish = mClock->now();
	int duration = finish.tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Generated Pointcloud from %1% scans in %2% seconds.") % vertices.size() % duration).str());
	return accCloud;
}

void PointcloudMapper::sendPointcloud(const VertexObjectList& vertices)
{
	PointCloud::Ptr accCloud;
	try
	{
		accCloud = buildPointcloud(vertices);
	}catch (boost::lock_error &e)
	{
		mLogger->message(ERROR, "Could not access the pose graph to build Pointcloud!");
		return;
	}
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
}

void PointcloudMapper::buildOcTree(const VertexObjectList& vertices)
{
	timeval start = mClock->now();
	try
	{
		boost::shared_lock<boost::shared_mutex> guard(mGraphMutex);
		for(VertexObjectList::const_iterator it = vertices.begin(); it != vertices.end(); it++)
		{
			addScanToOctoMap(*it);
		}
	}catch (boost::lock_error &e)
	{
		mLogger->message(ERROR, "Could not access the pose graph to build OcTree!");
		return;
	}
	mOcTree->updateInnerOccupancy();
	mOcTree->writeBinary("slam3d_octomap.bt");
	timeval finish = mClock->now();
	int duration = finish.tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Generated OcTree from %1% scans in %2% seconds.") % vertices.size() % duration).str());
}

bool PointcloudMapper::setLog_level(boost::int32_t value)
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

bool PointcloudMapper::configureHook()
{	
	if (! PointcloudMapperBase::configureHook())
		return false;
		
	mClock = new Clock();
	mLogger = new Logger(*mClock);
	setLog_level(_log_level);

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
	mMapper = new BoostMapper(mLogger);

	mLogger->message(INFO, " = GraphMapper - Parameters =");
	mLogger->message(INFO, (boost::format("use_odometry:           %1%") % _use_odometry.get()).str());	
	if(_use_odometry.get())
	{
		mOdometry = new RockOdometry(_robot2odometry, mLogger);
		mMapper->setOdometry(mOdometry, _add_odometry_edges.get());
		mLogger->message(INFO, (boost::format("add_odometry_edges:     %1%") % _add_odometry_edges.get()).str());
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

	mUseColorsAsViewpoints = _use_colors_as_viewpoints.get();
	mLogger->message(INFO, (boost::format("use_viewpoints:         %1%") % mUseColorsAsViewpoints).str());

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
	
	if(mUseColorsAsViewpoints)
	{
		if(cloud_in.colors.size() != cloud_in.points.size())
		{
			mLogger->message(WARNING, "Color vector from pointcloud has invalid size!");
			mUseColorsAsViewpoints = false;
		}
	}
	
	unsigned numPoints = cloud_in.points.size();
	for(unsigned i = 0; i < numPoints; ++i)
	{
		PointType p;
		p.x = cloud_in.points[i][0];
		p.y = cloud_in.points[i][1];
		p.z = cloud_in.points[i][2];
		
		if(mUseColorsAsViewpoints)
		{
			p.vp_x = cloud_in.colors[i][0];
			p.vp_y = cloud_in.colors[i][1];
			p.vp_z = cloud_in.colors[i][2];
		}
		
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

void PointcloudMapper::sendRobotPose()
{
	// Publish the robot pose in map
	base::samples::RigidBodyState rbs;
	rbs.setTransform(mCurrentPose);
	rbs.invalidateCovariances();
	rbs.sourceFrame = mRobotFrame;
	rbs.targetFrame = mMapFrame;
	rbs.time = mCurrentTime;
	_map2robot.write(rbs);
	
	// Publish the odometry drift
	Eigen::Affine3d drift = mCurrentPose * mCurrentOdometry.inverse();
	rbs.setTransform(drift);
	rbs.sourceFrame = mOdometryFrame;
	_map2odometry.write(rbs);
}

void PointcloudMapper::scanTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &scan_sample)
{
	++mScansReceived;
	mCurrentTime = ts;
	mCurrentOdometry = mOdometry->getOdometricPose(ts);

	// Transform base::samples::Pointcloud --> Pointcloud
	PointCloud::Ptr cloud = createFromRockMessage(scan_sample);
	
	// Downsample and add to map
	PointCloudMeasurement* measurement;
	try
	{
		if(mScanResolution > 0)
		{
			PointCloud::ConstPtr downsampled_cloud = mPclSensor->downsample(cloud, mScanResolution);
			mLogger->message(DEBUG, (boost::format("Downsampled cloud has %1% points.") % downsampled_cloud->size()).str());
			measurement = new PointCloudMeasurement(downsampled_cloud, mRobotName, mPclSensor->getName(), mPclSensor->getSensorPose());
		}else
		{
			measurement = new PointCloudMeasurement(cloud, mRobotName, mPclSensor->getName(), mPclSensor->getSensorPose());
		}
	}catch(std::exception& e)
	{
		mLogger->message(ERROR, (boost::format("Downsampling failed: %1%") % e.what()).str());
		return;
	}

	if(state() == RUNNING)
	{
		bool added = false;
		try
		{
			if(added = mMapper->addReading(measurement))
			{
				mScansAdded++;
				mNewVertices.push(mMapper->getLastVertex());
				if(mMapPublishRate > 0 && mScansAdded % mMapPublishRate == 0)
				{
					optimize();
					generate_map();
				}
			}
			mCurrentPose = mMapper->getCurrentPose();
			sendRobotPose();
		}catch(std::exception& e)
		{
			mLogger->message(ERROR, (boost::format("Adding scan to map failed: %1%") % e.what()).str());
		}
		if(!added)
		{
			delete measurement;
		}
	}else if(state() == PAUSED)
	{
		try
		{
			Transform pose;
			pose.translation() = mCurrentPose.translation();
			pose.linear() = mCurrentPose.linear();
			TransformWithCovariance twc = mPclSensor->calculateTransform(mMapCloud, measurement, pose);
			mCurrentPose = twc.transform;
			sendRobotPose();
		}catch(NoMatch)
		{
			mLogger->message(WARNING, "Could not localize in map!");
		}
		delete measurement;
	}
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
